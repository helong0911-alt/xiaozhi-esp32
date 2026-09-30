#include "dual_protocol.h"
#include "application.h"
#include "board.h"
#include "mcp_server.h"
#include "settings.h"
#include "system_info.h"

#include <esp_log.h>
#include <arpa/inet.h>
#include <cJSON.h>
#include <cstring>
#include "assets/lang_config.h"

#define TAG "DUAL"

DualProtocol::DualProtocol() {
    event_group_handle_ = xEventGroupCreate();

    esp_timer_create_args_t reconnect_timer_args = {
        .callback =
            [](void* arg) {
                DualProtocol* protocol = (DualProtocol*)arg;
                auto& app = Application::GetInstance();
                if (app.GetDeviceState() == kDeviceStateIdle) {
                    ESP_LOGI(TAG, "Reconnecting to MQTT server");
                    auto alive = protocol->alive_;
                    app.Schedule([protocol, alive]() {
                        if (*alive) {
                            protocol->StartMqttClient();
                        }
                    });
                }
            },
        .arg = this,
    };
    esp_timer_create(&reconnect_timer_args, &reconnect_timer_);
}

DualProtocol::~DualProtocol() {
    ESP_LOGI(TAG, "DualProtocol deinit");

    *alive_ = false;

    if (reconnect_timer_ != nullptr) {
        esp_timer_stop(reconnect_timer_);
        esp_timer_delete(reconnect_timer_);
    }

    websocket_.reset();

    {
        std::lock_guard<std::mutex> lock(mqtt_mutex_);
        mqtt_.reset();
    }

    if (event_group_handle_ != nullptr) {
        vEventGroupDelete(event_group_handle_);
    }
}

bool DualProtocol::Start() {
    return StartMqttClient();
}

bool DualProtocol::StartMqttClient() {
    {
        std::lock_guard<std::mutex> lock(mqtt_mutex_);
        if (mqtt_ != nullptr) {
            ESP_LOGW(TAG, "MQTT client already started");
            mqtt_.reset();
        }
    }

    Settings settings("mqtt", false);
    auto endpoint = settings.GetString("endpoint");
    auto username = settings.GetString("username");
    auto password = settings.GetString("password");
    int keepalive_interval = settings.GetInt("keepalive", 240);
    mqtt_publish_topic_ = settings.GetString("publish_topic");
    auto subscribe_topic = settings.GetString("subscribe_topic");

    if (endpoint.empty()) {
        ESP_LOGW(TAG, "MQTT endpoint is not specified");
        return false;
    }

    auto network = Board::GetInstance().GetNetwork();
    auto mqtt = network->CreateMqtt(0);
    mqtt->SetKeepAlive(keepalive_interval);

    mqtt->OnDisconnected([this]() {
        if (on_disconnected_ != nullptr) {
            on_disconnected_();
        }
        ESP_LOGI(TAG, "MQTT disconnected, schedule reconnect in %d seconds",
                 DUAL_MQTT_RECONNECT_INTERVAL_MS / 1000);
        esp_timer_start_once(reconnect_timer_, DUAL_MQTT_RECONNECT_INTERVAL_MS * 1000);
    });

    mqtt->OnConnected([this]() {
        if (on_connected_ != nullptr) {
            on_connected_();
        }
        esp_timer_stop(reconnect_timer_);
    });

    mqtt->OnMessage([this](const std::string& topic, const std::string& payload) {
        cJSON* root = cJSON_Parse(payload.c_str());
        if (root == nullptr) {
            ESP_LOGE(TAG, "Failed to parse MQTT message: %s", payload.c_str());
            return;
        }

        cJSON* type = cJSON_GetObjectItem(root, "type");
        if (cJSON_IsString(type)) {
            if (strcmp(type->valuestring, "goodbye") == 0) {
                auto session_id = cJSON_GetObjectItem(root, "session_id");
                ESP_LOGI(TAG, "Received MQTT goodbye, session_id: %s",
                         cJSON_IsString(session_id) ? session_id->valuestring : "null");
                if (cJSON_IsString(session_id) && session_id_ == session_id->valuestring) {
                    auto alive = alive_;
                    Application::GetInstance().Schedule([this, alive]() {
                        if (*alive) {
                            CloseAudioChannel(false);
                        }
                    });
                }
            } else if (strcmp(type->valuestring, "mcp") == 0) {
                // Handle MCP messages via MQTT with MQTT-specific response sender
                auto mcp_payload = cJSON_GetObjectItem(root, "payload");
                if (cJSON_IsObject(mcp_payload)) {
                    auto alive = alive_;
                    McpServer::ResponseSender mqtt_response_sender =
                        [this, alive](const std::string& response_payload) {
                            if (!*alive) return;
                            std::string message = "{\"session_id\":\"" + session_id_ +
                                                  "\",\"type\":\"mcp\",\"payload\":" +
                                                  response_payload + "}";
                            std::lock_guard<std::mutex> lock(mqtt_mutex_);
                            if (mqtt_ != nullptr && !mqtt_publish_topic_.empty()) {
                                mqtt_->Publish(mqtt_publish_topic_, message);
                            }
                        };
                    McpServer::GetInstance().ParseMessage(mcp_payload, mqtt_response_sender);
                }
            } else if (on_incoming_json_ != nullptr) {
                on_incoming_json_(root);
            }
        } else if (on_incoming_json_ != nullptr) {
            on_incoming_json_(root);
        }
        cJSON_Delete(root);
        last_incoming_time_ = std::chrono::steady_clock::now();
    });

    std::string broker_address;
    int broker_port = 9003;
    size_t pos = endpoint.find(':');
    if (pos != std::string::npos) {
        broker_address = endpoint.substr(0, pos);
        auto port_str = endpoint.substr(pos + 1);
        char* end = nullptr;
        long parsed_port = strtol(port_str.c_str(), &end, 10);
        if (end != port_str.c_str() && parsed_port > 0 && parsed_port <= 65535) {
            broker_port = static_cast<int>(parsed_port);
        } else {
            ESP_LOGW(TAG, "Invalid port in MQTT endpoint \"%s\", using %d", endpoint.c_str(), broker_port);
        }
    } else {
        broker_address = endpoint;
    }

    ESP_LOGI(TAG, "Connecting to MQTT: %s:%d", broker_address.c_str(), broker_port);
    auto client_id = Board::GetInstance().GetUuid();
    if (auto connected = mqtt->Connect(broker_address, broker_port, client_id, username, password);
        !connected) {
        ESP_LOGE(TAG, "Failed to connect to MQTT: %s", connected.error().ToString().c_str());
        SetError(Lang::Strings::SERVER_NOT_CONNECTED,
                 broker_address + ":" + std::to_string(broker_port));
        return false;
    }

    ESP_LOGI(TAG, "MQTT connected");

    {
        std::lock_guard<std::mutex> lock(mqtt_mutex_);
        mqtt_ = std::move(mqtt);
    }

    // Subscribe to command topic for server-to-device messages
    if (!subscribe_topic.empty()) {
        std::lock_guard<std::mutex> lock(mqtt_mutex_);
        if (mqtt_ != nullptr) {
            mqtt_->Subscribe(subscribe_topic);
            ESP_LOGI(TAG, "Subscribed to %s", subscribe_topic.c_str());
        }
    }

    // Publish presence hello
    cJSON* hello = cJSON_CreateObject();
    cJSON_AddStringToObject(hello, "type", "hello");
    cJSON* data = cJSON_CreateObject();
    cJSON_AddStringToObject(data, "device_id", SystemInfo::GetMacAddress().c_str());
    cJSON_AddStringToObject(data, "client_id", client_id.c_str());
    cJSON_AddItemToObject(hello, "data", data);
    auto json_str = cJSON_PrintUnformatted(hello);
    std::string hello_msg(json_str);
    cJSON_free(json_str);
    cJSON_Delete(hello);

    {
        std::lock_guard<std::mutex> lock(mqtt_mutex_);
        if (mqtt_ != nullptr) {
            mqtt_->Publish(mqtt_publish_topic_, hello_msg);
        }
    }

    ESP_LOGI(TAG, "MQTT presence announced on %s", mqtt_publish_topic_.c_str());
    return true;
}

bool DualProtocol::SendText(const std::string& text) {
    if (websocket_ == nullptr || !websocket_->IsConnected()) {
        ESP_LOGW(TAG, "WebSocket not connected, cannot send text");
        return false;
    }
    if (!websocket_->Send(text)) {
        ESP_LOGE(TAG, "Failed to send text via WebSocket: %s", text.c_str());
        SetError(Lang::Strings::SERVER_ERROR);
        return false;
    }
    return true;
}

bool DualProtocol::SendAudio(std::unique_ptr<AudioStreamPacket> packet) {
    if (websocket_ == nullptr || !websocket_->IsConnected()) {
        return false;
    }

    if (ws_version_ == 2) {
        std::string serialized;
        serialized.resize(sizeof(BinaryProtocol2) + packet->payload.size());
        auto bp2 = (BinaryProtocol2*)serialized.data();
        bp2->version = htons(ws_version_);
        bp2->type = 0;
        bp2->reserved = 0;
        bp2->timestamp = htonl(packet->timestamp);
        bp2->payload_size = htonl(packet->payload.size());
        memcpy(bp2->payload, packet->payload.data(), packet->payload.size());
        return websocket_->Send(serialized.data(), serialized.size(), true);
    } else if (ws_version_ == 3) {
        std::string serialized;
        serialized.resize(sizeof(BinaryProtocol3) + packet->payload.size());
        auto bp3 = (BinaryProtocol3*)serialized.data();
        bp3->type = 0;
        bp3->reserved = 0;
        bp3->payload_size = htons(packet->payload.size());
        memcpy(bp3->payload, packet->payload.data(), packet->payload.size());
        return websocket_->Send(serialized.data(), serialized.size(), true);
    } else {
        return websocket_->Send(packet->payload.data(), packet->payload.size(), true);
    }
}

bool DualProtocol::IsAudioChannelOpened() const {
    return websocket_ != nullptr && websocket_->IsConnected() && !error_occurred_ && !IsTimeout();
}

void DualProtocol::CloseAudioChannel(bool send_goodbye) {
    if (send_goodbye && websocket_ != nullptr && websocket_->IsConnected()) {
        cJSON* root = cJSON_CreateObject();
        cJSON_AddStringToObject(root, "type", "goodbye");
        cJSON_AddStringToObject(root, "session_id", session_id_.c_str());
        auto json_str = cJSON_PrintUnformatted(root);
        std::string msg(json_str);
        cJSON_free(json_str);
        cJSON_Delete(root);
        websocket_->Send(msg);
    }

    websocket_.reset();
}

bool DualProtocol::OpenAudioChannel() {
    Settings settings("websocket", false);
    std::string url = settings.GetString("url");
    std::string token = settings.GetString("token");
    int version = settings.GetInt("version");
    if (version != 0) {
        ws_version_ = version;
    }

    error_occurred_ = false;
    session_id_ = "";
    xEventGroupClearBits(event_group_handle_, DUAL_WS_HELLO_EVENT);

    auto network = Board::GetInstance().GetNetwork();
    websocket_ = network->CreateWebSocket(1);
    if (websocket_ == nullptr) {
        ESP_LOGE(TAG, "Failed to create WebSocket");
        return false;
    }

    if (!token.empty()) {
        if (token.find(" ") == std::string::npos) {
            token = "Bearer " + token;
        }
        websocket_->SetHeader("Authorization", token.c_str());
    }
    websocket_->SetHeader("Protocol-Version", std::to_string(ws_version_).c_str());
    websocket_->SetHeader("Device-Id", SystemInfo::GetMacAddress().c_str());
    websocket_->SetHeader("Client-Id", Board::GetInstance().GetUuid().c_str());

    websocket_->OnData([this](const char* data, size_t len, bool binary) {
        if (binary) {
            if (on_incoming_audio_ != nullptr) {
                if (ws_version_ == 2) {
                    BinaryProtocol2* bp2 = (BinaryProtocol2*)data;
                    bp2->version = ntohs(bp2->version);
                    bp2->type = ntohs(bp2->type);
                    bp2->timestamp = ntohl(bp2->timestamp);
                    bp2->payload_size = ntohl(bp2->payload_size);
                    auto payload = (uint8_t*)bp2->payload;
                    on_incoming_audio_(std::make_unique<AudioStreamPacket>(AudioStreamPacket{
                        .sample_rate = server_sample_rate_,
                        .frame_duration = server_frame_duration_,
                        .timestamp = bp2->timestamp,
                        .payload = std::vector<uint8_t>(payload, payload + bp2->payload_size)}));
                } else if (ws_version_ == 3) {
                    BinaryProtocol3* bp3 = (BinaryProtocol3*)data;
                    bp3->type = bp3->type;
                    bp3->payload_size = ntohs(bp3->payload_size);
                    auto payload = (uint8_t*)bp3->payload;
                    on_incoming_audio_(std::make_unique<AudioStreamPacket>(AudioStreamPacket{
                        .sample_rate = server_sample_rate_,
                        .frame_duration = server_frame_duration_,
                        .timestamp = 0,
                        .payload = std::vector<uint8_t>(payload, payload + bp3->payload_size)}));
                } else {
                    on_incoming_audio_(std::make_unique<AudioStreamPacket>(AudioStreamPacket{
                        .sample_rate = server_sample_rate_,
                        .frame_duration = server_frame_duration_,
                        .timestamp = 0,
                        .payload = std::vector<uint8_t>((uint8_t*)data, (uint8_t*)data + len)}));
                }
            }
        } else {
            auto root = cJSON_ParseWithLength(data, len);
            if (root == nullptr) {
                ESP_LOGE(TAG, "Failed to parse WS JSON message");
                return;
            }
            auto type = cJSON_GetObjectItem(root, "type");
            if (cJSON_IsString(type)) {
                if (strcmp(type->valuestring, "hello") == 0) {
                    ParseWsServerHello(root);
                } else if (strcmp(type->valuestring, "mcp") == 0) {
                    // Handle MCP messages via WebSocket with WS-specific response sender
                    auto mcp_payload = cJSON_GetObjectItem(root, "payload");
                    if (cJSON_IsObject(mcp_payload)) {
                        McpServer::ResponseSender ws_response_sender =
                            [this](const std::string& response_payload) {
                                std::string message = "{\"session_id\":\"" + session_id_ +
                                                      "\",\"type\":\"mcp\",\"payload\":" +
                                                      response_payload + "}";
                                if (websocket_ != nullptr && websocket_->IsConnected()) {
                                    websocket_->Send(message);
                                }
                            };
                        McpServer::GetInstance().ParseMessage(mcp_payload, ws_response_sender);
                    }
                } else {
                    if (on_incoming_json_ != nullptr) {
                        on_incoming_json_(root);
                    }
                }
            } else {
                ESP_LOGE(TAG, "Missing message type in WS data: %s", std::string(data, len).c_str());
            }
            cJSON_Delete(root);
        }
        last_incoming_time_ = std::chrono::steady_clock::now();
    });

    websocket_->OnDisconnected([this]() {
        ESP_LOGI(TAG, "WebSocket disconnected");
        if (on_audio_channel_closed_ != nullptr) {
            on_audio_channel_closed_();
        }
    });

    ESP_LOGI(TAG, "Connecting to WebSocket: %s with version: %d", url.c_str(), ws_version_);
    if (auto connected = websocket_->Connect(url.c_str()); !connected) {
        ESP_LOGE(TAG, "Failed to connect to WebSocket: %s",
                 connected.error().ToString().c_str());
        SetError(Lang::Strings::SERVER_NOT_CONNECTED, url);
        return false;
    }

    auto message = GetWsHelloMessage();
    if (!websocket_->Send(message)) {
        return false;
    }

    EventBits_t bits =
        xEventGroupWaitBits(event_group_handle_, DUAL_WS_HELLO_EVENT, pdTRUE,
                            pdFALSE, pdMS_TO_TICKS(10000));
    if (!(bits & DUAL_WS_HELLO_EVENT)) {
        ESP_LOGE(TAG, "Failed to receive WebSocket server hello");
        SetError(Lang::Strings::SERVER_TIMEOUT);
        return false;
    }

    if (on_audio_channel_opened_ != nullptr) {
        on_audio_channel_opened_();
    }

    return true;
}

std::string DualProtocol::GetWsHelloMessage() {
    cJSON* root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "type", "hello");
    cJSON_AddNumberToObject(root, "version", ws_version_);
    cJSON* features = cJSON_CreateObject();
#if CONFIG_USE_SERVER_AEC
    cJSON_AddBoolToObject(features, "aec", true);
#endif
    cJSON_AddBoolToObject(features, "mcp", true);
    cJSON_AddItemToObject(root, "features", features);
    AddTextFontCapabilities(root);
    cJSON_AddStringToObject(root, "transport", "websocket");
    cJSON* audio_params = cJSON_CreateObject();
    cJSON_AddStringToObject(audio_params, "format", "opus");
    cJSON_AddNumberToObject(audio_params, "sample_rate", 16000);
    cJSON_AddNumberToObject(audio_params, "channels", 1);
    cJSON_AddNumberToObject(audio_params, "frame_duration", OPUS_FRAME_DURATION_MS);
    cJSON_AddItemToObject(root, "audio_params", audio_params);
    auto json_str = cJSON_PrintUnformatted(root);
    std::string message(json_str);
    cJSON_free(json_str);
    cJSON_Delete(root);
    return message;
}

void DualProtocol::ParseWsServerHello(const cJSON* root) {
    auto transport = cJSON_GetObjectItem(root, "transport");
    if (!cJSON_IsString(transport)) {
        ESP_LOGE(TAG, "Missing or non-string transport in server hello");
        return;
    }
    if (strcmp(transport->valuestring, "websocket") != 0) {
        ESP_LOGE(TAG, "Unsupported transport: %s", transport->valuestring);
        return;
    }

    auto session_id = cJSON_GetObjectItem(root, "session_id");
    if (cJSON_IsString(session_id)) {
        session_id_ = session_id->valuestring;
        ESP_LOGI(TAG, "Session ID: %s", session_id_.c_str());
    }

    auto audio_params = cJSON_GetObjectItem(root, "audio_params");
    if (cJSON_IsObject(audio_params)) {
        auto sample_rate = cJSON_GetObjectItem(audio_params, "sample_rate");
        if (cJSON_IsNumber(sample_rate)) {
            server_sample_rate_ = sample_rate->valueint;
        }
        auto frame_duration = cJSON_GetObjectItem(audio_params, "frame_duration");
        if (cJSON_IsNumber(frame_duration)) {
            server_frame_duration_ = frame_duration->valueint;
        }
    }

    xEventGroupSetBits(event_group_handle_, DUAL_WS_HELLO_EVENT);
}
