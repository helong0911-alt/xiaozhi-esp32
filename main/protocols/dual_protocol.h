#ifndef DUAL_PROTOCOL_H
#define DUAL_PROTOCOL_H


#include "protocol.h"
#include <mqtt.h>
#include <web_socket.h>
#include <cJSON.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <esp_timer.h>

#include <functional>
#include <string>
#include <memory>
#include <atomic>
#include <mutex>

#define DUAL_MQTT_RECONNECT_INTERVAL_MS 60000

#define DUAL_WS_HELLO_EVENT (1 << 0)

class DualProtocol : public Protocol {
public:
    DualProtocol();
    ~DualProtocol();

    bool Start() override;
    bool SendAudio(std::unique_ptr<AudioStreamPacket> packet) override;
    bool OpenAudioChannel() override;
    void CloseAudioChannel(bool send_goodbye = true) override;
    bool IsAudioChannelOpened() const override;

private:
    std::shared_ptr<std::atomic<bool>> alive_ = std::make_shared<std::atomic<bool>>(true);

    EventGroupHandle_t event_group_handle_;

    // MQTT control plane
    std::string mqtt_publish_topic_;
    std::unique_ptr<Mqtt> mqtt_;
    esp_timer_handle_t reconnect_timer_;
    mutable std::mutex mqtt_mutex_;

    // WS voice data plane
    std::unique_ptr<WebSocket> websocket_;
    int ws_version_ = 1;

    bool StartMqttClient();
    void ParseWsServerHello(const cJSON* root);
    std::string GetWsHelloMessage();
    void PublishDeviceStatus();

    bool SendText(const std::string& text) override;
};


#endif // DUAL_PROTOCOL_H
