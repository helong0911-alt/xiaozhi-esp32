#include "ml307_gps.h"
#include <esp_log.h>
#include <cstring>
#include <cstdlib>
#include <sstream>
#include <vector>

#define TAG "Ml307Gps"

static const int kPollTimeoutMs = 5000;

std::string GpsData::ToJson() const {
    char buf[256];
    snprintf(buf, sizeof(buf),
        "{\"valid\":%s,\"latitude\":%.6f,\"longitude\":%.6f,\"altitude\":%.1f,\"speed\":%.1f}",
        valid ? "true" : "false", latitude, longitude, altitude, speed);
    return std::string(buf);
}

Ml307Gps::Ml307Gps(std::shared_ptr<AtUart> at_uart, int interval_seconds)
    : at_uart_(at_uart), interval_seconds_(interval_seconds) {
}

Ml307Gps::~Ml307Gps() {
    Disable();
}

bool Ml307Gps::Enable(GpsDataCallback callback) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (enabled_.load()) {
            ESP_LOGW(TAG, "GPS already enabled");
            return true;
        }
    }

    // Power on GPS module (outside lock to avoid blocking other threads)
    std::string response;
    if (!SendGpsCommand("AT+CGPS=1", response)) {
        ESP_LOGE(TAG, "Failed to enable GPS, response: %s", response.c_str());
        return false;
    }

    // Check for ERROR in response
    if (response.find("ERROR") != std::string::npos) {
        ESP_LOGE(TAG, "GPS enable rejected: %s", response.c_str());
        return false;
    }

    // Wait for GPS to initialize
    vTaskDelay(pdMS_TO_TICKS(2000));

    {
        std::lock_guard<std::mutex> lock(mutex_);
        callback_ = std::move(callback);
        enabled_.store(true);
    }

    // Start polling task
    BaseType_t ret = xTaskCreate(
        [](void* arg) {
            auto* self = static_cast<Ml307Gps*>(arg);
            self->PollTask();
            self->poll_task_ = nullptr;
            vTaskDelete(nullptr);
        },
        "gps_poll",
        4096,
        this,
        5,
        &poll_task_
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create GPS poll task");
        std::lock_guard<std::mutex> lock(mutex_);
        enabled_.store(false);
        SendGpsCommand("AT+CGPS=0", response);
        return false;
    }

    ESP_LOGI(TAG, "GPS enabled, polling interval: %d seconds", interval_seconds_);
    return true;
}

void Ml307Gps::Disable() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!enabled_.load()) {
            return;
        }
        enabled_.store(false);
    }

    // Wait for poll task to exit. It checks enabled_ every 100ms during sleep,
    // but might be blocked in SendGpsCommand (up to 5s timeout).
    // Wait up to 6 seconds for clean exit.
    for (int i = 0; i < 60; i++) {
        vTaskDelay(pdMS_TO_TICKS(100));
        if (poll_task_ == nullptr) {
            break;
        }
    }
    poll_task_ = nullptr;

    // Power off GPS module
    std::string response;
    SendGpsCommand("AT+CGPS=0", response);

    ESP_LOGI(TAG, "GPS disabled");
}

GpsData Ml307Gps::QueryPosition() {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_data_;
}

void Ml307Gps::PollTask() {
    // Wait for initial GPS fix, checking enabled_ flag periodically
    for (int i = 0; i < 50 && enabled_.load(); i++) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    while (enabled_.load()) {
        GpsData data;
        std::string response;

        if (SendGpsCommand("AT+CGPSINFO", response)) {
            if (ParseGpsInfo(response, data)) {
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    last_data_ = data;
                }
                if (data.valid && callback_) {
                    callback_(data);
                }
                if (data.valid) {
                    ESP_LOGD(TAG, "GPS: lat=%.6f, lon=%.6f, alt=%.1f, speed=%.1f",
                        data.latitude, data.longitude, data.altitude, data.speed);
                }
            }
        }

        // Sleep for interval
        for (int i = 0; i < interval_seconds_ * 10 && enabled_.load(); i++) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

bool Ml307Gps::SendGpsCommand(const std::string& command, std::string& response) {
    if (!at_uart_->SendCommand(command, kPollTimeoutMs)) {
        ESP_LOGE(TAG, "Failed to send command: %s", command.c_str());
        return false;
    }
    response = at_uart_->GetResponse();
    return true;
}

bool Ml307Gps::ParseGpsInfo(const std::string& response, GpsData& data) {
    // Response format: +CGPSINFO: <lat>,<N/S>,<lon>,<E/W>,<date>,<time>,<alt>,<speed>,<course>
    // Example: +CGPSINFO: 2232.7648,N,11403.5612,E,120526,081234.0,50.0,0.5,120.3
    //          ddmm.mmmm       ddmm.mmmm

    auto pos = response.find("+CGPSINFO:");
    if (pos == std::string::npos) {
        // No fix or error
        data.valid = false;
        return true;
    }

    std::string info = response.substr(pos + strlen("+CGPSINFO:"));
    // Trim leading/trailing whitespace
    auto start = info.find_first_not_of(" \r\n\t");
    if (start == std::string::npos) {
        data.valid = false;
        return true;
    }
    info = info.substr(start);

    // Check if GPS has no fix (empty or all zeros)
    if (info.empty() || info.find(",,,,") != std::string::npos) {
        data.valid = false;
        return true;
    }

    // Parse comma-separated fields
    std::vector<std::string> fields;
    std::stringstream ss(info);
    std::string field;
    while (std::getline(ss, field, ',')) {
        fields.push_back(field);
    }

    if (fields.size() < 8) {
        ESP_LOGW(TAG, "Incomplete GPS data: %s", response.c_str());
        data.valid = false;
        return true;
    }

    // Parse latitude
    if (!fields[0].empty() && !fields[1].empty()) {
        data.latitude = NmeaToDecimal(fields[0], true);
        if (fields[1] == "S") {
            data.latitude = -data.latitude;
        }
    }

    // Parse longitude
    if (!fields[2].empty() && !fields[3].empty()) {
        data.longitude = NmeaToDecimal(fields[2], false);
        if (fields[3] == "W") {
            data.longitude = -data.longitude;
        }
    }

    // Parse altitude (field 6)
    if (!fields[6].empty()) {
        data.altitude = std::atof(fields[6].c_str());
    }

    // Parse speed in km/h (field 7, unit is km/h for ML307)
    if (!fields[7].empty()) {
        data.speed = std::atof(fields[7].c_str());
    }

    data.valid = (data.latitude != 0.0 || data.longitude != 0.0);
    return true;
}

double Ml307Gps::NmeaToDecimal(const std::string& coord, bool is_latitude) {
    // NMEA format: ddmm.mmmm (latitude) or dddmm.mmmm (longitude)
    // Latitude: 2 digits degrees, rest minutes
    // Longitude: 3 digits degrees, rest minutes
    int degree_digits = is_latitude ? 2 : 3;

    if ((int)coord.length() <= degree_digits + 1) {
        return 0.0;
    }

    double degrees = std::atof(coord.substr(0, degree_digits).c_str());
    double minutes = std::atof(coord.substr(degree_digits).c_str());

    return degrees + minutes / 60.0;
}
