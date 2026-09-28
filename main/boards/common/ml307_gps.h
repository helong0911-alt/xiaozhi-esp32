#ifndef _ML307_GPS_H_
#define _ML307_GPS_H_

#include <at_modem.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <functional>
#include <mutex>
#include <string>
#include <atomic>

struct GpsData {
    double latitude = 0.0;
    double longitude = 0.0;
    double altitude = 0.0;
    double speed = 0.0;  // km/h
    bool valid = false;

    std::string ToJson() const;
};

class Ml307Gps {
public:
    using GpsDataCallback = std::function<void(const GpsData& data)>;

    Ml307Gps(std::shared_ptr<AtUart> at_uart, int interval_seconds = 30);
    ~Ml307Gps();

    /// Enable GPS module and start periodic reporting
    bool Enable(GpsDataCallback callback);

    /// Disable GPS module and stop reporting
    void Disable();

    /// Query current GPS position (blocking, returns immediately with last known data)
    GpsData QueryPosition();

    /// Check if GPS is enabled
    bool IsEnabled() const { return enabled_.load(); }

private:
    std::shared_ptr<AtUart> at_uart_;
    int interval_seconds_;
    std::atomic<bool> enabled_{false};
    TaskHandle_t poll_task_ = nullptr;
    std::mutex mutex_;
    GpsData last_data_;
    GpsDataCallback callback_;

    void PollTask();
    bool SendGpsCommand(const std::string& command, std::string& response);
    bool ParseGpsInfo(const std::string& response, GpsData& data);
    static double NmeaToDecimal(const std::string& coord, bool is_latitude);
};

#endif  // _ML307_GPS_H_
