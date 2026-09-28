#include "gps_mcp_tool.h"
#include "application.h"
#include <esp_log.h>

#define TAG "GpsMcpTool"

GpsMcpTool::GpsMcpTool(std::shared_ptr<Ml307Gps> gps) : gps_(gps) {
}

void GpsMcpTool::Initialize() {
    auto& mcp_server = McpServer::GetInstance();
    mcp_server.AddTool("self.gps_control",
        "Control GPS positioning module. Actions:\n"
        "- \"enable\": Turn on GPS and start reporting position every 30 seconds\n"
        "- \"disable\": Turn off GPS module\n"
        "- \"query\": Get current GPS position (latitude, longitude, altitude, speed)",
        PropertyList({
            Property("action", kPropertyTypeString)
        }),
        [this](const PropertyList& properties) -> ReturnValue {
            return HandleGpsControl(properties);
        });

    ESP_LOGI(TAG, "GpsMcpTool initialized");
}

ReturnValue GpsMcpTool::HandleGpsControl(const PropertyList& properties) {
    auto action = properties["action"].value<std::string>();

    if (action == "enable") {
        if (gps_->IsEnabled()) {
            return std::string("GPS is already enabled");
        }
        bool ok = gps_->Enable([this](const GpsData& data) {
            Application::GetInstance().SendTelemetry(data.ToJson());
        });
        if (ok) {
            ESP_LOGI(TAG, "GPS enabled, starting position reporting");
            return std::string("GPS has been enabled. Position will be reported every 30 seconds.");
        }
        return std::string("Failed to enable GPS module");
    }

    if (action == "disable") {
        if (!gps_->IsEnabled()) {
            return std::string("GPS is already disabled");
        }
        gps_->Disable();
        ESP_LOGI(TAG, "GPS disabled");
        return std::string("GPS has been disabled.");
    }

    if (action == "query") {
        auto data = gps_->QueryPosition();
        if (!data.valid) {
            return std::string("GPS position is not available. GPS may be disabled or has no satellite fix.");
        }
        return data.ToJson();
    }

    return std::string("Invalid GPS action: " + action + ". Use \"enable\", \"disable\", or \"query\".");
}
