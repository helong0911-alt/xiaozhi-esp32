#include "i2c_device.h"

#include <esp_log.h>
#include <stdlib.h>
#include <string.h>

#define TAG "I2cDevice"

I2cDevice::I2cDevice(i2c_master_bus_handle_t i2c_bus, uint8_t addr)
    : i2c_bus_(i2c_bus), device_address_(addr) {
    i2c_device_config_t i2c_device_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = 400 * 1000,
        .scl_wait_us = 0,
        .flags =
            {
                .disable_ack_check = 0,
            },
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &i2c_device_cfg, &i2c_device_));
    assert(i2c_device_ != NULL);
}

void I2cDevice::WriteReg(uint8_t reg, uint8_t value) {
    TryWriteReg(reg, value);
}

bool I2cDevice::TryWriteReg(uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = {reg, value};
    esp_err_t err = i2c_master_transmit(i2c_device_, buffer, 2, 100);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WriteReg failed: addr=0x%02x, reg=0x%02x, err=%s",
                 device_address_, reg, esp_err_to_name(err));
        return false;
    }
    return true;
}

void I2cDevice::WriteRegs(uint8_t reg, const uint8_t* buffer, size_t length) {
    TryWriteRegs(reg, buffer, length);
}

bool I2cDevice::TryWriteRegs(uint8_t reg, const uint8_t* buffer, size_t length) {
    uint8_t* tx = (uint8_t*)malloc(length + 1);
    if (tx == nullptr) {
        ESP_LOGE(TAG, "WriteRegs failed: malloc returned nullptr");
        return false;
    }
    tx[0] = reg;
    memcpy(tx + 1, buffer, length);
    esp_err_t err = i2c_master_transmit(i2c_device_, tx, length + 1, 100);
    free(tx);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WriteRegs failed: addr=0x%02x, reg=0x%02x, err=%s",
                 device_address_, reg, esp_err_to_name(err));
        return false;
    }
    return true;
}

uint8_t I2cDevice::ReadReg(uint8_t reg) {
    uint8_t value = 0;
    if (!TryReadReg(reg, value)) {
        return 0;
    }
    return value;
}

bool I2cDevice::TryReadReg(uint8_t reg, uint8_t& out_value) {
    out_value = 0;
    esp_err_t err = i2c_master_transmit_receive(i2c_device_, &reg, 1, &out_value, 1, 100);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ReadReg failed: addr=0x%02x, reg=0x%02x, err=%s",
                 device_address_, reg, esp_err_to_name(err));
        return false;
    }
    return true;
}

void I2cDevice::ReadRegs(uint8_t reg, uint8_t* buffer, size_t length) {
    TryReadRegs(reg, buffer, length);
}

bool I2cDevice::TryReadRegs(uint8_t reg, uint8_t* buffer, size_t length) {
    esp_err_t err = i2c_master_transmit_receive(i2c_device_, &reg, 1, buffer, length, 100);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ReadRegs failed: addr=0x%02x, reg=0x%02x, len=%d, err=%s",
                 device_address_, reg, (int)length, esp_err_to_name(err));
        return false;
    }
    return true;
}

esp_err_t I2cDevice::ResetBus(const char* reason) {
    ESP_LOGW(TAG, "Resetting I2C bus: %s", reason ? reason : "unspecified");
    return i2c_master_bus_reset(i2c_bus_);
}
