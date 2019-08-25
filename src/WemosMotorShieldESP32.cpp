// MIT License

// Copyright (c) 2019 ShellAddicted

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

/*!please use the following clang-settings {BasedOnStyle: Google, ColumnLimit: 130, IndentWidth: 4}!*/
#include "WemosMotorShieldESP32.h"

static const char *WEMOS_MOTOR_SHIELD_LOG_TAG = "WemosMS";

WemosMotorShield::WemosMotorShield(i2c_port_t i2cPort, uint8_t i2cAddr, float pwmFreq, gpio_num_t standby_io_pin) {
    _i2cPort = i2cPort;
    _i2cAddr = i2cAddr;
    _standby_io_pin = standby_io_pin;
    _setfreq(pwmFreq);
    if (standby_io_pin < GPIO_NUM_MAX) {
        _setStandbyPin(0);
    }
}

void WemosMotorShield::_i2c_writeLen(uint8_t reg, uint8_t *buffer, uint8_t len, uint32_t timeoutMS) {
    esp_err_t errx;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_i2cAddr << 1) | I2C_MASTER_WRITE, ACK_EN);
    i2c_master_write_byte(cmd, reg, ACK_EN);
    i2c_master_write(cmd, buffer, len, 0x01);
    i2c_master_stop(cmd);

    for (int round = 1; round <= I2C_ROUND_NUM; round++) {
#ifndef WEMOS_MOTOR_SHEILD_ESP32_DEBUG_OFF
        ESP_LOGD(WEMOS_MOTOR_SHIELD_LOG_TAG, "(i2c_WL) Round %d", round);
#endif
        errx = i2c_master_cmd_begin(_i2cPort, cmd, timeoutMS / portTICK_PERIOD_MS);
        if (errx == ESP_OK) {
            break;
        } else if ((errx != ESP_OK) && (round < I2C_ROUND_NUM)) {
            continue;
        } else {
            i2c_cmd_link_delete(cmd);
            ESP_LOGE(WEMOS_MOTOR_SHIELD_LOG_TAG, "(i2c WL) Error: %d", (int)errx);
            throw WemosMotorShieldI2CError();
        }
    }
    i2c_cmd_link_delete(cmd);
}

void WemosMotorShield::_setStandbyPin(uint8_t state) {
    gpio_pad_select_gpio(_standby_io_pin);
    gpio_set_direction(_standby_io_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(_standby_io_pin, state);
}

void WemosMotorShield::_setfreq(uint32_t freq) {
    uint8_t data[4];
    data[0] = (freq >> 16) & 0x0F;
    data[1] = freq >> 16;
    data[2] = freq >> 8;
    data[3] = freq;
    _i2c_writeLen(0x00, data, 4, 100);
    vTaskDelay(100 / portTICK_PERIOD_MS);
}
void WemosMotorShield::setMotor(motor_num_t motor, motor_direction_t direction, float speed) {
    if (_standby_io_pin != GPIO_NUM_MAX) {
        if (direction == MotorDirection::STANDBY) {
            _setStandbyPin(1);
        } else {
            _setStandbyPin(0);
        }
    }

    if (speed < 0.0) {
        speed = 0.0;
    } else if (speed > 100.0) {
        speed = 100.0;
    }

    uint16_t _pwm_val = speed * 100;

    const uint8_t reg = motor | 0x10;
    uint8_t data[3];
    data[0] = direction;
    data[1] = _pwm_val >> 8;
    data[2] = _pwm_val & 0xFF;
    _i2c_writeLen(reg, data, 3);

    vTaskDelay(100 / portTICK_PERIOD_MS);
}