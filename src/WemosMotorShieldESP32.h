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
#ifndef WEMOS_MOTOR_SHEILD_ESP32_H
#define WEMOS_MOTOR_SHEILD_ESP32_H

#define WEMOS_MOTOR_SHEILD_ESP32_DEBUG_OFF  // uncomment to DISABLE DEBUG LOGS

#include <exception>
#include <string>
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define ACK_EN 0x01
#define I2C_ROUND_NUM 32

#ifndef WEMOS_MOTOR_SHEILD_ESP32_DEBUG_OFF
#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#endif
#include "esp_log.h"

class WemosMotorShieldBaseException : public std::exception {
   protected:
    std::string _msg;

   public:
    WemosMotorShieldBaseException(std::string message = ":-(, an error occurred.") { _msg = message; };
    virtual const char *what() const throw() { return _msg.c_str(); }
};

class WemosMotorShieldI2CError : public WemosMotorShieldBaseException {
   public:
    WemosMotorShieldI2CError(std::string message = "I2CError: Check your wiring.") : WemosMotorShieldBaseException(message){};
};

typedef enum MotorNum {
    MOTOR_A = 0,
    MOTOR_B,
} motor_num_t;

typedef enum MotorDirection {
    SHORT_BRAKE = 0,
    FORWARD,
    BACKWARD,
    STOP,
    STANDBY,
} motor_direction_t;

class WemosMotorShield {
   public:
    WemosMotorShield(i2c_port_t i2cPort, uint8_t i2cAddr = 0x30, float pwmFreq = 1000, gpio_num_t standby_io_pin = GPIO_NUM_MAX);
    void setMotor(motor_num_t motor, motor_direction_t direction, float speed = 100);

   protected:
    i2c_port_t _i2cPort;
    uint8_t _i2cAddr;
    gpio_num_t _standby_io_pin;

    void _setfreq(uint32_t freq);
    void _setStandbyPin(uint8_t state);
    void _i2c_writeLen(uint8_t reg, uint8_t *buffer, uint8_t len, uint32_t timeoutMS = 100);
};

#endif