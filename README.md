# WemosMotorShieldESP32

This idf-component provides a C++ Interface for [Wemos's Motor Shield V1](https://wiki.wemos.cc/products:retired:motor_shield_v1.0.0) compatible with [Espressif's ESP32 SoC](https://www.espressif.com/en/products/hardware/esp32/overview) (running [esp-idf](https://github.com/espressif/esp-idf)).

***Some motor shields ship with a broken firmware, you must update the board to make it work (or being detected). more info [here](https://medium.com/@shelladdicted/how-to-update-the-firmware-of-a-wemos-motor-shield-fadbb496fd01)***

# Getting Started
***NOTE: this code is not (yet) Production Ready.***   
You can use this library as a component for your project: 
```
cd <YOUR_PROJECT_ROOT>
mkdir components/
cd components/
git clone https://github.com/ShellAddicted/WemosMotorShieldESP32
```
Remember to enable ```Compiler Options -> Enable C++ Exceptions``` using ```make menuconfig```

for more details see [examples/](https://github.com/ShellAddicted/WemosMotorShieldESP32/tree/master/examples)

## Wiring

### I2C
T (or D1) -> SCL (Default: GPIO_NUM_22)  
R (or D2) -> SDA (Default: GPIO_NUM_21)
