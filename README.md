# PlantCtrl

The following problems shall be solved with this project:
* Solar Powered
* Low Powered
* Plant monitoring
* Plant watering
* IoT

# Hardware
Open hardware design (powered by KiCAD).
The complete PCB is stored in the ***board*** sub directory.

There the following components are connected:
* ESP32 NodeMCU Module (the one of A-Z Delivery was used)
* Lipo
* 7 moist sensors
* 7 pump
* DC-DC convert (generating voltage from Lipo for pumps)
* DS18B20 temperature sensors
* water tank ultrasonic sensor (via HC_SR04)
* general purpose expansion pin

# Software
The firmware for the controller is stored in ***esp32*** sub directory.
