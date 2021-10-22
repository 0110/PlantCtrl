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
* ESP32 **16MB flash required**
* Lipo monitoring (DS2438)
* 7 moist sensors
* 7 pump
* DC-DC convert (generating voltage from Lipo for pumps)
* DS18B20 temperature sensors
* water level via laser distance sensor (VL53L0X)
* DS2438 battery monitor
* general purpose expansion pins

# Software
The firmware for the controller is stored in ***esp32*** sub directory.
