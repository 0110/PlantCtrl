# PlantControl
## Hardware

Main processor
* ESP32 with 16MB Flash 

One-Wire
* Temperatur Sensor (DS18B20)
* Lipo-Monitoring (DS2438)

Lipo Protection
* Open drain 3.3V detector (CN61CN33 @ jlcpcb parts)



### Used Pins:
* See '''include/ControllerConfiguration.h'''

## Software
* MQTT topics

# Hardware
## Features
* Support for up to
 * 7 Moister sensors
 * 7 Pumps
 * Sensors
  * Solar powered (voltage)
  * Lipo-Powered (DS2438 for monitoring)
  * Temperature
  * Laser distance sensor [VL53L0X]
 * Custom GPIO

## Documentation of Power-Modes
https://lastminuteengineers.com/esp32-sleep-modes-power-consumption/#esp32-deep-sleep


gpio 17 only out no hold
gpio 16 only out no hold

## Additional hardware
solar charger 2A?
https://www.aliexpress.com/item/4000238259949.html?spm=a2g0o.productlist.0.0.7e50231cCWGu0Z&algo_pvid=9ab7b0d3-5026-438b-972b-1d4a81d4dc56&algo_expid=9ab7b0d3-5026-438b-972b-1d4a81d4dc56-11&btsid=0b0a0ac215999246489888249e72a9&ws_ab_test=searchweb0_0,searchweb201602_,searchweb201603_

MT3608 boost für pumpe
https://www.aliexpress.com/item/32925951391.html?spm=a2g0o.productlist.0.0.39e21087nAzH9q&algo_pvid=7db0a849-62f7-4403-88e3-615ee4d99339&algo_expid=7db0a849-62f7-4403-88e3-615ee4d99339-0&btsid=0b0a0ac215999252934777876e7253&ws_ab_test=searchweb0_0,searchweb201602_,searchweb201603_

DS18B20 one wire temp sensor


# Features
## Empires Wunschliste
 * Pflanze
  * Pumpe
   * [x] Zeitspann (wann laufen darf)
   * [x] Helligkeitstrigger (Um den Morgen zum pumpen zu erkennen)
   * [-] Maximal Dauer zum Pumpen (als Zeit oder Milliliter)
   * [x] Zeitspanne zwischen zwei Pumpvorgängen
  * Moister sensor
   * [x] Schwellwert für Pumpe
* Tank
 * Füllstand Anzeige (in Liter)
 * Minimum Wasserstand (in cm damit Pumpen nicht leer laufen; enspricht 0 nutzbaren Liter)
 * [x] Maximaler Wasserstand des Tanks (in cm & Liter)

## Masterplan 2.0
 * Partitionslayout


