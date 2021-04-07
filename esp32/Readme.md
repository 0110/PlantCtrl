# PlantControl
## Hardware

Uses ESP32MiniKit

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
  * Lipo-Powered (voltage)
  * Temperature
  * Distance sensor [JSN-SR04T-2.0] (for waterlevel)
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
   * Zeitspann (wann laufen darf)
   * Helligkeitstrigger (Um den Morgen zum pumpen zu erkennen)
   * Maximal Dauer zum Pumpen (als Zeit oder Milliliter)
   * Zeitspanne zwischen zwei Pumpvorgängen
  * Moister sensor
   * Oberen
   * Unteren Wert
* Tank
 * Füllstand Anzeige (in Liter)
 * Minimum Wasserstand (in cm damit Pumpen nicht leer laufen; enspricht 0 nutzbaren Liter)
 * Trigger-Erinnerungen um Wasser nachzufüllen
 * Maximaler Wasserstand des Tanks (in cm & Liter)
* System
 * Tiefentladungsschutz vom LIPO (fest im Controller die Spannung festlegen)
  * 3.5V unterschritten, dann nur noch Deepsleep
  * MQTT Topic, wenn Spannung unterschritten wurde
 * Lipo innerhalb 24h nicht geladen -> MQTT Topic
 * Deep-Sleep
  * Mode1: 
    * Nur Sensor werte einsameln
    * Wird verlassen bei Aktionen
        * Pumpe schalten
        * MQTT Nachrichten
        * nach x Minuten nur in Mode1
  * Mode2: 
    * WLAN aktivieren und Werte über MQTT raus hauen
    * aktuelle Werte raushauen
    * MQTT lesen
  * Mode3:
    * Deepsleep verboten (MQTT topic, retained)
    * alle Pumpen & Sensoren deaktiviert

## Masterplan 2.0
 * kein WLAN 
   * Bewässerung muss immer laufen
   * Timeout um nicht ewig aufs WLAN zu warten
 * Nicht mehrere Messungen vom Temp-Sensor nur warten, bis wir einen Wert bekommen
 * Partitionslayout
 * OW-search address in topic nutzen
 * Wifi bei timout deaktivieren (damit wir entweder wlan spielen oder fallback: nur pflanzen giessen)


