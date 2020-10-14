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
 * Custom GPIO

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

