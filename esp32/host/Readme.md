# Configuration 
## File
Generate a file as, described in 
https://homieiot.github.io/homie-esp8266/docs/3.0.0/configuration/json-configuration-file/
For further details have a look at the Readme.md one level above.

## Upload
* Start ESP
* Login to Wifi, opened by the ESP
* Use the script to upload the configuration file
* restart the ESP

# Remote Upload

This script will allow you to send an OTA update to your device.

## Installation

Requirements are:
* paho-mqtt

## Usage

See ***upload-via-mqtt.sh***

# Remote Upload - Backend

## Usage

```text
usage: ota_updater.py [-h] -l BROKER_HOST -p BROKER_PORT [-u BROKER_USERNAME]
                      [-d BROKER_PASSWORD] [-t BASE_TOPIC] -i DEVICE_ID
                      firmware

ota firmware update scirpt for ESP8226 implemenation of the Homie mqtt IoT
convention.

positional arguments:
  firmware              path to the firmware to be sent to the device

arguments:
  -h, --help            show this help message and exit
  -l BROKER_HOST, --broker-host BROKER_HOST
                        host name or ip address of the mqtt broker
  -p BROKER_PORT, --broker-port BROKER_PORT
                        port of the mqtt broker
  -u BROKER_USERNAME, --broker-username BROKER_USERNAME
                        username used to authenticate with the mqtt broker
  -d BROKER_PASSWORD, --broker-password BROKER_PASSWORD
                        password used to authenticate with the mqtt broker
  -t BASE_TOPIC, --base-topic BASE_TOPIC
                        base topic of the homie devices on the broker
  -i DEVICE_ID, --device-id DEVICE_ID
                        homie device id
```

* `BROKER_HOST` and `BROKER_PORT` defaults to 127.0.0.1 and 1883 respectively if not set.
* `BROKER_USERNAME` and `BROKER_PASSWORD` are optional.
* `BASE_TOPIC` has to end with a slash, defaults to `homie/` if not set.

### Example:

```bash
python ota_updater.py -l localhost -u admin -d secure -t "homie/" -i "device-id" /path/to/firmware.bin
```

The Parameter can be extracted from the serial console
```serial
{} Stored configuration
  • Hardware device ID: 12345abcd
  • Device ID: MyDeviceId
  • Name: MyDeviceName
  • Device Stats Interval: 60 sec
  • Wi-Fi:
    ◦ SSID: MyWifi
    ◦ Password not shown
  • MQTT:
    ◦ Host: 192.168.0.2
    ◦ Port: 1883
    ◦ Base topic: /test/
    ◦ Auth? no
```

will result in the following command (when executed in this folder):
```bash
python ota_updater.py -l 192.168.0.2 -t "/test/" -i "MyDeviceId" ../.pio/build/esp32doit-devkit-v1/firmware.bin
```

### Source
https://github.com/homieiot/homie-esp8266/blob/develop/scripts/ota_updater
