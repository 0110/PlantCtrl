#!/bin/bash
echo "Homie device is in AP mode, then the configuration can be uploaded"

if [ ! -f config.json ]; then
 echo "Create config file according to :"
 echo "https://homieiot.github.io/homie-esp8266/docs/3.0.0/configuration/json-configuration-file/"
 exit 2
fi

echo "Check connection to Plug in AP-mode"
ping -c 4 192.168.123.1

curl -X PUT http://192.168.123.1/config --header "Content-Type: application/json" -d @config.json
