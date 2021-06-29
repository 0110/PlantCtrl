#!//bin/bash

if [ $# -ne 3 ]; then
	echo "Homie prefex and device index must be specified:"
	echo "$0 <mqtt host> <prefix> <device index>"
	echo "e.g."
	echo "$0 192.168.0.2 test/ MyDeviceId"
	exit 1
fi

mqttHost=$1
mqttPrefix=$2
homieId=$3
firmwareFile=../.pio/build/esp32doit-devkit-v1/firmware.bin

if [ ! -f $firmwareFile ]; then
	echo "the script $0 must be started in host/ sub directory"
	exit 2
fi

mosquitto_pub -h $mqttHost -t "${mqttPrefix}${homieId}/stay/alive/set" -m "1" -r
echo "Waiting ..."
mosquitto_sub -h $mqttHost -t "${mqttPrefix}${homieId}/#" -R -C 1
set -e
python ota_updater.py -l $mqttHost -t "$mqttPrefix" -i "$homieId" $firmwareFile

mosquitto_pub -h $mqttHost -t "${mqttPrefix}${homieId}/stay/alive/set" -m "0" -r
exit 0
