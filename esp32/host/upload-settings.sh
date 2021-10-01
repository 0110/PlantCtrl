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

settingsFile=settings.json
if [ ! -f $settingsFile ]; then
	echo "$settingsFile missing"
	echo "check $settingsFile.example"
	exit 1
fi

mosquitto_pub -h $mqttHost -t "${mqttPrefix}${homieId}/stay/alive/set" -m "1" -r
echo "Waiting ..."
mosquitto_sub -h $mqttHost -t "${mqttPrefix}${homieId}/#" -R -C 1
set -e
echo "Waiting 30 seconds ..."
sleep 30
mosquitto_pub -h $mqttHost -t "${mqttPrefix}${homieId}/\$implementation/config/set" -f $settingsFile
echo "Waiting for reboot ..."
sleep 1
mosquitto_sub -h $mqttHost -t "${mqttPrefix}${homieId}/#" -R -C 1
echo "Alive"
sleep 20
echo "Create Backup ..."
mosquitto_pub -h $mqttHost -t "${mqttPrefix}${homieId}/config/backup/set" -m "true" -r
sleep 5
echo "Shutdown ..."
mosquitto_pub -h $mqttHost -t "${mqttPrefix}${homieId}/stay/alive/set" -m "0" -r
exit 0
