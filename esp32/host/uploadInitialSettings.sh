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

mosquitto_pub -h $mqttHost -t "${mqttPrefix}${homieId}/stay/alive/set" -m "1" -r
echo "Waiting ..."
mosquitto_sub -h $mqttHost -t "${mqttPrefix}${homieId}/#" -R -C 1
set -e
sleep 30
mosquitto_pub -h $mqttHost -t "${mqttPrefix}${homieId}/\$implementation/config/set" -m "{
	\"settings\": {
		\"sleep\":600,
		\"nightsleep\": 1200,
		\"pumpsleep\": 5,
		\"tankmax\": 1000,
		\"tankmin\": 100,
		\"tankwarn\": 200,
		\"tankVolume\": 100,
		\"lipoDSAddr\": \"abcdefghijklmnop\",
		\"tankDSAddr\": \"abcdefghijklmnop\",
		\"ntpServer\":\"pool.ntp.org\",
		\"dry0\":5000,
		\"hourstart0\":6,
		\"hourend0\":20,
		\"lowLight0\": false,
		\"delay0\": 10,
		\"dry1\":5000,
		\"hourstart1\":6,
		\"hourend1\":20,
		\"lowLight1\": false,
		\"delay1\": 10,
		\"dry2\":5000,
		\"hourstart2\":6,
		\"hourend2\":20,
		\"lowLight2\": false,
		\"delay2\": 10,		
		\"dry3\":5000,
		\"hourstart3\":6,
		\"hourend3\":20,
		\"lowLight3\": false,
		\"delay3\": 10,
		\"dry4\":5000,
		\"hourstart4\":6,
		\"hourend4\":20,
		\"lowLight4\": false,
		\"delay4\": 10,
		\"dry5\":5000,
		\"hourstart5\":6,
		\"hourend5\":20,
		\"lowLight5\": false,
		\"delay5\": 10,
		\"dry6\":5000,
		\"hourstart6\":6,
		\"hourend6\":20,
		\"lowLight6\": false,
		\"delay6\": 10
	}
}"
echo "Waiting for reboot"
sleep 1
mosquitto_sub -h $mqttHost -t "${mqttPrefix}${homieId}/#" -R -C 1
sleep 20
mosquitto_pub -h $mqttHost -t "${mqttPrefix}${homieId}/config/backup/set" -m "true" -r
sleep 5
mosquitto_pub -h $mqttHost -t "${mqttPrefix}${homieId}/stay/alive/set" -m "0" -r
exit 0
