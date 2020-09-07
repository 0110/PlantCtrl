#!/bin/bash
# Needs the tool pcb2gcode
# Was documented at: http://marcuswolschon.blogspot.de/2013/02/milling-pcbs-using-gerber2gcode.html

MILLSPEED=600
MILLFEED=200
PROJECT=PlantCtrlESP32
pcb2gcode --back $PROJECT-B_Cu.gbr --metric --zsafe 5 --zchange 10 --zwork -0.01 --offset 0.02 --mill-feed $MILLFEED --mill-speed $MILLSPEED --drill $PROJECT.drl --zdrill -2.5 --drill-feed $MILLFEED --drill-speed $MILLSPEED --basename $PROJECT

if [ "$1" ==  "C3MA" ]; then
	#update all Tools higher and equal to T4 in generated file
	for i in 4 5 6 7; do
		echo "Replace T$i"
		sed -i.bakT$i "s/T${i}/T3/" ${PROJECT}_drill.ngc
	done
fi
