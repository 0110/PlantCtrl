#!/bin/bash
# Needs the tool pcb2gcode
# Was documented at: http://marcuswolschon.blogspot.de/2013/02/milling-pcbs-using-gerber2gcode.html

MILLSPEED=600
MILLFEED=200
PROJECT=PlantCtrlESP32
pcb2gcode --back $PROJECT-B_Cu.gbr --outline $PROJECT-Edge_Cuts.gbr --metric \
	--zsafe 5 --zchange 10 --zwork -0.01 --offset 0.02 --mill-feed $MILLFEED --mill-speed $MILLSPEED \
	--drill $PROJECT.drl --zdrill -2.5 --drill-feed $MILLFEED --drill-speed $MILLSPEED --basename $PROJECT \
	--zcut 1.0 --cut-infeed 1.0  --cut-feed $MILLFEED --cutter-diameter 1.0 --cut-speed $MILLSPEED
