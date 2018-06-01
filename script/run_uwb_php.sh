#!/bin/bash

pid=$(ps -ef | grep "SCREEN" | grep -v grep | awk '{print $2}')

if [ -n "$pid" ]; then
	kill "$pid"
fi

screen -d -m  UWB_Location -f /home/pi/cfg/uwb_launch.cfg -d > /dev/null 2>&1 &

