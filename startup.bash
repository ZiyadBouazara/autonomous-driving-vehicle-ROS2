#!/bin/bash

source /home/team0/design3_ws/source_ws.bash

screen -r -S "bridge" -X quit 2>/dev/null
screen -r -S "system" -X quit 2>/dev/null

screen -S system -dm bash -c "ros2 launch rosbridge_server rosbridge_websocket_launch.xml"
screen -S system -dm bash -c "ros2 launch design3_system system.launch.py"
