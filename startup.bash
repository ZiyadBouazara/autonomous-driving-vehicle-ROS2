#!/bin/bash

source /home/team0/design3_ws/source_ws.bash

screen -r -S "bridge" -X quit 2>/dev/null
screen -r -S "system" -X quit 2>/dev/null
screen -r -S "perception" -X quit 2>/dev/null
screen -r -S "planning" -X quit 2>/dev/null
screen -r -S "control" -X quit 2>/dev/null

screen -S bridge -dm bash -c "ros2 launch rosbridge_server rosbridge_websocket_launch.xml"
screen -S system -dm bash -c "ros2 launch design3_system system.launch.py"
screen -S perception -dm bash -c "ros2 launch design3_system perception.launch.py"
screen -S planning -dm bash -c "ros2 launch design3_system planning.launch.py"
screen -S control -dm bash -c "ros2 launch design3_system control.launch.py"
