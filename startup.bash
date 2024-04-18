#!/bin/bash

source /home/team0/design3_ws/source_ws.bash

screen -r -S "bridge" -X quit > /dev/null
screen -r -S "system" -X quit > /dev/null
screen -r -S "perception" -X quit > /dev/null
screen -r -S "planning" -X quit > /dev/null
screen -r -S "control" -X quit > /dev/null

screen -S bridge -dm bash -c "ros2 launch rosbridge_server rosbridge_websocket_launch.xml"
screen -S system -dm bash -c "ros2 launch design3_system system.launch.py | tee /home/team0/system-screen.log"
screen -S perception -dm bash -c "ros2 launch design3_system perception.launch.py"
screen -S control -dm bash -c "ros2 launch design3_system control.launch.py"
sleep 2
screen -S planning -dm bash -c "ros2 launch design3_system planning.launch.py"

echo "Waiting for camera startup..."

while true; do
        sleep 10
        if cat /home/team0/system-screen.log | grep "Camera ready"; then
                echo "System ready!"
                exit 0
        else
                echo "Restarting camera..."
                screen -r -S "system" -X quit >/dev/null
                screen -S system -dm bash -c "ros2 launch design3_system system.launch.py | tee /home/team0/system-screen.log"
        fi
done
