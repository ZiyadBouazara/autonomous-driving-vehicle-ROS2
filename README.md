# design3_ws

This repo contains the colcon workspace for the design3 project. Every ROS packages are cloned using submodules. It also includes a package 'design3_system' which has every launch files to properly start each subsystem

## Installation

```bash
git clone --recurse-submodules git@github.com:GLO-3013-eq09/design3_ws.git
cd design3_ws
rosdep install --from-paths src -y --ignore-src --rosdistro=foxy --include-eol-distros
```

Setup the udev rules so that the lidar is on `/dev/sensors/lidar` with proper permissions

## Launching the system

```bash
cd design3_ws
./build.bash
./source_ws.bash # Always source your workspace after a build
./startup.bash # This starts the whole system
```

## Stopping the system

```bash
killall screen
```

## Launching a subsystem

```bash
ros2 launch design3_system <subsystem>.launch.py # You can use autocomplete to list the different available launch file
```

## Running tests and code coverage

```bash
./test.bash
```

## Visualization

Prerequisite:

- Install Foxglove Studio on your personal machine

1. On your personal machine, connect to the robot's wifi (`WAVE_ROVER_3`)
2. On the robot, launch the whole system (You need the rosbridge which is launched with the startup script)
3. On your personal machine, open Foxglove and select Open connection to a robot > Rosbridge > http://192.168.4.2:9090
4. You should be able to visualize everything on the robot
