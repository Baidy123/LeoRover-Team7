#!/bin/bash

# Start SLAM + Nav2 for hardware
ros2 launch leo_test leo_hardware_autonomous.launch.py &

# Wait for initialization
sleep 8

# Start frontier exploration
ros2 launch leo_explorer explore.launch.py use_sim_time:=false

# Keep script running
wait