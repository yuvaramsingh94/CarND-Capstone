#! /bin/bash
# This script safely launches ros nodes with buffer time to allow param server population
x-terminal-emulator -e roslaunch launch/styx.launch &
#sleep 5 &&
#x-terminal-emulator -e rosrun twist_controller dbw_node.py &
#sleep 5 &&
#x-terminal-emulator -e rosrun waypoint_updater waypoint_updater.py

#sleep 5 &&
#x-terminal-emulator -e rosrun tl_detector image_saver.py

sleep 5 &&
x-terminal-emulator -e rosrun tl_detector tl_detector.py
