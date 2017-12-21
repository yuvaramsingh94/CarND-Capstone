#! /bin/bash
# This script safely launches ros nodes with buffer time to allow param server population
x-terminal-emulator -e roslaunch launch/styx.launch &
sleep 3 &&
x-terminal-emulator -e rosrun waypoint_updater waypoint_updater.py
