#!/bin/bash
source .pixi/envs/humble-robot/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml
# ros2 run demo_nodes_cpp talker