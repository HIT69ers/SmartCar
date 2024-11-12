#!/bin/bash

gnome-terminal --window -- bash -c "roslaunch ucar_nav ucar_navigation.launch"
sleep 30s
gnome-terminal --window -- bash -c "rosservice call /xf_asr_offline_node/set_awake_word_srv “小飞”"
gnome-terminal --window -- bash -c "rosrun ucar_controller publish_base_link.py "
gnome-terminal --window -- bash -c "rosrun ucar_nav thread_new.py"
gnome-terminal --window -- bash -c "rosrun ucar_nav pid_new.py"
sleep 1s
rosparam set nav_modified 1


