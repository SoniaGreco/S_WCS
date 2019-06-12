#!/bin/sh

gnome-terminal -- roslaunch setup gserver.launch

sleep 5
gnome-terminal -- roslaunch communication_node registration_server.launch
sleep 5
gnome-terminal -- roslaunch setup spawn.launch
sleep 5
gnome-terminal -- roslaunch communication_node update_info.launch
sleep 5
gnome-terminal -- roslaunch communication_node message_handler.launch
sleep 5
gnome-terminal -- roslaunch setup navigation.launch
sleep 5
gnome-terminal -- gzclient
sleep 5
gnome-terminal -- roslaunch communication_node connection_visualizer.launch
sleep 5
gnome-terminal -- rviz
#sleep 5
#gnome-terminal -- roslaunch exploration burgard.launch



