# WCS

Based on https://github.com/taherahmadi/WCS.git

# How to start

#EVERY COMMAND IN A NEW TERMINAL

roslaunch setup gserver.launch

roslaunch communication_node registration_server.launch

roslaunch setup spawn.launch

#check robot registered with
rosparam get /robots_list

#otherwise register them with
rosrun communication_node registration_client robot0

gzclient

roslaunch communication_node update_info.launch

roslaunch communication_node message_handler.launch

RVIZ

roslaunch communication_node connection_visualizer.launch

#Send and receive laser scan data

rosrun sample_package read_from_scan.py

rosrun sample_package receive_from_scan.py
