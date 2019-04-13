# WCS

Based on https://github.com/taherahmadi/WCS.git

# ROLE-BASED STRATEGY IMPLEMENTED: 
# How to test it:

[EVERY COMMAND IN A NEW TERMINAL]

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

roslaunch setup navigation.launch


roslaunch communication_node connection_visualizer.launch

RVIZ

(IN RVIZ ADD A TOPIC CALLED "/robot0/updated_map")

#ROLE-BASED BASE STATION NODE
rosrun exploration rb_base_station.py

#ROLE-BASED ROBOT1 NODE
rosrun exploration rb_robot1.py

#ROLE-BASED ROBOT2 NODE
rosrun exploration rb_robot2.py

 
