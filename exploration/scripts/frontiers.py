#!/usr/bin/env python

#
# 
#

import rospy
from communication_node.messenger_api import *
from communication_node.msg import *
from sample_package.msg import *
from std_msgs.msg import *
from nav_msgs.msg import *
from sensor_msgs.msg import *
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from frontier_search import *
import numpy as np

done = 0
header = None
info = None
data = None
merged_map = None

robot_x = None
robot_y = None

def update_map(msg):
	
	global header, info, data, merged_map
	#header = msg.header
	#info = msg.info
	#data = msg.data
	merged_map = msg
	print("\nUpdating map.. \n")

def odom_callback(odom_data):
	global robot_x,robot_y
	robot_x=odom_data.pose.pose.position.x
	robot_y=odom_data.pose.pose.position.y

def compute_frontier_distance(frontiers):
    global robot_x,robot_y
    frontier_distances=[]
    for i in range(0, len(frontiers)):
        distance = math.sqrt( (robot_x-frontiers[i].travel_point.x)**2 +  (robot_y-frontiers[i].travel_point.y)**2)
        frontier_distances.append(distance)
    return list(frontier_distances)


def get_frontiers(map_data):
		global robot_x,robot_y
		rospy.sleep(1.0)
		frontiers=[]
		while(robot_x==None or robot_y==None):
			rospy.sleep(0.5)
		print("Getting frontiers..")
		fsc=FrontierSearch(map_data,5,"balanced")
		test_frontiers=fsc.searchFrom(Point(robot_x,robot_y,0.0))
		frontiers=list(test_frontiers[0])
		test_id=0
		return list(frontiers)

def movebase_client():

	global goal_x
	global goal_y
	global active_x, active_y
	global client

	active_x = goal_x
	active_y = goal_y

	client = actionlib.SimpleActionClient("/robot0/move_base", MoveBaseAction)
	client.wait_for_server()
	
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = "/map"
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.position.x = active_x
	goal.target_pose.pose.position.y = active_y
	goal.target_pose.pose.orientation.w = 1.0

	print("Going to "+ "(" +str(active_x)+", "+ str(active_y)+")")
	client.send_goal(goal)
	wait = client.wait_for_result()
	if not wait:
		rospy.logerr("Action server not available!")
		rospy.signal_shutdown("Action server not available!")
	else:
		return client.get_result()

def initial_move():
	global goal_x, goal_y
	global robot_x, robot_y

	goal_x = robot_x+2.5
	goal_y = robot_y-2.5


def send_feedback(result):
	global active_x, active_y, done
	global goal_x, goal_y
	rate = rospy.Rate(5)
	while not rospy.is_shutdown() and not done:		
		if result:
			feed = Data_Map()
			feed.source = "robot1"
			feed.destination = "robot0"
			feed.command = "reached_1"
			
			temp_var=OccupancyGrid()
			
			feed.data = temp_var
			send_message(feed,Data_Map,"map")
			print("Sending positive feedback")

		if active_x!=goal_x or active_y!=goal_y:
			break

		rate.sleep()

		if active_x!=goal_x or active_y!=goal_y:
			break


def main():
	global header, info, data, done
	global a
	global goal_x
	global goal_y
	global merged_map


	print("Robot0_node started: \n")

	rospy.Subscriber("/robot0/map", OccupancyGrid, update_map)
	rospy.Subscriber("/robot0/odom", Odometry, odom_callback)

	rate = rospy.Rate(5)
	while not rospy.is_shutdown():
		rate.sleep()

		if not robot_x == None:
			break
	
	initial_move()

	rate = rospy.Rate(5)
	while not rospy.is_shutdown():
		rate.sleep()

		if not goal_x == None:
			break

	result = movebase_client()
	if result:
		print("Initial move done!")
		
		
	rate = rospy.Rate(5)
	while not rospy.is_shutdown():

		while not rospy.is_shutdown():
			rate.sleep()

			if not merged_map == None:
				break

		try: 
			print("Getting frontiers")
			frontiers=get_frontiers(merged_map)
			if (len(frontiers)==0):
				print(name_space,"no new frontiers")
				exit()

			print("Detected " + str(len(frontiers)) + " frontiers")
			
			distances = compute_frontier_distance(frontiers)
			nearest = distances.index(min(distances))

			goal_x = frontiers[nearest].travel_point.x
			goal_y = frontiers[nearest].travel_point.y

			result = movebase_client()
			if result:
				print("Frontier reached!")
			
			
		except rospy.ROSInterruptException:
			rospy.loginfo("Navigation test finished.")


		

if __name__ == '__main__':
	rospy.init_node("frontiers_robot0")
	main()
	rospy.spin()
	
