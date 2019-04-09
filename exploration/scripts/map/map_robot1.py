#!/usr/bin/env python

#
# Works with map_robot0.py and map_robot2.py !
# ROBOT1 IS AN EXPLORING BOT, REACHING GOALS AND SENDING MAP BACK TO ROBOT0
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

done = 0
header = None
info = None
data = None

goal_x = None
goal_y = None
active_x = None
active_y = None

def update_map(msg):
	
	global header, info, data
	header = msg.header
	info = msg.info
	data = msg.data
	print("\nUpdating map.. \n")

		
def callback(mess):
	global goal_x
	global goal_y
	global header, info, data, done

	if mess.data.c == "map":
		# Done represent the end of the goals sequence
		done = 1
		print("Received request for map!")
		msg = Data_Map()
		msg.source = "robot1"
		msg.destination = "robot0"
		msg.command = "map1"
		rate = rospy.Rate(100)
		while not rospy.is_shutdown():
			
			if not header ==None and not info == None and not data == None:
				temp_var=OccupancyGrid()
				temp_var.header=header
				temp_var.info= info
				temp_var.data=data

				msg.data = temp_var
				send_message(msg,Data_Map,"map")
				print("Sent occupancy grid! ")
			rate.sleep()

	elif mess.data.c == "goal":
		print ("Goal plan received:")
		print(str(mess.data.a)+", "+str(mess.data.b)) 
		goal_x = mess.data.a
		goal_y = mess.data.b


def movebase_client():

	global goal_x
	global goal_y
	global active_x, active_y

	active_x = goal_x
	active_y = goal_y

	client1 = actionlib.SimpleActionClient("/robot1/move_base", MoveBaseAction)
	client1.wait_for_server()
	
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = "map"
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.position.x = active_x
	goal.target_pose.pose.position.y = active_y
	goal.target_pose.pose.orientation.w = 1.0

	print("Going to "+ "(" +str(active_x)+", "+ str(active_y)+")")
	client1.send_goal(goal)
	wait = client1.wait_for_result()
	if not wait:
		rospy.logerr("Action server not available!")
		rospy.signal_shutdown("Action server not available!")
	else:
		return client1.get_result()


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

	print("Robot1_node started: \n")

	a=receive_message("robot1", Data_Sample, "sample", callback)

	rospy.Subscriber("/robot1/map", OccupancyGrid, update_map)

	rate = rospy.Rate(5)	
	while not rospy.is_shutdown() and not done:
		while not rospy.is_shutdown():
			rate.sleep()
			if not goal_x == None:
				break
	
		try:
			result = movebase_client()
			if result:
				print("Goal execution done!")


				#SENDS FEEDBACK TO SENDER
				send_feedback(result)				
			
		except rospy.ROSInterruptException:
			rospy.loginfo("Navigation test finished.")


		

if __name__ == '__main__':
	rospy.init_node("map_robot1")
	main()
	rospy.spin()
	
