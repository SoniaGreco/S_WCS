#!/usr/bin/env python

#
# Works with goal_sender.py !
#

import rospy
from  communication_node.messenger_api import *
from communication_node.msg import *
from sample_package.msg import *
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

a = None
name_space = "robot0"
goal_x = None
goal_y = None
active_x = None
active_y = None

def send_feedback(result):
	global active_x, active_y
	global goal_x, goal_y
	rate = rospy.Rate(5)
	while not rospy.is_shutdown():		
		if result:
			feed = Data_Sample()
			feed.source = "robot0"
			feed.destination = "robot1"
			
			temp_var=sample_message()
			
			temp_var.a=1
			temp_var.b= 1
			temp_var.c=str(1)
			feed.data = temp_var
			send_message(feed,Data_Sample,"sample")
			print("Sending positive feedback")

		if active_x!=goal_x or active_y!=goal_y:
			break

		rate.sleep()

		if active_x!=goal_x or active_y!=goal_y:
			break

def movebase_client():

	global goal_x
	global goal_y
	global active_x, active_y

	active_x = goal_x
	active_y = goal_y

	client = actionlib.SimpleActionClient("/"+name_space+"/move_base", MoveBaseAction)
	client.wait_for_server()
	
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = "map"
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


def callback(data):
	global goal_x
	global goal_y
	print ("Goal plan received:")
	print(str(data.data.a)+", "+str(data.data.b)+" --- goal number "+str(data.data.c)) 
	goal_x = data.data.a
	goal_y = data.data.b


def main():
	global a
	global goal_x
	global goal_y
	rospy.init_node('movebase_client_py')

	print ("Goal receiver started:")
	

	a=receive_message("robot0", Data_Sample, "sample",callback)
	rate = rospy.Rate(5)	
	while not rospy.is_shutdown():
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
	
	rospy.spin()

if __name__ == '__main__':
	main()

	
