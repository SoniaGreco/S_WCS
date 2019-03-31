#!/usr/bin/env python

import rospy
from  communication_node.messenger_api import *
from communication_node.msg import *
from sample_package.msg import *
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

a = None
name_space = "robot0"
goal_x = 0
goal_y = 0

def movebase_client():
	client = actionlib.SimpleActionClient("/"+name_space+"/move_base", MoveBaseAction)
	client.wait_for_server()

	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = "map"
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.position.x = goal_x
	goal.target_pose.pose.position.y = goal_y
	goal.target_pose.pose.orientation.w = 1.0

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
	rospy.loginfo("%d,%d", data.data.a, data.data.b)  
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
	'''while not rospy.is_shutdown():
		rate.sleep()
		if not goal_x ==0:
			break
	'''
	try:
		result = movebase_client()
		if result:
			rospy.loginfo("Goal execution done!")
		rospy.spin()
		#SENDS FEEDBACK TO SENDER
		msg = Data_Sample()
		msg.source = "robot0"
		msg.destination = "robot1"
		rate = rospy.Rate(100)
		while not rospy.is_shutdown():
			temp_var=sample_message()
			
			temp_var.a=1
			temp_var.b= 1
			temp_var.c="goal reached"
			msg.data = temp_var
			send_message(msg,Data_Sample,"sample")
			rospy.loginfo("Sending positive feedback")
			rate.sleep()
			i +=1
		
	except rospy.ROSInterruptException:
		rospy.loginfo("Navigation test finished.")
	
	
if __name__ == '__main__':
	main()

	