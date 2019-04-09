#!/usr/bin/env python

#
# Works with goal_receiver.py !
#

import rospy
from  communication_node.messenger_api import *
from communication_node.msg import *
from sample_package.msg import *
from std_msgs.msg import *
from sensor_msgs.msg import *

reached = 0
a = None


def callback(data):
	global reached
	rospy.loginfo("Feedback received")
	if data.data.a ==1:
		print("Feedback received: Goal reached!")
		reached = 1

def wait_for_feedback():
	global reached
	rate = rospy.Rate(5)
	a=receive_message("robot1", Data_Sample, "sample",callback)
	while not rospy.is_shutdown():	
		rate.sleep()			
		if reached:
			break
	
		


def main():
	global reached
	global a
	print("Sending node started: \n")
	msg = Data_Sample()
	msg.source = "robot1"
	msg.destination = "robot0"
	rate = rospy.Rate(5)
	j = 0
	i = 1
	goals_list = [[1,1],[1,2],[2,1],[-1,0],[0,-2]]
	a = receive_message("robot1", Data_Sample, "sample",callback)
	while (not rospy.is_shutdown()) and j<len(goals_list):
		if reached:
			reached = 0
			j = j+1
		else:			
			temp_var = sample_message()
			# x coordinate
			temp_var.a = goals_list[j][0]
			# y coordinate
			temp_var.b = goals_list[j][1]
			temp_var.c = str(j)
			msg.data = temp_var
			send_message(msg,Data_Sample,"sample")
			print("Sent goal: "+ str(temp_var.a) + ", " + str(temp_var.b) + "--- goal number"+str(j))
		
		rate.sleep()

if __name__ == '__main__':
	rospy.init_node("sending_goal_from_robot1")
	main()
	rospy.spin()




