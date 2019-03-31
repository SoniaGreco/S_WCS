#!/usr/bin/env python

import rospy
from  communication_node.messenger_api import *
from communication_node.msg import *
from sample_package.msg import *
from std_msgs.msg import *
from sensor_msgs.msg import *

more_goals = 0

def callback(data):
	global more_goals
	if data.data.a ==1:
		print ("Feedback received: Goal reached!")
		more_goals = 1




def main():
	global more_goals
	print("Sending node started: \n")
	msg = Data_Sample()
	msg.source = "robot1"
	msg.destination = "robot0"
	rate = rospy.Rate(100)
	i = 2
	while not rospy.is_shutdown():
				
		temp_var=sample_message()
		# x coordinate
		temp_var.a=i
		# x coordinate
		temp_var.b= i

		temp_var.c=str(i)
		msg.data = temp_var
		send_message(msg,Data_Sample,"sample")
		rospy.loginfo("sent goal: %d, %d ", i,i)
		rate.sleep()
		i +=1
			
		a=receive_message("robot1", Data_Sample, "sample",callback)
		if not more_goals:
			break
		

if __name__ == '__main__':
	rospy.init_node("sending_goal_from_robot1")
	main()
	rospy.spin()




