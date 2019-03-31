#!/usr/bin/env python

import rospy
from  communication_node.messenger_api import *
from communication_node.msg import *
from sample_package.msg import *
from std_msgs.msg import *
from sensor_msgs.msg import *


def main():
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
		

if __name__ == '__main__':
	rospy.init_node("sending_goal_from_robot1")
	main()
	rospy.spin()




