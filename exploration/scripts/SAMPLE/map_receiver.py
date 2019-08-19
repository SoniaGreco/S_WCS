#!/usr/bin/env python

import rospy
from  communication_node.messenger_api import *
from communication_node.msg import *
from sample_package.msg import *


def callback(msg):

	print ("map info received:")
	print(msg.data.data)


def main():
	global a
	print ("OccupancyGrid receiver started:")
	a=receive_message("robot1", Data_Map, "map",callback)
	


if __name__ == "__main__":
	rospy.init_node("receiver_robot1")
	main()
	rospy.spin()
