#!/usr/bin/env python

import rospy
from communication_node.messenger_api import *
from communication_node.msg import *
from sample_package.msg import *
from std_msgs.msg import *
from nav_msgs.msg import *
from sensor_msgs.msg import *


header = None
info = None
data = None

def callback(msg):
	
	global header, info, data
	header = msg.header
	info = msg.info
	data = msg.data
	print("Got map. Sending it..")
  
	

def main():
	global header, info, data
	print("Sending node started: \n")
	msg = Data_Map()
	msg.source = "robot0"
	msg.destination = "robot1"
	rospy.Subscriber("/robot0/map", OccupancyGrid, callback)
	rate = rospy.Rate(100)
	while not rospy.is_shutdown():

		temp_var=OccupancyGrid()

		temp_var.header=header
		temp_var.info= info
		temp_var.data=data

		msg.data = temp_var
		send_message(msg,Data_Map,"map")
		print("Sent occupancy grid! ")
		rate.sleep()
		

if __name__ == '__main__':
	rospy.init_node("map_robot0")
	main()
	rospy.spin()




