#!/usr/bin/env python

import rospy
from communication_node.messenger_api import *
from communication_node.msg import *
from sample_package.msg import *
from std_msgs.msg import *
from nav_msgs.msg import *
from sensor_msgs.msg import *
import numpy as np


grid = None
merged_map = []

def own_callback(msg):
	
	global grid, merged_map
	if grid == None:
		grid = msg
		merged_map = msg.data
  
	
def merge_maps(new_map):
	global merged_map, grid
	new_map_np = np.array(new_map, dtype= np.int8) 
	merged_np = np.array(merged_map, dtype= np.int8) 
	
	unknown_index = (merged_np==-1)

	#Delete -1 values from previous map and replace them with values from new map
	merged_np[unknown_index] = new_map_np[unknown_index]
	
	zero_indexes = (new_map_np == 0)
	merged_np[zero_indexes] = 0

	known_index = (new_map_np>0)
	merged_np[known_index] = 100

	merged_map = np.ndarray.tolist(merged_np)

	if not grid == None:
		grid.data = merged_map
	

def map_callback(msg):

	merge_maps(msg.data.data)
	#print("merging map")

def main():
	global a
	global header, info, data, grid, merged_map
	a=receive_message("robot0", Data_Map, "map",map_callback)
	rospy.Subscriber("/robot0/map", OccupancyGrid, own_callback)
	pub = rospy.Publisher('/robot0/updated_map', OccupancyGrid, queue_size=10)

	robots_list=rospy.get_param("/robots_list")


	print("Base active")

	starting_time = rospy.get_time()

	print(starting_time)
	
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		rate.sleep()
		
		if not grid == None:
			break

	while not rospy.is_shutdown():
		
		pub.publish(grid)
		time = rospy.get_time()
		delta = float(time) - starting_time
		map_np = np.array(merged_map, dtype= np.int8) 
		known = (map_np != -1)
		known_elements = np.count_nonzero(known)
		
		percentage = (float(known_elements)/(float(len(map_np))*0.328 )) * 100

		#if abs(delta % 20) <2:
		print("Timestamp "+str(delta)+" seconds: % Area covered = "+ str(percentage))


		for j in range(0,len(robots_list)):

			if not j == 0:
				msg = Data_Map()
				msg.source = "robot0"
				msg.destination = robots_list[j]
			
				temp_var=OccupancyGrid()

				temp_var.data=merged_map

				msg.data = temp_var
				send_message(msg,Data_Map,"map")
				#print("Sent occupancy grid! ")

	
		rate.sleep()
				

if __name__ == '__main__':
	rospy.init_node("map_robot0")
	main()
	rospy.spin()

