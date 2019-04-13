#!/usr/bin/env python

#
# BASE STATION OF ROLE-BASED STRATEGY
#
# Its main function is to wait for robots to update the map
# of the area until the all area has been covered and mapped
#
# BASE_X = 0 || BASE_Y = 0
#

import rospy
from  communication_node.messenger_api import *
from communication_node.msg import *
from sample_package.msg import *
from std_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *
import numpy as np
from frontier_search import *

merged_map = []
done = 0
grid = None
robots_list = []
printed = 0

def send_updated_map(grid):
	msg = Data_Map()
	msg.source = "robot0"
	msg.destination = grid.source
	msg.command = "map"
	temp_var=OccupancyGrid()
	temp_var.data = grid.data.data
	msg.data = temp_var
	send_message(msg,Data_Map,"map")
	#print("Sent updated map to "+ str(grid.source) + "! ")

def merge_maps(grid):
	global merged_map, grid
	
	if len(merged_map)==0:
		merged_map = np.array(grid.data.data, dtype= np.int8)
	else:

		unknown_index = (merged_map==-1)

		new_map = np.array(grid.data.data, dtype= np.int8) 
		
		#Delete -1 values from previous map and replace them with values from new map
		merged_map[unknown_index] = new_map[unknown_index]

		known_index = np.logical_and(merged_map>0, new_map>0)
		merged_map[known_index] = 100
		grid.data = merged_map

	
		
def callback(grid):

	global merged_map, done, robots_list, printed 
	
	if grid.command == "map":
		print("Received map update from "+ grid.source + "! Merging...")
		merge_maps(grid)
		send_updated_map(grid)
	elif grid.command == "done":
		if done == len(robots_list)-2 and not printed:
			print("All frontiers have been explored! \nCalling all robots back...")
			printed = 1
		else:
			done = done+1  

def update_map(map):
	global merged_map, grid
	if len(merged_map) ==0:
		merged_map =  np.array(map.data, dtype= np.int8)
		grid = map
		grid.data = merged_map
		print("\nInitializing map.. \n")

def main():
	global a, done
	global merged_map, grid, robots_list
	print ("\nBase station is active:")

	robots_list=rospy.get_param("/robots_list")

	a = receive_message("robot0", Data_Map, "map",callback)
	pub = rospy.Publisher('/robot0/updated_map', OccupancyGrid, queue_size=10)
	rospy.Subscriber("/robot0/map", OccupancyGrid, update_map)


	rate = rospy.Rate(1)

	print("Waiting for map updates...")
	while not rospy.is_shutdown():
		if not grid == None:
			rate.sleep()
			break

	while (not rospy.is_shutdown()):
		merged=list(merged_map)
		grid.data = merged
		pub.publish(grid)
		rate.sleep()



if __name__ == "__main__":
	rospy.init_node("base_station_robot0")
	main()
	rospy.spin()
