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

def send_updated_map(new_grid):
	msg = Data_Map()
	msg.source = "robot0"
	msg.destination = new_grid.source
	msg.command = "map"
	temp_var=OccupancyGrid()
	temp_var.data = merged_map
	msg.data = temp_var
	send_message(msg,Data_Map,"map")
	#print("Sent updated map to "+ str(new_grid.source) + "! ")

def merge_maps(new_grid):
	global merged_map, grid
	
	if len(merged_map)==0:
		merged_map = new_grid.data.data
	else:

		merged_np = np.array(merged_map, dtype= np.int8) 

		unknown_index = (merged_np==-1)

		new_map_np = np.array(new_grid.data.data, dtype= np.int8) 
		
		#Delete -1 values from previous map and replace them with values from new map
		merged_np[unknown_index] = new_map_np[unknown_index]

		zero_indexes = (new_map_np == 0)
		merged_np[zero_indexes] = 0

		known_index = (new_map_np>0)
		merged_np[known_index] = 100
		merged_map = np.ndarray.tolist(merged_np)

	if not grid == None:
		grid.data = merged_map
	

	
		
def callback(new_grid):

	global merged_map, done, robots_list, printed 
	
	if new_grid.command == "map":
		if new_grid.source != "robot0":
			print("Received map update from "+ new_grid.source + "! Merging...")
			merge_maps(new_grid)
			send_updated_map(new_grid)
	elif new_grid.command == "done":
		if done == len(robots_list)-1 and not printed:
			print("\nEXPLORATION IS OVER! \nCalling all robots back...")
			printed = 1
		else:
			done = done+1  

def update_map(u_map):
	global merged_map, grid
	if len(merged_map) ==0:
		merged_map =  u_map.data
		grid = u_map
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
	
	starting_time = rospy.get_time()


	while (not rospy.is_shutdown()):
		merged=list(merged_map)
		grid.data = merged
		pub.publish(grid)

		time = rospy.get_time()
		delta = float(time) - starting_time
		map_np = np.array(merged_map, dtype= np.int8) 
		known = (map_np != -1)
		known_elements = np.count_nonzero(known)
		
		percentage = (float(known_elements)/(float(len(map_np))*0.37 )) * 100

		if abs(delta % 20) <1:
			print("Timestamp "+str(delta)+" seconds: % Area covered = "+ str(percentage))

		rate.sleep()



if __name__ == "__main__":
	rospy.init_node("base_station_robot0")
	main()
	rospy.spin()
