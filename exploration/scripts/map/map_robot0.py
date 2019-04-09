#!/usr/bin/env python

#
# Works with map_robot1.py and map_robot2.py !
# ROBOT0 IS THE BASE STATION: IT SENDS GOALS FOR ROBOT1 AND ROBOT2 TO REACH, 
# AND FINALLY MERGES THE MAPS COLLECTED BY THEM
#

import rospy
from  communication_node.messenger_api import *
from communication_node.msg import *
from sample_package.msg import *
from std_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *
import numpy as np

#Set to 1 if goal reached by robot1 and robot2
reached_1 = 0
reached_2 = 0

#Set to 1 if map data received
map1 = 0
map2 = 0

#Variables to store MapMetaData info and array data from OccupancyGrid structure
grid1 = None
grid2 = None


def merge_maps(info_map1, info_map2):
	array_map1 = np.array(info_map1.data, dtype= np.int8)
	array_map2 = np.array(info_map2.data, dtype= np.int8)

	final_map = array_map1

	unknown_index = (array_map1==-1)
	
	#Delete -1 values from array_1 and replace them with values from array_2
	final_map[unknown_index_1] = array_map2[unknown_index_1]

	known_index = np.logical_and(array_map1>0, array_map2>0)
	final_map[known_index] = 100
	
def callback(msg):

	global reached_1, reached_2
	global map1, map2, grid1, grid2
	if msg.command == "reached_1":
		print("Feedback received: Goal reached by robot1!")
		reached_1 = 1
	elif msg.command == "reached_2":
		print("Feedback received: Goal reached by robot2!")
		reached_2 = 1
	elif msg.command == "map1":
		print ("map1 info received:")
		print(msg.data.data[0:10])	
		grid1 = msg.data
		map1 = 1
	elif msg.command == "map2":
		print ("map2 info received:")
		print(msg.data.data[0:10])
		grid2 = msg.data
		map2 = 1


def main():
	global a
	global reached_1, reached_2, map1, map2, grid1, grid2
	print ("Map receiver - goal sender started:")

	msg1 = Data_Sample()
	msg1.source = "robot0"
	msg1.destination = "robot1"
	
	msg2 = Data_Sample()
	msg2.source = "robot0"
	msg2.destination = "robot2"
	
	goals_list_1 = [[1,1],[1,2],[2,1],[-1,0],[0,-2]]
	goals_list_2 = [[0,3],[1,0],[1,3],[-1,1],[0,0]]

	a = receive_message("robot0", Data_Map, "map",callback)

	rate = rospy.Rate(5)
	j = 0
	k = 0
	while (not rospy.is_shutdown()) and (j<len(goals_list_1) or k<len(goals_list_2)):
		if reached_1:
			reached_1 = 0
			j = j+1
		if reached_2:
			reached_2 = 0
			k = k+1
		else:
			if j<len(goals_list_1):			
				temp_var_1 = sample_message()
				# x coordinate
				temp_var_1.a = goals_list_1[j][0]
				# y coordinate
				temp_var_1.b = goals_list_1[j][1]
				temp_var_1.c = "goal"
				msg1.data = temp_var_1
				send_message(msg1,Data_Sample,"sample")
				print("Sent goal to robot1: "+ str(temp_var_1.a) + ", " + str(temp_var_1.b) + "--- goal number "+str(j))

			if k<len(goals_list_2):	
				temp_var_2 = sample_message()
				# x coordinate
				temp_var_2.a = goals_list_2[k][0]
				# y coordinate
				temp_var_2.b = goals_list_2[k][1]
				temp_var_2.c = "goal"
				msg2.data = temp_var_2
				send_message(msg2,Data_Sample,"sample")
				print("Sent goal to robot2: "+ str(temp_var_2.a) + ", " + str(temp_var_2.b) + "--- goal number "+str(k))
			
		rate.sleep()
	
	rate = rospy.Rate(5)

	#When all the goals have been reached
	if j==len(goals_list_1) and k==len(goals_list_2):
		#Sends request for maps and waits until it reeives both
		while (not rospy.is_shutdown()) and (not map_1 or not map_2):
			print("All goals reached. \nWaiting for maps from robot1 and robot2...")
			
			if not map1:
				request1 = Data_Sample()
				request1.source = "robot0"
				request1.destination = "robot1"
				map_msg_1 = sample_message()
				map_msg_1.c = "map"
				request1.data = map_msg_1
				send_message(request1,Data_Sample,"sample")
				print("Sent request to robot1..")
	
			if not map2:
				request2 = Data_Sample()
				request2.source = "robot0"
				request2.destination = "robot2"
				map_msg_2 = sample_message()
				map_msg_2.c = "map"
				request2.data = map_msg_2
				send_message(request2,Data_Sample,"sample")
				print("Sent request to robot2..")
				
			rate.sleep()

	#MERGE MAPS FROM ROBOT1 AND ROBOT2
	merge_maps(grid1, grid2)


if __name__ == "__main__":
	rospy.init_node("map_receiver_robot0")
	main()
	rospy.spin()

