#!/usr/bin/env python

import rospy
from  communication_node.messenger_api import *
from communication_node.msg import *
from sample_package.msg import *
from std_msgs.msg import *
from sensor_msgs.msg import *

a = None
reached = 0
j = 0
ping_id = None
connected_positions = []
goals_list = [[1,1],[1,2],[2,1],[-1,0],[0,-2]]

def send_ping_response(ping_id):

	msg = Data_Location()
	msg.source = "robot1"
	msg.destination = "robot0"
	
	temp_var = loc_message()
	temp_var.x = 0
	temp_var.y = 0
	temp_var.command = "ping_response"
	temp_var.msg_id = ping_id
	msg.data = temp_var
	send_message(msg,Data_Location,"loc")

	print("Sent ping response: with ping_id "+ str(ping_id))


def send_goal_position():
	global j
	global goals_list
	msg = Data_Location()
	msg.source = "robot1"
	msg.destination = "robot0"
	   
	temp_var = loc_message()
	# x coordinate
	temp_var.x = goals_list[j][0]
	# y coordinate
	temp_var.y = goals_list[j][1]
	temp_var.command = "goal"
	temp_var.msg_id = j
	msg.data = temp_var
	send_message(msg,Data_Location,"loc")

	print("Sent goal: "+ str(temp_var.x) + ", " + str(temp_var.y) + "--- goal number"+str(j))




def callback(data):
	global connected_positions
	global reached
	global j
	global goals_list
	global ping_id
	pos = []

	#If goal is reached
	cmd_goal = "goal_reached"
	cmd_ping = "ping"

	if data.data.command == cmd_goal:
		print("Goal " + str(j) +" reached!")
		reached = 1
		j +=1
		pos[0] = data.data.x
		pos[1] = data.data.y
		if pos not in connected_positions:
			connected_positions.append(pos)
				
	#If ping request	
	elif data.data.command == cmd_ping:
		print("Ping request received!")
		ping_id = data.data.msg_id


		
	

def main():
	global a
	global j
	global reached
	global goals_list
	global ping_id
	print("Base Station node started: \n")
	a = receive_message("robot1", Data_Location, "loc",callback)
	
	rate = rospy.Rate(5)
	while (not rospy.is_shutdown() and j<len(goals_list)):
		#SENDS NEW GOAL
		if reached:
			reached =0
			print("Sending new goal coordinates...")
			send_goal_position()
		#SENDS A PING RESPONSE
		elif not ping_id == None:
			print("Sending ping response...")
			send_ping_response(ping_id)

	rate.sleep()
	
	print("Sending node completed execution!")
	#END OF PROGRAM



if __name__ == '__main__':
	rospy.init_node("sending_goal_from_robot1")
	main()
	rospy.spin()




