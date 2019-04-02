#!/usr/bin/env python

import rospy
from  communication_node.messenger_api import *
from communication_node.msg import *
from sample_package.msg import *
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

a = None
name_space = "robot0"
goal_x = None
goal_y = None
active_x = None
active_y = None
j = 0
check = 0
connected_positions = []
ping_id = None
reached = 0
ping_response = 0


def send_ping():
	global ping_response

	#reset for a new ping_id
	ping_response = 0

	ping = Data_Location()
	ping.source = "robot0"
	ping.destination = "robot1"
	
	temp_var=loc_message()
	
	temp_var.x= active_x
	temp_var.y= active_y
	temp_var.command="ping"
	temp_var.msg_id = ping_id
	feed.data = temp_var
	send_message(feed,Data_Location,"loc")
	print("Sending ping: id " + str(ping_id))



def send_feedback(result):
	global active_x, active_y
	global goal_x, goal_y
	global j
	global reached
	rate = rospy.Rate(5)
	while not rospy.is_shutdown():		
		if result:
			feed = Data_Location()
			feed.source = "robot0"
			feed.destination = "robot1"
			
			temp_var=loc_message()
			
			temp_var.x= active_x
			temp_var.y= active_y
			temp_var.command= "goal_reached"
			feed.data = temp_var
			send_message(feed,Data_Location,"loc")
			print("Sending feedback: goal " + str(j) + "reached!")
			reached = 1
			j+=1

		if active_x!=goal_x or active_y!=goal_y:
			break

		rate.sleep()

		if active_x!=goal_x or active_y!=goal_y:
			break

def movebase_client():

	global goal_x
	global goal_y
	global active_x, active_y
	global ping_id
	global reached
	global ping_response

	active_x = goal_x
	active_y = goal_y

	client = actionlib.SimpleActionClient("/"+name_space+"/move_base", MoveBaseAction)
	client.wait_for_server()
	
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = "map"
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.position.x = active_x
	goal.target_pose.pose.position.y = active_y
	goal.target_pose.pose.orientation.w = 1.0

	print("Going to "+ "(" +str(active_x)+", "+ str(active_y)+")")
	reached = 0

	client.send_goal(goal=goal, done_cb=send_feedback())
	
				
	'''
	wait = client.wait_for_result()
	if not wait:
		rospy.logerr("Action server not available!")
		rospy.signal_shutdown("Action server not available!")
	else:
		return client.get_result()
	'''


def callback(data):
	global goal_x
	global goal_y
	global ping_id
	global ping_response

	if data.data.command == "goal":
		print ("Goal plan received:")
		print(str(data.data.x)+", "+str(data.data.y)+" --- goal number "+str(data.data.msg_id)) 
		goal_x = data.data.x
		goal_y = data.data.y

	elif data.data.command == "ping_response":
		if data.data.msg_id == ping_id:
			ping_response =1
			ping_id +=1



def main():
	global a
	global goal_x
	global goal_y
	global check 
	global j
	global reached
	global ping_id
	global ping_response
	global connected_positions

	rospy.init_node('movebase_client_py')

	print ("Goal receiver started:")
	

	a=receive_message("robot0", Data_Location, "loc",callback)
	rate = rospy.Rate(0.1)	
	while not rospy.is_shutdown():
		while not rospy.is_shutdown():
			rate.sleep()
			if not goal_x == None:
				break
	
		try:
			#send to goal and check connection
			if reached or j==0:
				print("Moving to new goal...")
				movebase_client()
			#Sends ping before reaching the goal
			if not reached:
				print("Sending ping request..")
				send_ping()		

			if not ping_id == None and not ping_response and len(connected_positions)>0:
				goal_x = connected_positions[-1][0]
				goal_y = connected_positions[-1][1]
				print("Gone back to connected position!")
				

		except rospy.ROSInterruptException:
			rospy.loginfo("Navigation test finished.")
	
	rospy.spin()

if __name__ == '__main__':
	main()

	
