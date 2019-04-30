#!/usr/bin/env python

#
# EXPLORING ROBOT IN ROLE-BASED STRATEGY:
# 
# Its goal is to find new frontiers and reach them,
# while sharing information, like its position and
# its map, to all the connected robots.
# When updates are received, the closest robot to the BS goes back
# to the base station and shares the data.
#

import rospy
from communication_node.messenger_api import *
from communication_node.msg import *
from sample_package.msg import *
from std_msgs.msg import *
from nav_msgs.msg import *
from sensor_msgs.msg import *
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import numpy as np
import math
from frontier_search import *


robot_is_moving = 0
connection_list = []
original_x = None
original_y = None

#VALUES TO SET
base_x = 0
base_y = 0
##############


start = 1

frontiers = []
robots_goals_list = []
robots_list = []

base_connected = 0
grid = None
header = None
info= None
data = None

goal_x = None
goal_y = None
active_x = None
active_y = None

merged_map = []
base_map = []
going_to_base = 0

robot1_goal_x = None
robot1_goal_y = None

final = 0
stop = 0

covered_distance = 0

starting_time = None

new_data = 0

def update_map(map):
	global header, info, merged_map, grid
	header = map.header
	info = map.info
	grid = map
	if len(merged_map)==0: 
		merged_map = map.data
	else:
		merge_maps(map.data)
	
	grid.data = merged_map
	#print("\nUpdating map.. \n")

def merge_maps(new_map):
	global merged_map, grid
	new_map_np = np.array(new_map, dtype= np.int8) 
	merged_np = np.array(merged_map, dtype= np.int8) 
	
	unknown_index = (merged_np==-1)

	#Delete -1 values from previous map and replace them with values from new map
	merged_np[unknown_index] = new_map_np[unknown_index]

	zero_indexes = (new_map_np == 0)
	merged_np[zero_indexes] = 0

	known_index = np.logical_and(merged_np>0, new_map_np>0)
	merged_np[known_index] = 100

	merged_map = np.ndarray.tolist(merged_np)

	if not grid == None:
		grid.data = merged_map
	
	

def share_data(robot_i):
	global merged_map, grid
	global goal_x, goal_y
	print("Robot2 connected to ROBOT"+ str(robot_i)+ " - Sending map and goal position...")
	i=0
	while i<2:
		if not grid== None:
			
			msg = Data_Sample()
			msg.source = "robot2"
			msg.destination = "robot"+str(robot_i)
			
			temp_var=sample_message()
			temp_var.a = goal_x
			temp_var.b = goal_y
			temp_var.c = "goal"
			msg.data = temp_var
			send_message(msg,Data_Sample,"sample")
			#print("Sent goal position to robot"+ str(robot_i)+ "! ")

			msg = Data_Map()
			msg.source = "robot2"
			msg.destination = "robot"+str(robot_i)
			msg.command = "map"
			temp_var=OccupancyGrid()
			temp_var = grid
			msg.data = temp_var
			send_message(msg,Data_Map,"map")
			#print("Sent occupancy grid to robot"+ str(robot_i)+ "! ")
			
		i+=1


def check_connected_robots():
	global connection_list
	global base_connected, robots_list, base_map, going_to_base

	connection_list=[]

	connection_list=(rospy.get_param("/connection_list_robot2"))

	base = robots_list.index("robot0")   
	connection=connection_list[1+base]
	if connection:
		base_connected =1
		base_map = merged_map
	else: 
		base_connected = 0
		base_map_np = np.array(base_map, dtype = np.int8)
		merged_map_np = np.array(merged_map, dtype = np.int8)
		#number of known elements at the base
		known_ind = (base_map_np != -1)
		known_elements = len(known_ind)

		diff = merged_map_np-base_map_np
		not_zero = (diff !=0)
		#number of new elements
		new_elements = len(not_zero)

		#
		# IF ROBOT COLLECTED A LOT OF DATA WITHOUT SHARING IT, IT GOES BACK TO THE BASE STATION
		#
		if known_elements / (known_elements + new_elements) < 0.8:
			going_to_base = 1

	for i in range (0, len(robots_list)):
		if not i == 1:
			robot_i = robots_list.index("robot"+str(i))   
			connection=connection_list[1+robot_i]
			if connection:
				share_data(i)
				
				
	
	
def check_nearest_robot_to_base(other_bot_goal_x, other_bot_goal_y):
	global goal_x
	global goal_y, base_x, base_y
	global merged_map, base_connected, going_to_base

	my_distance = math.sqrt( (goal_x-base_x)**2 +  (goal_y-base_y)**2)
	other_distance = math.sqrt( (other_bot_goal_x-base_x)**2 +  (other_bot_goal_y-base_y)**2)
		  
	if my_distance < other_distance:
		print("Need to go back to the BS!")
		going_to_base = 1

def callback_goal(mess):
	global goal_x
	global goal_y, robot2_goal_x, robot2_goal_y
	global merged_map, base_connected, going_to_base, robots_goals_list
	 
	if mess.data.c == "goal":
		#print ("Received goal position from "+str(mess.source)+"!")
		pos_x = mess.data.a
		pos_y = mess.data.b

		robot1_goal_x = pos_x
		robot1_goal_y = pos_y

		robots_goals_list.append((pos_x, pos_y))


		
def callback_map(mess):
	global goal_x
	global goal_y, robot2_goal_x, robot2_goal_y
	global merged_map, base_connected, going_to_base, robots_goals_list

	if mess.command == "map":
		#print("Received map from "+str(mess.source)+"! \nMerging maps...")
		merge_maps(mess.data.data)
		

	if not base_connected and not ord(mess.source[5])==0 and not robot2_goal_x==None:
		check_nearest_robot_to_base(robot2_goal_x, robot2_goal_y)


def move_base_callback(r1,r2):
	global robot_is_moving, going_to_base, stop
	print("Goal reached!")
	robot_is_moving = 0
	if final:
		stop = 1
	if going_to_base:
		going_to_base = 0

def movebase_client():

	global goal_x
	global goal_y
	global active_x, active_y
	global robot_is_moving 

	robot_is_moving = 1

	active_x = goal_x
	active_y = goal_y

	client1 = actionlib.SimpleActionClient("/robot2/move_base", MoveBaseAction)
	client1.wait_for_server()
	
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = "/map"
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.position.x = active_x
	goal.target_pose.pose.position.y = active_y
	goal.target_pose.pose.orientation.w = 1.0

	print("Going to "+ "(" +str(active_x)+", "+ str(active_y)+")")

	client1.send_goal(goal=goal, done_cb=move_base_callback)

def send_done():
	global active_x, active_y, final
	global goal_x, goal_y, original_x, original_y
	rate = rospy.Rate(5)

	i=0
	print("Sending final message. Exploration is over, going back to original pose...")

	while i<2:
		feed = Data_Map()
		feed.source = "robot2"
		feed.destination = "robot0"
		feed.command = "done"
		
		temp_var=OccupancyGrid()
		
		feed.data = temp_var
		send_message(feed,Data_Map,"map")
		
		i+=1

	final = 1
	goal_x = original_x
	goal_y = original_y
	movebase_client()
	
def odom_callback(odom_data):
	global robot_x,robot_y, original_x, original_y, covered_distance
	if original_x == None:
		original_x = odom_data.pose.pose.position.x
		original_y = odom_data.pose.pose.position.y
		#print"Got original pose!\n"

	else: 

		current_x=odom_data.pose.pose.position.x
		current_y=odom_data.pose.pose.position.y
		#Uses previously stored position in robot_x, robot_y
		covered_distance += math.sqrt(pow((current_x - robot_x),2) + pow((current_y - robot_y), 2))
	
	#Set new current position
	robot_x= odom_data.pose.pose.position.x
	robot_y=odom_data.pose.pose.position.y

	if not starting_time == None:
		time = rospy.get_time()
		delta = time - starting_time
		if abs(delta % 60) <1:
			print("Timestamp "+str(delta)+" seconds: travelled distance = " + str(covered_distance))




	
def compute_frontier_distance(frontiers):
	global robot_x,robot_y
	frontier_distances=[]
	for i in range(0, len(frontiers)):
		distance = math.sqrt( (robot_x-frontiers[i].travel_point.x)**2 +  (robot_y-frontiers[i].travel_point.y)**2)
		frontier_distances.append(distance)
	return list(frontier_distances)



def get_frontiers(map_data):
		global robot_x,robot_y
		rospy.sleep(1.0)
		frontiers=[]
		while(robot_x==None or robot_y==None):
			rospy.sleep(0.5)
		print("\nGetting frontiers..")
		fsc=FrontierSearch(map_data,10,"balanced")
		test_frontiers=fsc.searchFrom(Point(robot_x,robot_y,0.0))
		frontiers=list(test_frontiers[0])
		test_id=0
		return list(frontiers)

def explore():

	global start, grid, goal_x, goal_y
	global robot_is_moving, robots_goals_list, robots_list

	frontiers=get_frontiers(grid)
	if (len(frontiers)==0):
		print("No new frontiers")
		send_done()
		return

	print("\nDetected " + str(len(frontiers)) + " frontiers")

	distances = compute_frontier_distance(frontiers)
	
	if start:
		#If it is the first decision, goal is set to the closest frontier
		nearest = distances.index(min(distances))

		goal_x = frontiers[nearest].travel_point.x
		goal_y = frontiers[nearest].travel_point.y

		movebase_client()
		
		start = 0
	
	else:
		
		for g in robots_goals_list:
			for f in range(0,len(frontiers)):
				dist = math.sqrt((g[0]-frontiers[f].travel_point.x)**2 +  (g[1]-frontiers[f].travel_point.y)**2)
				if dist < 30:
					penalty = 10 * (1- dist/30 )
					distances[f]=distances[f]+penalty
		
		nearest = distances.index(min(distances))

		if not frontiers[nearest].travel_point.x == goal_x or not frontiers[nearest].travel_point.y == goal_y:
			goal_x = frontiers[nearest].travel_point.x
			goal_y = frontiers[nearest].travel_point.y
		elif len(frontiers)>1:
			second = distances.index(max(distances))
			goal_x = frontiers[second].travel_point.x
			goal_y = frontiers[second].travel_point.y
		else:
			if (len(frontiers)==0):
				print("No new frontiers")
				send_done()
				return

		#RESET ROBOTS_GOALS_LIST
		robots_goals_list = []


		movebase_client()
		

def initial_move():
	global goal_x, goal_y
	global robot_x, robot_y

	goal_x = robot_x+1
	goal_y = robot_y-1

		

def main():

	global a, merged_map
	global goal_x, base_connected
	global goal_y, robot_is_moving, final, stop
	global going_to_base, robots_list, robots_goals_list, starting_time

	print("\nRobot2 is active: exploration started!\n")

	robots_list=rospy.get_param("/robots_list")

	a=receive_message("robot2", Data_Map, "map", callback_map)
	b=receive_message("robot2", Data_Sample, "sample", callback_goal)

	rospy.Subscriber("/robot2/map", OccupancyGrid, update_map)
	rospy.Subscriber("/robot2/odom", Odometry, odom_callback)
	#rospy.Subscriber("/robot2/move_base/NavfnROS/plan", Path, path_callback)

	starting_time = rospy.get_time()

	rate = rospy.Rate(5)
	while not rospy.is_shutdown():
		rate.sleep()
		
		if not original_x == None:
			break
	
	initial_move()	

	movebase_client()
	while not rospy.is_shutdown():
		rate.sleep()
		if not robot_is_moving and not len(merged_map)==0:
			print("Initial move done!")
			break

	rate = rospy.Rate(10)
	while not rospy.is_shutdown() and not stop:

		rate.sleep()

		if robot_is_moving:
			#Checks if there are connected robots and sends them map and goal position
			#otherwise continues towards the goal
			check_connected_robots()

			if base_connected and going_to_base:
				going_to_base = 0
				explore()
			
		else:
			if not stop:
				#If robot reached the goal or the base station
				print("\nRobot2 deciding new task..")

				#If it needs to go back to the base station, the goal is set to its original pose
				if going_to_base:
					goal_x = original_x
					goal_y = original_y

					print("Going back to base station!")
					movebase_client()


				#If it doesnt need to go back to the base station, it searches for another frontier
				else:
					explore()

				

if __name__ == '__main__':
	rospy.init_node("map_robot2")
	main()
	rospy.spin()
	