#!/usr/bin/env python
# coding=utf-8
import rospy;
import math;
import threading;
import numpy as np;
from random import random;
from visualization_msgs.msg import Marker,MarkerArray;
from nav_msgs.srv import GetPlan ,GetPlanRequest;
from nav_msgs.msg import Path , OccupancyGrid, Odometry;
from communication_node.msg import Data_Goal,Data_Map;
from geometry_msgs.msg import Point , PoseStamped
from std_msgs.msg import Bool;
from Algorithms import *;
import roslib;
import actionlib;
from actionlib_msgs.msg import *;
from move_base_msgs.msg import *;
from frontier_search import *

a_star=None;
name_space="robot1";
robot_number=0;
number_of_robots=0;
######################
merged_map_lock=threading.Lock()
merged_map=None;
#####################
map_publisher=None;
goal_publisher=None;
#############
odom_subscriber=None;
robot_x=None;
robot_y=None;
#################
beta=1;
alpha=25;
utility=100;
laser_range=30;
#################
my_server=None;
my_goals=[None];
goals_list=[];
goals_list_lock=threading.Lock();
other_robots_list=[];
#############################
checking_goals_flag=False;
checking_goals_subscriber=None;
checking_goals_publisher=None;
############################
map_pub_frequnecy=2;
map_pub_counter=0;
###############################
move_client_=None
move_client_goal_=None;
goal_pose=PoseStamped();
current_goal_status = 0 ; # goal status--- PENDING=0--- ACTIVE=1---PREEMPTED=2--SUCCEEDED=3--ABORTED=4---REJECTED=5--PREEMPTING=6---RECALLING=7---RECALLED=8---LOST=9
move_base_status_subscriber=None;
#########################
test_frontier_visualization_publisher=None;
test_connection_visualization_publisher=None;
#########################

class MyWrapper:
    def __init__(self,list_index,robot_name_space):
        self.list_index=list_index;
        self.robot_name_space=robot_name_space;
        self.map_subscriber=rospy.Subscriber("/"+name_space+"/inbox_Map", Data_Map, self.set_Map);
        self.goal_subscriber = rospy.Subscriber("/"+name_space+"/inbox_Goal", Data_Goal, self.set_Goal);
        self.odom_subscriber=rospy.Subscriber("/"+robot_name_space+"/odom", Odometry, self.odom_callback);
        self.robot_x=None;
        self.robot_y=None;
    def odom_callback(self,odom_data):
        self.robot_x=odom_data.pose.pose.position.x;
        self.robot_y=odom_data.pose.pose.position.y;
    def set_Map(self,map_data):
        global merged_map_lock;
        global merged_map;
        merged_map_lock.acquire();
        if(merged_map==None):
            merged_map_lock.release();
            return;
        temp_map=np.array([map_data.data.data,merged_map.data]);
        merged_map.data=list(np.max(temp_map,axis=0));
        merged_map_lock.release();
    def set_Goal(self, goal_data):
        global goals_list,goals_list_lock;
        goals_list_lock.acquire()
        if (goal_data.source==self.robot_name_space):
            goals_list[self.list_index]=goal_data.data;
        goals_list_lock.release();

################################################
################################################
def frontier_is_new(new_frontier,frontiers_list):
    for i in frontiers_list:
        if(((new_frontier[0]-i[0]) ** 2 + (new_frontier[1]-i[1]) ** 2)<5):
            return False;
    return True;

def get_frontiers(map_data):
        global robot_x,robot_y;
        global test_frontier_visualization_publisher,name_space;
        rospy.sleep(1.0);
        frontiers=[];
        while(robot_x==None or robot_y==None):
            rospy.sleep(0.5);
        print(name_space,"x",robot_x,"y",robot_y)
        fsc=FrontierSearch(map_data,5,"balanced");
        test_frontiers=fsc.searchFrom(Point(robot_x,robot_y,0.0));
        frontiers=list(test_frontiers[0]);
        test_markers=[];
        test_id=0;
        for i in range(len(test_frontiers[1])):
            gr=random();
            red=random();
            blue=random();
            for j in test_frontiers[1][i]:
                test_marker = Marker()
                test_marker.header.frame_id = "/map"
                test_marker.header.stamp = rospy.Time.now()
                test_marker.ns = "basic_shapes"
                test_marker.id = test_id
                test_id+=1;
                test_marker.type = int(name_space[-1])+1;
                test_marker.action = Marker.ADD
                test_marker.pose.position.x = j.x
                test_marker.pose.position.y = j.y
                test_marker.pose.position.z = 0
                test_marker.pose.orientation.x = 0.0
                test_marker.pose.orientation.y = 0.0
                test_marker.pose.orientation.z = 0.0
                test_marker.pose.orientation.w = 1.0
                test_marker.scale.x = 0.1
                test_marker.scale.y = 0.1
                test_marker.scale.z = 0.1
                test_marker.color.r = red;
                test_marker.color.g = gr;
                test_marker.color.b = blue;
                test_marker.color.a = 1.0
                test_marker.lifetime = rospy.Duration(10.0)
                test_markers.append(test_marker);
        test_frontier_visualization_publisher.publish(test_markers);
        return list(frontiers);
        '''
        map_width=int( map_data.info.width); #max of x
        map_height=int(map_data.info.height);#max of y
        map_size=map_height*map_width;
        temp_list=[-1,0,1];
        for y in range(1,map_height-1):
            for x in range(1,map_width-1):
                counter=0;
                if map_data.data[(y*map_width)+x]==0:
                    for i in temp_list:
                        for j in temp_list:
                            if not(j==i and i==0):
                                if(map_data.data[(y+i)*map_width+(x+j)]<0):
                                    counter+=1;
                                if(map_data.data[(y+i)*map_width+(x+j)]>10):
                                    counter=10;
                    if (counter==3):
                        temp_x=(x)*map_data.info.resolution+map_data.info.origin.position.x;
                        temp_y=(y)*map_data.info.resolution+map_data.info.origin.position.y;
                        if(frontier_is_new([temp_x,temp_y],frontiers)==True):
                            frontiers.append([temp_x,temp_y]);


        print("this is number of fronteirs", len(frontiers))
        return list(frontiers);
        '''

def compute_frontier_distance(frontiers):
    global robot_x,robot_y;
    frontier_distances=[];
    temp=-2;
    temp1=0;
    temp2=0;
    for i in frontiers:
        temp=-2;
        while temp==-2:
            temp=request(robot_x,robot_y,i.travel_point.x,i.travel_point.y);
            if(temp==10000000):
                temp1+=1;
            else:
                temp2+=1;
        frontier_distances.append([i[0],i[1],utility-beta*temp]);
    print(name_space,"no path",temp1 ,"path",temp2)
    return list(frontier_distances);

################################################
################################################
def callback_goal_status(data,data2):
    global current_goal_status;
    current_goal_status=True;
    return;
    if len(data.status_list)==0 :
        return;
    current_goal_status = data.status_list[len(data.status_list) - 2].status;

def move_base_tools():
    global move_client_goal_;
    global move_client_;
    global name_space;
    global move_base_status_subscriber;
    move_client_=actionlib.SimpleActionClient("/"+name_space+"/move_base", MoveBaseAction);
    move_client_goal_=MoveBaseGoal();
    print(name_space,"move base tools are ok")
    #move_base_status_subscriber=rospy.Subscriber("/"+name_space+"/move_base/status", GoalStatusArray, callback_goal_status);


################################################
################################################################################################
################################################



def request(sx,sy,gx,gy):
    global a_star;
    temp_object=Problem(sx=int(merged_map.info.resolution*(sx-merged_map.info.origin.position.x)),sy=int(merged_map.info.resolution*(sy-merged_map.info.origin.position.y)),gy=int(merged_map.info.resolution*(gy-merged_map.info.origin.position.y)),gx=int(merged_map.info.resolution*(gx-merged_map.info.origin.position.x)),matrix=list(merged_map.data),width=int( merged_map.info.width),height=int( merged_map.info.height));
    a_star.problem=temp_object;
    path=a_star.Astar_graph();
    if path==None:
        return 10000000;
    return path;

def send_goal(goal_x,goal_y):
    global other_robots_list,goal_publisher;
    global name_space;
    global move_client_;
    global move_client_goal_;
    global goal_pose;
    global my_current_goal;
    my_current_goal=Point(goal_x,goal_y,0.0);
    for i in other_robots_list:
        new_data=Data_Goal();
        new_data.source=name_space;
        new_data.destination=i.robot_name_space;
        new_data.data=my_current_goal;
        goal_publisher.publish(new_data);

        # set goal
    goal_pose.pose.position.x = goal_x;
    goal_pose.pose.position.y = goal_y;
    goal_pose.pose.orientation.w = 1.0;
    goal_pose.pose.orientation.z = 0;
    goal_pose.header.frame_id = "/map";
    goal_pose.header.stamp = rospy.Time.now();
        # send goal
    move_client_.cancel_goals_at_and_before_time(rospy.Time.now());
    move_client_goal_.target_pose=goal_pose;
    rospy.sleep(0.5);
    checking_goals_publisher.publish(Bool(False));
    #move_client_.send_goal_and_wait(goal=move_client_goal_,execute_timeout = rospy.Duration(300),preempt_timeout = rospy.Duration(1));
    move_client_.send_goal(goal=move_client_goal_,done_cb=callback_goal_status);
    print(name_space,"sent goal");
    goal_pose.header.seq =goal_pose.header.seq+1 ;
################################################
################################################
def map_callback(map_data):
    global merged_map,merged_map_lock;
    global other_robots_list;
    global map_publisher;
    global map_pub_counter,map_pub_frequnecy;
    merged_map_lock.acquire();
    if (merged_map==None):
        merged_map=map_data;
    else:
        temp_map2=list(merged_map.data);
        merged_map=map_data;
        temp_map=np.array([map_data.data,list(temp_map2)]);
        merged_map.data=list(np.max(temp_map,axis=0));
    merged_map_lock.release();
    if (map_pub_counter==0):
        for i in other_robots_list:
            if map_publisher==None:continue;
            new_data=Data_Map();
            new_data.source=name_space;
            new_data.destination=i.robot_name_space;
            new_data.data=map_data;
            map_publisher.publish(new_data);
    elif(map_pub_counter>=10):
        map_pub_counter=0;
    else:
        map_pub_counter+=map_pub_frequnecy;


def odom_callback(odom_data):
    global robot_x,robot_y;
    robot_x=odom_data.pose.pose.position.x;
    robot_y=odom_data.pose.pose.position.y;
################################################
################################################

def burgard():
    global merged_map_lock;
    global merged_map;
    global name_space;
    global goals_list;
    global goals_list_lock;
    global alpha;
    global checking_goals_publisher,checking_goals_flag;
    global current_goal_status,move_client_;
    global goal_publisher,my_current_goal,other_robots_list,my_goals;
    global test_connection_visualization_publisher;
    while(merged_map==None):
        pass;
    while not rospy.is_shutdown():
        merged_map_lock.acquire();
        print(name_space,"going for frointiers")
        frontiers=get_frontiers(merged_map);
        merged_map_lock.release();
        if (len(frontiers)==0):
            print(name_space,"no new frontiers");
            exit();
        #frontiers=compute_frontier_distance(frontiers);
        print(name_space,"we have frontiers",len(frontiers));
        if (len(frontiers)==0):
            print(name_space,"no path to frointiers");
            exit();
        rate = rospy.Rate(0.5);
        for k in range(int(name_space[-1]),number_of_robots-1):
            while goals_list[k]==None:
                pass;

        checking_goals_publisher.publish(Bool(True));
        rate = rospy.Rate(0.5);
        while(not checking_goals_flag):
            rate.sleep();
        rospy.sleep(0.5);
        for i in range(0,len(frontiers)):
            goals_list_lock.acquire();
            for j in goals_list:
                if(j==None):continue;
                temp_distance=math.sqrt( (j.x-frontiers[i].travel_point.x)**2 +  (j.y-frontiers[i].travel_point.y)**2);
                if(temp_distance<=laser_range):
                    print(name_space,"before increment",str(frontiers[i].min_distance))
                    frontiers[i].min_distance+=alpha*(1-temp_distance/laser_range);
                    print(name_space,"after ",str(frontiers[i].min_distance))

                else:
                    print(name_space," goal out of range ",str(j.x),str(j.y));
                for j in my_goals:
                    if(j==None):continue;
                    temp_distance=math.sqrt( (j.x-frontiers[i].travel_point.x)**2 +  (j.y-frontiers[i].travel_point.y)**2);
                    if(temp_distance<=laser_range/10):
                        print(name_space,"before increment",str(frontiers[i].min_distance))
                        frontiers[i].min_distance+=100;
                        print(name_space,"after ",str(frontiers[i].min_distance))

                    else:
                        print(name_space," goal out of range ",str(j.x),str(j.y));
            goals_list_lock.release();

        print(name_space,"sorting");
        frontiers.sort(key=lambda node: node.min_distance);
        print(name_space,"worst frontier",frontiers[-1].min_distance,"  best frontier",frontiers[0].min_distance);
        print(name_space,"  goal is ",str(frontiers[0].travel_point.x),str(frontiers[0].travel_point.y));
        current_goal_status=False;
        my_goals[0]=frontiers[0].travel_point;
        send_goal(frontiers[0].travel_point.x,frontiers[0].travel_point.y);
        rospy.sleep(3.0);
        time_counter=0;
        test_robots_list=rospy.get_param("/robots_list");
        rate = rospy.Rate(1);
        while current_goal_status==False and time_counter<90:
            rate.sleep();
            time_counter+=2;
            test_tempx=0;
            test_tempy=0;
            test_connection_list=(rospy.get_param("/direct_connection_list_"+name_space));
            test_connection_markers=[];
            for i in range (0,int(name_space[-1])):
                test_tempx=None;
                test_tempx=other_robots_list[i].robot_x;
                test_tempy=other_robots_list[i].robot_y;
                print(name_space,"found",other_robots_list[i].robot_name_space)
                if(test_tempx==None):continue;
                test_marker = Marker();
                test_marker.header.frame_id = "/map";
                test_marker.header.stamp = rospy.Time.now();
                test_marker.ns = "points_and_lines";
                test_marker.id = i;
                test_marker.type = Marker.LINE_STRIP;
                test_marker.action = Marker.ADD;
                test_marker.pose.orientation.x = 0.0;
                test_marker.pose.orientation.y = 0.0;
                test_marker.pose.orientation.z = 0.0;
                test_marker.pose.orientation.w = 1.0;
                test_marker.scale.x = 0.1;
                test_temp_robotname=test_robots_list.index(other_robots_list[i].robot_name_space);
                test_marker.color.r = 1-int(test_connection_list[min(int(test_temp_robotname)+1,4)]);
                test_marker.color.g = int(test_connection_list[min(int(test_temp_robotname)+1,4)]);
                test_marker.color.b = 0.5;
                test_marker.color.a = 1.0;
                test_marker.points.append(Point(robot_x,robot_y,0.0));
                test_marker.points.append(Point(test_tempx,test_tempy,0.0));
                test_marker.lifetime = rospy.Duration(60.0);
                test_connection_markers.append(test_marker);
            test_connection_visualization_publisher.publish(test_connection_markers);
            for i in other_robots_list:
                new_data=Data_Goal();
                new_data.source=name_space;
                new_data.destination=i.robot_name_space;
                new_data.data=my_current_goal;
                goal_publisher.publish(new_data);
        current_goal_status=False;
        move_client_.cancel_goals_at_and_before_time(rospy.Time.now());


def checking_goals_response_callback(input_data):
    global checking_goals_flag;
    checking_goals_flag=input_data.data;


def main():
    global name_space,robot_number,number_of_robots;
    global merged_map,goals_list,other_robots_list;
    global goal_publisher,a_star;
    global map_publisher,odom_subscriber;
    global checking_goals_subscriber,checking_goals_publisher;
    global test_frontier_visualization_publisher,test_connection_visualization_publisher;
    rospy.init_node("burgard_exploration_node");
    a_star=Algorithmes();
    name_space = rospy.get_param("namespace", default="robot1");
    robot_number=int(name_space[-1]);
    number_of_robots=(int(rospy.get_param("number_of_robots", default=1)));
    temp_i=0;
    for i in range (0,number_of_robots):
        if (i==robot_number):continue;
        goals_list.append(None);
        other_robots_list.append(MyWrapper(list_index=temp_i,robot_name_space="robot"+str(i)));
        temp_i+=1;
    move_base_tools();
    test_connection_visualization_publisher=rospy.Publisher("/"+name_space+"/connection_graph", MarkerArray, queue_size=100)
    test_frontier_visualization_publisher=rospy.Publisher("/"+name_space+"/frontier_marker", MarkerArray, queue_size=100)
    map_subscriber=rospy.Subscriber("/"+name_space+"/map", OccupancyGrid, map_callback);
    odom_subscriber=rospy.Subscriber("/"+name_space+"/odom", Odometry, odom_callback);
    goal_publisher=rospy.Publisher("/message_server_Goal", Data_Goal,queue_size=15);
    map_publisher=rospy.Publisher("/message_server_map", Data_Map,queue_size=15);
    checking_goals_subscriber=rospy.Subscriber("/"+name_space+"/checking_goals_response", Bool, checking_goals_response_callback);
    checking_goals_publisher=rospy.Publisher("/"+name_space+"/checking_goals_request", Bool,queue_size=15);
    burgard();
    rospy.spin();

if __name__ == '__main__':
    main();
