#!/usr/bin/env python
# coding=utf-8
import rospy
from std_msgs.msg import String,Bool
import threading;
robots=[];
permission_lock=threading.Lock();

class MyWrapper:
    def __init__(self,robot_name_space):
        self.name_space=robot_name_space;
        self.checking_goals_requst_handler=rospy.Subscriber("/"+robot_name_space+"/checking_goals_request", Bool, self.request_handler);
        self.checking_goals_responser=rospy.Publisher("/"+robot_name_space+"/checking_goals_response", Bool,queue_size=15);
    def request_handler(self,input_data):
        global permission_lock;
        if(input_data.data==True):
            print(self.name_space,"permission requested");
            permission_lock.acquire();
            print(self.name_space,"permission granted");
            self.checking_goals_responser.publish(Bool(True));
        elif(input_data.data==False):
            permission_lock.release();
            print(self.name_space,"permission revoked");
            self.checking_goals_responser.publish(Bool(False));

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def main():
    global robots;
    rospy.init_node("permission_granter", anonymous=True)
    robot_list=["robot0","robot1","robot2","robot3"];
    for i in robot_list:
            robots.append(MyWrapper(i));
    rospy.spin();

if __name__ == '__main__':
    main();
