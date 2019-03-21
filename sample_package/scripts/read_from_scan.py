#!/usr/bin/env python

import rospy
from  communication_node.messenger_api import *
from communication_node.msg import *
from sample_package.msg import *
from std_msgs.msg import *
from sensor_msgs.msg import *

laserData = 0

def callback(msg):
    
    global laserData
    #Sends info about one ray of the scan
    laserData = msg.ranges[500]
    

def main():
    print("Sending node started: \n")
    msg = Data_Sample()
    msg.source = "robot0"
    msg.destination = "robot1"
    rospy.Subscriber("/robot0/hokuyo", LaserScan, callback)
    rate = rospy.Rate(100)
    i = 0
    while not rospy.is_shutdown():
            temp_var=sample_message();
	    temp_var.a=i

            #Sends data in variable b
	    temp_var.b= laserData

	    temp_var.c=str(i)+"--"+str(i*2)
	    msg.data = temp_var
	    send_message(msg,Data_Sample,"sample")
	    rospy.loginfo("sent laser info: %f", laserData)
	    rate.sleep()
            i = i+1
	    

if __name__ == '__main__':
    rospy.init_node("scan_robot0")
    main()
    rospy.spin()




