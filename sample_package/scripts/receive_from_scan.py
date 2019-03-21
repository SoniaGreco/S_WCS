#!/usr/bin/env python

import rospy
from  communication_node.messenger_api import *
from communication_node.msg import *
from sample_package.msg import *


def callback(data):

    print ("LaserScan info received:")
    rospy.loginfo("%f", data.data.b)


def main():
    global a;
    print ("LaserScan receiver started:")
    #rospy.Subscriber("robot1" + "/inbox_"+ "sample", Data_Sample, callback)
    a=receive_message("robot1", Data_Sample, "sample",callback)
    


if __name__ == "__main__":
    rospy.init_node("receive_robot1")
    main()
    rospy.spin()
