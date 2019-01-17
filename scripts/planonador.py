#!/usr/bin/env python 

import rospy
import numpy as np
import PyKDL
import math
import sys

from turtlesim.msg import Pose
from stero_mobile_init.srv import STPT


if __name__ == "__main__":
    if len(sys.argv)==4:
        rospy.init_node('planonador', anonymous=True)
        go_to_stpt = rospy.Publisher('/stero/set_goal', Pose, queue_size = 10)
        pose= Pose()
        pose.x = float(sys.argv[1])
        pose.y = float(sys.argv[2])
        pose.theta =  float(sys.argv[3])
        #print("x: {0}, y: {1}, theta: {2}".format(point.x, point.y, point.theta))
        rospy.sleep(1)
        go_to_stpt.publish(pose)
        print("--------------")   
    else:
        print("Usage: posicionador <x> <y> <theta>\n\r");


   
      
  