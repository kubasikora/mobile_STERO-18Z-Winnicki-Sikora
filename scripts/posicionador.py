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
        srv_name = 'stero/go_to_stpt'
        rospy.wait_for_service(srv_name)
        go_to_stpt = rospy.ServiceProxy(srv_name, STPT)
        point = Pose()
        point.x = float(sys.argv[1])
        point.y = float(sys.argv[2])
        point.theta =  float(sys.argv[3])
        #print("x: {0}, y: {1}, theta: {2}".format(point.x, point.y, point.theta))
        result = go_to_stpt(point)
        print("return code: {0}".format(result))
        print("--------------")   
    else:
        print("Usage: posicionador <x> <y> <theta>\n\r");


   
      
  