#!/usr/bin/env python 

import rospy
import numpy as np
import PyKDL
import math

from geometry_msgs.msg import PoseStamped, Twist
from turtlesim.msg import Pose

__CW__ = 1
__CCW__ = -1


global pub
global x_robot
global y_robot, theta_final

def pose_callback(pose):
    global x_robot, y_robot, theta_final
    print("act_pos: ({0}, {1})".format(x_robot, y_robot))
    print("stpt: ({0}, {1})".format(pose.x, pose.y))
    x = pose.x - x_robot
    y = pose.y - y_robot
    x_robot = x_robot + x
    y_robot = y_robot + y
    print("dr: ({0}, {1})".format(x, y))
    theta_st1 = math.atan2(y,x) 
    print(theta_st1)
    
    theta_spin = theta_st1 - theta_final
    # wyznacz odpowiedni kat i obroc sie w odpowiedni sposob
    if theta_spin > math.pi/2:
        theta_spin = theta_spin - 2*math.pi
    
    if theta_spin < 0:
        spin_elektron(-theta_spin, __CCW__)
    else:
        spin_elektron(theta_spin, __CW__)
        
    dist = math.sqrt(y**2 + x**2)
    move_elektron(pub, dist)
    print("act_pos: ({0}, {1})".format(x_robot, y_robot))

def spin_elektron(angle, direction):
    print("spinning")
    __spin_vel__ = 0.2
    t = rospy.get_time()
    dt = 0
    msg = Twist()
    msg.linear.x = 0.0
    msg.linear.y = 0.0
    msg.linear.z = 0.0
    msg.angular.y = 0.0
    msg.angular.z = 0.0
    while __spin_vel__*dt < angle:
        dt = rospy.get_time() - t
        msg.angular.z = __spin_vel__*direction
        pub.publish(msg)
        #print(dt)
        #print(msg)	
        rospy.sleep(0.1)
        	
    msg_final = Twist()
    msg_final.linear.x = 0.0
    msg_final.linear.y = 0.0
    msg_final.linear.z = 0.0
    msg_final.angular.x = 0.0
    msg_final.angular.y = 0.0
    msg_final.angular.z = 0.0
    pub.publish(msg_final)
    print(msg_final)
    print("finshed spinning")
    
def move_elektron(pub, distance):
    print("moving")
    __linear_vel__ = 0.2
    t = rospy.get_time()
    dt = 0
    msg = Twist()
    msg.linear.x = 0.0
    msg.linear.y = 0.0
    msg.linear.z = 0.0
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = 0.0
    while __linear_vel__*dt < distance:
        dt = rospy.get_time() - t
        msg.linear.x = __linear_vel__
        #msg.linear.y = __linear_vel__
        pub.publish(msg)
        #print(dt)
        #print(msg)	
        rospy.sleep(0.1)
        	
    msg_final = Twist()
    msg_final.linear.x = 0.0
    msg_final.linear.y = 0.0
    msg_final.linear.z = 0.0
    msg_final.angular.x = 0.0
    msg_final.angular.y = 0.0
    msg_final.angular.z = 0.0
    pub.publish(msg_final)
    print(msg_final)
    print("finished moving")

if __name__ == "__main__":
    theta_final = 0.0
    x_robot = 0.0
    y_robot = 0.0
    rospy.init_node('normalmente_movimiento', anonymous=False)
    rospy.Subscriber("new_pose", Pose, pose_callback)
    #rospy.Subscriber("elektron/mobile_base_controller/odom", Pose, callback)
    pub = rospy.Publisher('mux_vel_nav/cmd_vel', Twist, queue_size = 1)
    
    rospy.spin()