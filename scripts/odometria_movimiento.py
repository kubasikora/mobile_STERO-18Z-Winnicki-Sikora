#!/usr/bin/env python 

import rospy
import numpy as np
import PyKDL
import math

from geometry_msgs.msg import PoseStamped, Twist
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry

__CW__ = 1
__CCW__ = -1


global pub
global x_robot, y_robot, theta_robot

def optimize_theta(theta):
    if theta > math.pi:
        theta = theta - 2*math.pi
    if theta < -math.pi:
        theta = theta + 2*math.pi     
    return theta

def normalize_theta(theta):
    if theta > 2*math.pi:
        theta = theta - 2*math.pi
    if theta < -2*math.pi:
        theta = theta + 2*math.pi 
    return theta

def odometry_callback(odom):
    global x_robot, y_robot, theta_robot
    [x,y,z,w] = [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w]
    [roll,pitch,theta_robot] = PyKDL.Rotation.Quaternion(x,y,z,w).GetRot()
    x_robot = odom.pose.pose.position.x
    y_robot = odom.pose.pose.position.y
    #print("odom act_pos: ({0}, {1}); theta: {2};".format(x_robot, y_robot, theta_robot))    
	
def pose_callback(pose):
    global x_robot, y_robot, theta_robot
    #pozycja aktualna robota
    print("act_pos: ({0:.3f}, {1:.3f}); theta: {2:.3f};".format(x_robot, y_robot, theta_robot))
    #pozycja zadan robota bez kata
    print("stpt: ({0}, {1}, {2})".format(pose.x, pose.y, pose.theta))
    #zmiana pozycji
    dx = pose.x - x_robot
    dy = pose.y - y_robot
    #zmiana kata wynikajajaca ze zmiany pozycji
    theta_st1 = math.atan2(dy,dx) 
    print("(dx, dy, dtheta): ({0:.3f}, {1:.3f}, {2:.3f})".format(dx, dy, theta_st1))
    dist = math.sqrt(dy**2 + dx**2)
    # ograniczenie na minimalne przemieszczenie
    if dist<0.05:
        print("dist {0:.3f}, tylko obrot do zadanego theta".format(dist))
    else:
        # optymalizacja kierunku obrotu
        #kat obrotu robota przed optymalizacja ( przykladowo 2.5pi --> -0.5pi)
        theta_spin = theta_st1 - theta_robot
        theta_spin = optimize_theta(theta_spin)  
        # obrot w odpowiednim kierunku do zadanego kata z odometria
        if theta_spin < 0:
            spin_elektron_odom(theta_st1, __CCW__)
        else:
            spin_elektron_odom(theta_st1, __CW__)
        # poruszenie robotem do zadanego punktu z odometria
        move_elektron_odom(pub, pose.x, pose.y)

    # optymalizacja kierunku obrotu
    #kat obrotu robota przed optymalizacja ( przykladowo 2.5pi --> -0.5pi)
    theta_spin = pose.theta - theta_robot
    theta_spin = optimize_theta(theta_spin)  
    # obrot w odpowiednim kierunku do zadanego kata z odometria
    if theta_spin < 0:
        spin_elektron_odom(pose.theta, __CCW__)
    else:
        spin_elektron_odom(pose.theta, __CW__)

    print("act_pos: ({0:.3f}, {1:.3f}); theta: {2:.3f}".format(x_robot, y_robot, theta_robot))
    print("pos_err: {0:.3f}; theta_err: {1:.3f}".format(math.sqrt( (pose.x-x_robot)**2 + (pose.y-y_robot)**2 ), math.fabs(pose.theta-theta_robot)))
    print("--------------------------------------------------")

def spin_elektron_odom(stpt, direction):
    global theta_robot
    print("spinning with odom")
    __spin_vel__ = 0.1
    t = rospy.get_time()
    dt = 0
    msg = Twist()
    msg.linear.x = 0.0
    msg.linear.y = 0.0
    msg.linear.z = 0.0
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = __spin_vel__*direction
    #poczatek obrotu
    
    #ciagle porownywanie z odometria
    print(theta_robot)
    while 0.01 < math.fabs(stpt-theta_robot):
        print(math.fabs(stpt-theta_robot))
        pub.publish(msg)
        rospy.sleep(0.05)
    # koniec obrotu
    msg.angular.z = 0.0
    pub.publish(msg)
    print("finshed spinning with odom")
    
def move_elektron_odom(pub, stpt_x, stpt_y):
    global x_robot, y_robot
    print("moving with odom")
    __linear_vel__ = 0.2
    t = rospy.get_time()
    pos_err = 10
    prev_pos_err = pos_err
    msg = Twist()
    msg.linear.x = __linear_vel__
    msg.linear.y = 0.0
    msg.linear.z = 0.0
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = 0.0
    
    pub.publish(msg)
    pos_err = math.sqrt( (stpt_x-x_robot)**2 + (stpt_y-y_robot)**2 )
    while (0.001 < pos_err) & (pos_err<=prev_pos_err):        
        rospy.sleep(0.05)
        prev_pos_err = pos_err
        pos_err = math.sqrt( (stpt_x-x_robot)**2 + (stpt_y-y_robot)**2 )
        
    msg.linear.x = 0.0
    pub.publish(msg)
    print("finished moving with odom")

if __name__ == "__main__":
    x_robot = 0
    y_robot = 0
    theta_robot = 0
    rospy.init_node('normalmente_movimiento', anonymous=False)
    rospy.Subscriber("new_pose", Pose, pose_callback)
    rospy.Subscriber('/elektron/mobile_base_controller/odom', Odometry, odometry_callback)
    #rospy.Subscriber("elektron/mobile_base_controller/odom", Pose, callback)
    pub = rospy.Publisher('mux_vel_nav/cmd_vel', Twist, queue_size = 1)
    print("Python desarrollador le gusta colores amarillo y azul")
    rospy.spin()

