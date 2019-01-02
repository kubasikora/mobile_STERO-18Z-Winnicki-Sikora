#!/usr/bin/env python 

import rospy
import numpy as numpy
import PyKDL
import math
import sys

from geometry_msgs.msg import PoseStamped, Twist
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry

__CW__ = 1
__CCW__ = -1

global pub
global x_robot, y_robot, theta_robot

def pose_factory(x=0.0, y=0.0, theta=0.0):
    pose = Pose()
    pose.x = x
    pose.y = y
    pose.theta = theta
    return pose

def twist_factory(x = 0.0, y=0.0, z=0.0, ang_x=0.0, ang_y=0.0, ang_z=0.0):
    twist = Twist()
    twist.linear.x = x
    twist.linear.y = y
    twist.linear.z = z
    twist.angular.x = ang_x
    twist.angular.y = ang_y
    twist.angular.z = ang_z
    return twist

def odometry_callback(odom):
    global x_robot, y_robot, theta_robot
    [x,y,z,w] = [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w]
    [roll,pitch,theta_robot] = PyKDL.Rotation.Quaternion(x,y,z,w).GetRot()
    x_robot = odom.pose.pose.position.x
    y_robot = odom.pose.pose.position.y

def do_line(pub):
    global x_robot, y_robot, theta_robot
    print("Estoy haciendo una prueba de carretera")
    
    # init robot position
    x_robot = 0
    y_robot = 0
    theta_robot = 0

    robot_pos_pub = rospy.Publisher('/elektron/my_position', Pose, queue_size = 10)
    rospy.Subscriber('/elektron/mobile_base_controller/odom', Odometry, odometry_callback)
   
    print("------------")
    print("Actual position: [({0}, {1}), {2}]".format(x_robot, y_robot, theta_robot))
    print("STPT: [(2, 0), 0]")
    stpt_x = 2.0
    stpt_y = 0.0
    stpy_theta = 0.0

    __linear_vel__ = 0.2
    t = rospy.get_time()
    pos_err = 10
    prev_pos_err = pos_err
    
    print("Starting elektron")
    start_msg = twist_factory(x = __linear_vel__)
    print(start_msg)
    pub.publish(start_msg)
    print("Elektron started")
    pos_err = math.sqrt( (stpt_x-x_robot)**2 + (stpt_y-y_robot)**2 )    
    while (0.001 < pos_err) & (pos_err<=prev_pos_err):        
        rospy.sleep(0.05)
        pub.publish(start_msg)
        prev_pos_err = pos_err
        pos_err = math.sqrt( (stpt_x-x_robot)**2 + (stpt_y-y_robot)**2 )
        robot_pos_pub.publish(pose_factory(x_robot, y_robot, theta_robot))
        

    msg = twist_factory()
    pub.publish(msg)
    robot_pos_pub.publish(pose_factory(x_robot, y_robot, theta_robot))

def do_circle():
    global x_robot, y_robot, theta_robot
    print("Estoy haciendo una prueba de carretera")
    
    # init robot position
    x_robot = 0
    y_robot = 0
    theta_robot = 0

    robot_pos_pub = rospy.Publisher('/elektron/my_position', Pose, queue_size = 10)
    rospy.Subscriber('/elektron/mobile_base_controller/odom', Odometry, odometry_callback)
   
    print("------------")
    print("Actual position: [({0}, {1}), {2}]".format(x_robot, y_robot, theta_robot))
    print("STPT: [(2, 0), 0]")
    stpt_x = 2.0
    stpt_y = 0.0
    stpy_theta = 3.14

    __linear_vel__ = 0.2
    t = rospy.get_time()
    pos_err = 10
    prev_pos_err = pos_err
    
    print("Starting elektron")
    start_msg = twist_factory(ang_z = __linear_vel__)
    pub.publish(start_msg)
    print("Elektron started")
    while 0.005 < math.fabs(stpt-theta_robot):
        pub.publish(start_msg)
        rospy.sleep(0.05)

    print("Stopping elektron")
    stop_msg = twist_factory()
    pub.publish(stop_msg)
    print("Elektron has stopped")
    robot_pos_pub.publish(pose_factory(x_robot, y_robot, theta_robot))

def do_square():
    print("do square")

def do_help():
    print("Bienvenidos!")
    print("Sterowanie i Symulacja Robotow. Mini-projekt 3.")
    print("Wykonanie: Konrad Winnicki & Jakub Sikora")
    print("Dostepne argumenty wywolania:\n")
    print("-l, --line:                    test jazdy po linii")
    print("-c, --circle:                  test obrotu")
    print("-s, --square:                  test jazdy po kwadracie")


if __name__ == "__main__":
    __MODES__ = {
        "-l": do_line,
        "--line": do_line,
        "-c": do_circle,
        "--circle": do_circle,
        "-s": do_square,
        "-square": do_square,
        "-h": do_help,
        "--help": do_help
    }   

    try:
        task = __MODES__[sys.argv[1]]
    except Exception:
        print("Argumento incorrecto")
        do_help()
        exit(-1)
    
    rospy.init_node('probador_de_errores', anonymous=False)
    pub = rospy.Publisher('mux_vel_nav/cmd_vel', Twist, queue_size = 10)
    
    task(pub)
