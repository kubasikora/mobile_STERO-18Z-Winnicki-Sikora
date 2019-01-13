#!/usr/bin/env python 

import rospy
import numpy as np
import PyKDL
import math
import tf

from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovariance, Pose2D, Point
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetPlan
from stero_mobile_init.srv import STPT

global map_pose
global x_robot, y_robot, theta_robot

def tf_callback(pose):
    global map_pose

def odometry_callback(odom):
    global x_robot, y_robot, theta_robot
    [x,y,z,w] = [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w]
    [roll,pitch,theta_robot] = PyKDL.Rotation.Quaternion(x,y,z,w).GetRot()
    x_robot = odom.pose.pose.position.x
    y_robot = odom.pose.pose.position.y
    #print("odom act_pos: ({0}, {1}); theta: {2};".format(x_robot, y_robot, theta_robot))    
	
    
def create_start():
    start = PoseStamped()
    start.header.frame_id="base_link"
    time = rospy.Time()
    listener = tf.TransformListener()
    listener.waitForTransform("/map", "/base_link", time, rospy.Duration(4.0))
    (pos, quaternion) = listener.lookupTransform('/map', '/base_link', time)
    ps = PoseStamped()
    ps.pose.position.x = pos[0]
    ps.pose.position.y = pos[1]
    ps.pose.position.z = pos[2]
    ps.pose.orientation.x = quaternion[0]
    ps.pose.orientation.y = quaternion[1]
    ps.pose.orientation.z = quaternion[2]
    ps.pose.orientation.w = quaternion[3]
    ps.header.frame_id = "map"
    return ps

def do_path_step(pose):
    point = Pose()
    go_to_stpt = rospy.ServiceProxy('stero/go_to_stpt', STPT)
    point.x = pose.pose.position.x
    point.y = pose.pose.position.y
    [x,y,z,w] = [pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w]
    [roll,pitch,yaw] = PyKDL.Rotation.Quaternion(x,y,z,w).GetRot()
    point.theta = yaw
    print("x: {0}, y: {1}, theta: {2}".format(point.x, point.y, point.theta))
    print("return code: {0}".format(go_to_stpt(point)))
    print("--------------")   
   
def create_goal(x,y):
    goal = PoseStamped()
    goal.header.frame_id="map"
    goal.pose.position.x = x
    goal.pose.position.y = y
    return goal
  
def plan_goal(stpt):
    print("stpt received")
    
    make_plan = rospy.ServiceProxy('/global_planner/planner/make_plan', GetPlan)
    go_to_stpt = rospy.ServiceProxy('stero/go_to_stpt', STPT)
    
    start = create_start()
    goal = create_goal(stpt.x, stpt.y)
    
    print("### START ###")
    print(start.pose.position)
    print("### GOAL ###")
    print(goal.pose.position)
    poses = make_plan(start, goal, 0.1).plan.poses
    
    print("planned path contains {0} points".format(len(poses)))
    n=0
    while n<len(poses):
        print("stpt {0}: ".format(n))
        do_path_step(poses[n])
        n = n + 30
    print("stpt {0}: ".format(len(poses)-1))
    do_path_step(poses[len(poses)-1])


if __name__ == "__main__":
    rospy.init_node('marcador_de_objetivo', anonymous=False)
  
    robot_vel_pub = rospy.Publisher('mux_vel_nav/cmd_vel', Twist, queue_size = 10)
    rospy.Subscriber('/stero/set_goal', Point, plan_goal)
    #rospy.Subscriber('/elektron/mobile_base_controller/odom', Odometry, odometry_callback)
    print("waiting for /global_planner/planner/make_plan service")
    rospy.wait_for_service('/global_planner/planner/make_plan')
    print("waiting for stero/go_to_stpt service")
    rospy.wait_for_service('stero/go_to_stpt')
    print("OK, let's go!")
    rospy.spin()  
   
      
  