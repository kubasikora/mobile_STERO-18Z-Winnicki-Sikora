#!/usr/bin/env python 

import rospy
import numpy as np
import PyKDL
import math
import tf
import sys

from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovariance, Pose2D, Point
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetPlan
from stero_mobile_init.srv import STPT

global map_pose
global x_robot, y_robot, theta_robot
global srv_name

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
    global srv_name
    point = Pose()
    go_to_stpt = rospy.ServiceProxy(srv_name, STPT)
    point.x = pose.pose.position.x
    point.y = pose.pose.position.y
    [x,y,z,w] = [pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w]
    [roll,pitch,yaw] = PyKDL.Rotation.Quaternion(x,y,z,w).GetRot()
    point.theta = yaw
    print("x: {0}, y: {1}, theta: {2}".format(point.x, point.y, point.theta))
    result = go_to_stpt(point)
    print("return code: {0}".format(result))
    print("--------------")   
    return result
   
def create_goal(x,y, theta):
    goal = PoseStamped()
    goal.header.frame_id="map"
    goal.pose.position.x = x
    goal.pose.position.y = y
    quaternion = tf.transformations.quaternion_from_euler(0,0,theta)
    goal.pose.orientation.x = quaternion[0]
    goal.pose.orientation.y = quaternion[1]
    goal.pose.orientation.z = quaternion[2]
    goal.pose.orientation.w = quaternion[3]
    
    return goal
  
def plan_goal(stpt):
    __path_step__ = 30
    print("stpt received")
    
    make_plan = rospy.ServiceProxy('/global_planner/planner/make_plan', GetPlan)
    go_to_stpt = rospy.ServiceProxy('stero/go_to_stpt', STPT)
    
    start = create_start()
    goal = create_goal(stpt.x, stpt.y, stpt.theta)
    
    print("### START ###")
    print(start.pose.position)
    print("### GOAL ###")
    print(goal.pose)
    poses = make_plan(start, goal, 0.1).plan.poses
    lenPoses = len(poses)
    if lenPoses==0:
        print( "Global planner did not found path. return -1")
        return -1
    
    print("planned path contains {0} points".format(lenPoses))
    n=__path_step__
    while n<lenPoses:
        print("stpt {0}: ".format(n))
        result = do_path_step(poses[n])
        n = n + 40
    print("stpt {0}: ".format(lenPoses-1))
    do_path_step(poses[lenPoses-1])
    print("### FINISH ###")

if __name__ == "__main__":
    rospy.init_node('marcador_de_objetivo', anonymous=False)
  
    robot_vel_pub = rospy.Publisher('mux_vel_nav/cmd_vel', Twist, queue_size = 10)
    rospy.Subscriber('stero/set_goal', Pose, plan_goal)
    #rospy.Subscriber('/elektron/mobile_base_controller/odom', Odometry, odometry_callback)
    print("waiting for /global_planner/planner/make_plan service")
    rospy.wait_for_service('/global_planner/planner/make_plan')
    srv_name = 'stero/go_to_stpt'
    if len(sys.argv) > 1 and sys.argv[1]=="local_planner_active":
        srv_name = 'stero/plan_local_stpt'
    print("waiting for {0} service".format(srv_name))
    rospy.wait_for_service(srv_name)

    print("OK, let's go!")
    rospy.spin()  
   
      
  