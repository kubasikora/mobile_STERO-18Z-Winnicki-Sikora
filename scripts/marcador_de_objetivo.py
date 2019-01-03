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

global map_pose

def tf_callback(pose):
    global map_pose
    
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
    
   
def create_goal(x,y):
    goal = PoseStamped()
    goal.header.frame_id="map"
    goal.pose.position.x = x
    goal.pose.position.y = y
    return goal
  
def plan_goal(stpt):
    rospy.wait_for_service('/global_planner/planner/make_plan')
    
    make_plan = rospy.ServiceProxy('/global_planner/planner/make_plan', GetPlan)
    
    start = create_start()
    goal = create_goal(stpt.x, stpt.y)
    
    print("### START ###")
    print(start)
    print("### GOAL ###")
    print(goal)
    plan = make_plan(start, goal, 0.0)
    
    print(plan)
 


if __name__ == "__main__":
    rospy.init_node('marcador_de_objetivo', anonymous=False)
  
    robot_vel_pub = rospy.Publisher('mux_vel_nav/cmd_vel', Twist, queue_size = 10)
    rospy.Subscriber('/stero/set_goal', Point, plan_goal)
    rospy.spin()  
   
      
  