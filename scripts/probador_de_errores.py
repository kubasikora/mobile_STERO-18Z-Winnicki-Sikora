#!/usr/bin/env python 

import rospy
import numpy as numpy
import PyKDL
import math
import sys

from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovariance, Pose2D
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry

__CW__ = 1
__CCW__ = -1

global pub
global x_robot, y_robot, theta_robot
global x_gazebo, y_gazebo, theta_gazebo
global x_laser, y_laser, theta_laser

class STPT:
    def __init__(self, x, y, theta, velocity, linear=True):
        self.x = x
        self.y = y
        self.theta = theta
        self.velocity = velocity
        self.linear = linear

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

def gazebo_callback(odom):
    global x_gazebo, y_gazebo, theta_gazebo
    [x,y,z,w] = [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w]
    [roll,pitch,theta_gazebo] = PyKDL.Rotation.Quaternion(x,y,z,w).GetRot()
    x_gazebo = odom.pose.pose.position.x
    y_gazebo = odom.pose.pose.position.y

def laser_callback(pose):
    global x_laser, y_laser, theta_laser
    x_laser = pose.x
    y_laser = pose.y
    theta_laser = pose.theta

def execute_setpoints(setpoints):
    robot_vel_pub = rospy.Publisher('mux_vel_nav/cmd_vel', Twist, queue_size = 10)
    robot_pos_pub = rospy.Publisher('/stero/position', Pose, queue_size = 10)
    robot_stpt_pub = rospy.Publisher('/stero/setpoint', Pose, queue_size = 10)
    robot_odom_err_pub = rospy.Publisher('/stero/errors/odom', Pose, queue_size = 10)
    robot_laser_err_pub = rospy.Publisher('/stero/errors/laser', Pose, queue_size = 10)
        
    rospy.Subscriber('/elektron/mobile_base_controller/odom', Odometry, odometry_callback)
    rospy.Subscriber('/gazebo_odom', PoseWithCovariance, gazebo_callback)
    rospy.Subscriber('/pose2D', Pose2D, laser_callback)
    
    for stpt in setpoints:
        stpt_x = stpt.x
        stpt_y = stpt.y
        stpt_theta = stpt.theta
        velocity = stpt.velocity
        linear = stpt.linear

        stpt_msg = pose_factory(stpt.x, stpt.y, stpt.theta)
        robot_stpt_pub.publish(stpt_msg)
        
        print("------------")
        print("Actual position: [({0}, {1}), {2}]".format(x_robot, y_robot, theta_robot))
        print("STPT: [({0}, {1}), {2}]".format(stpt.x, stpt.y, stpt.theta))
        print("Velocity: {0}".format(stpt.velocity))
        print("Moving...")

        if(stpt.linear == True):
            t = rospy.get_time()
            pos_err = 10
            prev_pos_err = pos_err
            move_msg = twist_factory(x = stpt.velocity)
            pos_err = math.sqrt((stpt.x-x_robot)**2 + (stpt.y-y_robot)**2)
            while (pos_err - 0.1 > 0):        
                robot_vel_pub.publish(move_msg)
                prev_pos_err = pos_err
                pos_err = math.sqrt((stpt.x-x_robot)**2 + (stpt.y-y_robot)**2 )
                robot_pos_pub.publish(pose_factory(x_robot, y_robot, theta_robot))
                robot_stpt_pub.publish(stpt_msg)
                robot_odom_err_pub.publish(pose_factory(x_gazebo - x_robot, y_gazebo - y_robot, theta_gazebo - theta_robot))
                robot_laser_err_pub.publish(pose_factory(x_gazebo - x_laser, y_gazebo - y_laser, theta_gazebo - theta_laser))
        else:
            t = rospy.get_time()
            move_msg = twist_factory(ang_z = velocity)
            robot_vel_pub.publish(move_msg)
            while 0.05 < math.fabs(stpt.theta-theta_robot):
                robot_vel_pub.publish(move_msg)
                robot_pos_pub.publish(pose_factory(x_robot, y_robot, theta_robot))
                robot_stpt_pub.publish(stpt_msg)
                
        stop_msg = twist_factory()
        robot_vel_pub.publish(stop_msg)
        print("Goal achieved")
        print("Actual position: [({0}, {1}), {2}]".format(x_robot, y_robot, theta_robot))
        robot_pos_pub.publish(pose_factory(x_robot, y_robot, theta_robot))

def do_line():
    global x_robot, y_robot, theta_robot, x_gazebo, y_gazebo, theta_gazebo, x_laser, y_laser, theta_laser
    print("Estoy haciendo una prueba de carretera")

    __linear_vel__ = 0.2
   
    robot_vel_pub = rospy.Publisher('mux_vel_nav/cmd_vel', Twist, queue_size = 10)
    robot_pos_pub = rospy.Publisher('/stero/position', Pose, queue_size = 10)
    robot_stpt_pub = rospy.Publisher('/stero/setpoint', Pose, queue_size = 10)
    robot_odom_err_pub = rospy.Publisher('/stero/errors/odom', Pose, queue_size = 10)
    robot_laser_err_pub = rospy.Publisher('/stero/errors/laser', Pose, queue_size = 10)
        
    rospy.Subscriber('/elektron/mobile_base_controller/odom', Odometry, odometry_callback)
    rospy.Subscriber('/pose2D', Pose2D, laser_callback)
    rospy.Subscriber('/gazebo_odom', Odometry, gazebo_callback)
    
    print("------------")
    print("Actual position: [({0}, {1}), {2}]".format(x_robot, y_robot, theta_robot))
    print("Velocity: {0}".format(__linear_vel__))
    print("Moving...")

    __sim_time__ = 6
    move_msg = twist_factory(x = __linear_vel__)
    t = rospy.get_time()
    while (rospy.get_time() - t < __sim_time__):
        robot_vel_pub.publish(move_msg) # make elektron move
        robot_pos_pub.publish(pose_factory(x_robot, y_robot, theta_robot)) # publish own position
        robot_odom_err_pub.publish(pose_factory(math.fabs(x_gazebo - x_robot), math.fabs(y_gazebo - y_robot), math.fabs(theta_gazebo - theta_robot))) #publish odom error
        robot_laser_err_pub.publish(pose_factory(math.fabs(x_gazebo - x_laser), math.fabs(y_gazebo - y_laser), math.fabs(theta_gazebo - theta_laser))) #publish laser error
    
    robot_odom_err_pub.publish(pose_factory(math.fabs(x_gazebo - x_robot), math.fabs(y_gazebo - y_robot), math.fabs(theta_gazebo - theta_robot))) #publish odom error
    robot_laser_err_pub.publish(pose_factory(math.fabs(x_gazebo - x_laser), math.fabs(y_gazebo - y_laser), math.fabs(theta_gazebo - theta_laser))) #publish laser error
    stop_msg = twist_factory()
    robot_vel_pub.publish(stop_msg)

    print("Finished moving ")
    print("Real position = [({0}, {1}), {2}]".format(x_gazebo, y_gazebo, theta_gazebo))
    print("Odom position = [({0}, {1}), {2}]".format(x_robot, y_robot, theta_robot))
    print("Laser position = [({0}, {1}), {2}]".format(x_laser, y_laser, theta_laser))
    print("------------")

def do_circle():
    global x_robot, y_robot, theta_robot, x_gazebo, y_gazebo, theta_gazebo, x_laser, y_laser, theta_laser
    print("Me das vueltas, bebe, Justo como un disco, bebe")
    
    __angular_vel__ = 0.3
    
    robot_vel_pub = rospy.Publisher('mux_vel_nav/cmd_vel', Twist, queue_size = 10)
    robot_pos_pub = rospy.Publisher('/stero/position', Pose, queue_size = 10)
    robot_stpt_pub = rospy.Publisher('/stero/setpoint', Pose, queue_size = 10)
    robot_odom_err_pub = rospy.Publisher('/stero/errors/odom', Pose, queue_size = 10)
    robot_laser_err_pub = rospy.Publisher('/stero/errors/laser', Pose, queue_size = 10)
        
    rospy.Subscriber('/elektron/mobile_base_controller/odom', Odometry, odometry_callback)
    rospy.Subscriber('/pose2D', Pose2D, laser_callback)
    rospy.Subscriber('/gazebo_odom', Odometry, gazebo_callback)
    
    print("------------")
    print("Actual position: [({0}, {1}), {2}]".format(x_robot, y_robot, theta_robot))
    print("Velocity: {0}".format(__angular_vel__))
    print("Moving...")

    __sim_time__ = 10
    move_msg = twist_factory(ang_z = __angular_vel__)
    robot_vel_pub.publish(move_msg) # make elektron move
    t = rospy.get_time()
    while (rospy.get_time() - t < __sim_time__):
        robot_vel_pub.publish(move_msg) # make elektron move
        robot_pos_pub.publish(pose_factory(x_robot, y_robot, theta_robot)) # publish own position
        robot_odom_err_pub.publish(pose_factory(math.fabs(x_gazebo - x_robot), math.fabs(y_gazebo - y_robot), math.fabs(theta_gazebo - theta_robot))) #publish odom error
        robot_laser_err_pub.publish(pose_factory(math.fabs(x_gazebo - x_laser), math.fabs(y_gazebo - y_laser), math.fabs(theta_gazebo - theta_laser))) #publish laser error
    
    robot_odom_err_pub.publish(pose_factory(math.fabs(x_gazebo - x_robot), math.fabs(y_gazebo - y_robot), math.fabs(theta_gazebo - theta_robot))) #publish odom error
    robot_laser_err_pub.publish(pose_factory(math.fabs(x_gazebo - x_laser), math.fabs(y_gazebo - y_laser), math.fabs(theta_gazebo - theta_laser))) #publish laser error
    stop_msg = twist_factory()
    robot_vel_pub.publish(stop_msg)

    print("Finished moving ")
    print("Real position = [({0}, {1}), {2}]".format(x_gazebo, y_gazebo, theta_gazebo))
    print("Odom position = [({0}, {1}), {2}]".format(x_robot, y_robot, theta_robot))
    print("Laser position = [({0}, {1}), {2}]".format(x_laser, y_laser, theta_laser))
    print("------------")


def do_square():
    global x_robot, y_robot, theta_robot, x_gazebo, y_gazebo, theta_gazebo, x_laser, y_laser, theta_laser
    print("La universidad no es solo un juego de minecraft.")

    __linear_vel__ = 0.1
    __angular_vel__ = 0.2

    robot_vel_pub = rospy.Publisher('mux_vel_nav/cmd_vel', Twist, queue_size = 1)
    robot_pos_pub = rospy.Publisher('/stero/position', Pose, queue_size = 10)
    robot_stpt_pub = rospy.Publisher('/stero/setpoint', Pose, queue_size = 10)
    robot_odom_err_pub = rospy.Publisher('/stero/errors/odom', Pose, queue_size = 10)
    robot_laser_err_pub = rospy.Publisher('/stero/errors/laser', Pose, queue_size = 10)
        
    rospy.Subscriber('/elektron/mobile_base_controller/odom', Odometry, odometry_callback)
    rospy.Subscriber('/pose2D', Pose2D, laser_callback)
    rospy.Subscriber('/gazebo_odom', Odometry, gazebo_callback)

    for i in range(4):
        #spin about 90 degrees
        print("------------")
        print("Actual position: [({0}, {1}), {2}]".format(x_robot, y_robot, theta_robot))
        print("Velocity: {0}".format(__angular_vel__))
        print("Spinning...")

        __sim_time__ = 0.5*math.pi / __angular_vel__
        print("Spin time: {0}".format(__sim_time__))
        move_msg = twist_factory(ang_z = __angular_vel__)
        t = rospy.get_time()
        robot_vel_pub.publish(move_msg) # make elektron move
        while (rospy.get_time() - t < __sim_time__):
            rospy.sleep(0.05)
            robot_vel_pub.publish(move_msg) # make elektron move
            robot_pos_pub.publish(pose_factory(x_robot, y_robot, theta_robot)) # publish own position
            robot_odom_err_pub.publish(pose_factory(math.fabs(x_gazebo - x_robot), math.fabs(y_gazebo - y_robot), math.fabs(theta_gazebo - theta_robot))) #publish odom error
            robot_laser_err_pub.publish(pose_factory(math.fabs(x_gazebo - x_laser), math.fabs(y_gazebo - y_laser), math.fabs(theta_gazebo - theta_laser))) #publish laser error
    
        robot_odom_err_pub.publish(pose_factory(math.fabs(x_gazebo - x_robot), math.fabs(y_gazebo - y_robot), math.fabs(theta_gazebo - theta_robot))) #publish odom error
        robot_laser_err_pub.publish(pose_factory(math.fabs(x_gazebo - x_laser), math.fabs(y_gazebo - y_laser), math.fabs(theta_gazebo - theta_laser))) #publish laser error
        stop_msg = twist_factory()
        robot_vel_pub.publish(stop_msg)
        print("Finished spinning")

        print("Real position = [({0}, {1}), {2}]".format(x_gazebo, y_gazebo, theta_gazebo))
        print("Odom position = [({0}, {1}), {2}]".format(x_robot, y_robot, theta_robot))
        print("Laser position = [({0}, {1}), {2}]".format(x_laser, y_laser, theta_laser))
        print("-----")

        #move forward
        print("Moving forward...")
        __sim_time__ = 2  / __linear_vel__
        print("Move time: {0}".format(__sim_time__))
        move_msg = twist_factory(x = __linear_vel__)
        t = rospy.get_time()
        robot_vel_pub.publish(move_msg) # make elektron move
        while (rospy.get_time() - t < __sim_time__):
            rospy.sleep(0.05)
            robot_vel_pub.publish(move_msg) # make elektron move
            robot_pos_pub.publish(pose_factory(x_robot, y_robot, theta_robot)) # publish own position
            robot_odom_err_pub.publish(pose_factory(math.fabs(x_gazebo - x_robot), math.fabs(y_gazebo - y_robot), math.fabs(theta_gazebo - theta_robot))) #publish odom error
            robot_laser_err_pub.publish(pose_factory(math.fabs(x_gazebo - x_laser), math.fabs(y_gazebo - y_laser), math.fabs(theta_gazebo - theta_laser))) #publish laser error
    
        robot_odom_err_pub.publish(pose_factory(math.fabs(x_gazebo - x_robot), math.fabs(y_gazebo - y_robot), math.fabs(theta_gazebo - theta_robot))) #publish odom error
        robot_laser_err_pub.publish(pose_factory(math.fabs(x_gazebo - x_laser), math.fabs(y_gazebo - y_laser), math.fabs(theta_gazebo - theta_laser))) #publish laser error
        stop_msg = twist_factory()
        robot_vel_pub.publish(stop_msg)

        print("Finished part {0}".format(i+1))
        print("Real position = [({0}, {1}), {2}]".format(x_gazebo, y_gazebo, theta_gazebo))
        print("Odom position = [({0}, {1}), {2}]".format(x_robot, y_robot, theta_robot))
        print("Laser position = [({0}, {1}), {2}]".format(x_laser, y_laser, theta_laser))
        print("------------")


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
    

    x_robot = 0
    y_robot = 0
    theta_robot = 0
    x_gazebo = 0
    y_gazebo = 0
    theta_gazebo = 0
    x_laser = 0
    y_laser = 0
    theta_laser = 0
    task()
