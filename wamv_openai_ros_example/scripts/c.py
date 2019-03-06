#!/usr/bin/env python

import gym
import numpy
import time
from gym import wrappers
import rospy
import rospkg
import math

# import our training environment
from openai_ros.task_envs.wamv import wamv_nav_twosets_buoys

# messages
from robotx_gazebo.msg import UsvDrive
from nav_msgs.msg import Odometry


class Controller:

    def __init__(self):
	   #copied structure from pa2
        print("Controller")

        #create the gym environment
        #rospy.init_node('wamv_nav_twosets_buoys', anonymous=True, log_level=rospy.WARN)

        #env = gym.make('WamvNavTwoSetsBuoys-v0')
        #rospy.loginfo("Gym environment done")

        # Set the logging system
        # rospack = rospkg.RosPack()
        # pkg_path = rospack.get_path('wamv_openai_ros_example')
        # outdir = pkg_path + '/training_results'
        # env = wrappers.Monitor(env, outdir, force=True)
        # rospy.loginfo("Monitor Wrapper started")


        self.usv_data = None
        self.odom_data = None
        self.current_pos = None

        

        print("Subscribers")

	   #subscribers
        self.usv = rospy.Subscriber('/cmd_drive', UsvDrive, self.drive_callback)
        self.odom = rospy.Subscriber('/wamv/odom', Odometry, self.odom_callback)




    def drive_callback(self):
        print("In callback")

        self.usv_data = usv.data

        print("Data: ", self.usv_data)

    def odom_callback(self, odom):
        pass
        #print("odom callback")
        #self.odom_data = data
        #self.current_pos = odom.pose.pose.position

        #print("Current position: ". self.current_pos)
		

# def calculate_velcoties(msg):
#     rospy.loginfo(msg)



if __name__ == '__main__':

   # rospy.init_node('diff_drive', anonymous = True)

    rospy.init_node("diff_drive")

    r = rospy.Rate(10)



    drive = Controller()

    while not rospy.is_shutdown():
        rospy.spin



    # # Create the Gym environment
    # env = gym.make('WamvNavTwoSetsBuoys-v0')
    # rospy.loginfo("Gym environment done")


    # # GOAL VALUES NEED TO BE FOUND SOMEHOW (fomr environment?)
    # goal_x = rospy.get_param("~goal_x", 10)
    # goal_y = rospy.get_param("~goal_y", 10)


    # # wait for messages where relavent
    # rospy.wait_for_message("/wamv/odom", Odometry, timeout=1.0)

    # # ROS subscribers
    # rospy.Subscriber("/wamv/odom", Odometry, calculate_velcoties)

    # # ROS publishers
    # cmd_drive = rospy.Publisher('/cmd_drive', UsvDrive, queue_size=0)

    # some relavent equations for differential kinematics of a diff drive