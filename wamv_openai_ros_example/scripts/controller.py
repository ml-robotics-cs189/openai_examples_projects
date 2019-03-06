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
# from gym import spaces
# from openai_ros.robot_envs import wamv_env
# from gym.envs.registration import register

# messages
from robotx_gazebo.msg import UsvDrive
from nav_msgs.msg import Odometry

def calculate_velcoties(msg):
    print("here4")
    rospy.loginfo(msg)



if __name__ == '__main__':

    rospy.init_node('wamv_nav_twosets_buoys', anonymous=True, log_level=rospy.WARN)

    # # Create the Gym environment
    # env = gym.make('WamvNavTwoSetsBuoys-v0')
    # rospy.loginfo("Gym environment done")


    # GOAL VALUES NEED TO BE FOUND SOMEHOW (fomr environment?)
    goal_x = rospy.get_param("~goal_x", 10)
    goal_y = rospy.get_param("~goal_y", 10)

    #     # Get Desired Point to Get
    # self.desired_point = Point()
    # self.desired_point.x = rospy.get_param("/wamv/desired_point/x")
    # self.desired_point.y = rospy.get_param("/wamv/desired_point/y")
    # self.desired_point.z = rospy.get_param("/wamv/desired_point/z")
    # self.desired_point_epsilon = rospy.get_param("/wamv/desired_point_epsilon")


    print("here1")
    # wait for messages where relavent
    rospy.wait_for_message("/wamv/odom", Odometry, timeout=5.0)

    print("here2")

    # ROS subscribers
    rospy.Subscriber("/wamv/odom", Odometry, calculate_velcoties)


    print("here3")

    # ROS publishers
    cmd_drive = rospy.Publisher('/cmd_drive', UsvDrive, queue_size=1)

    # some relavent equations for differential kinematics of a diff drive
