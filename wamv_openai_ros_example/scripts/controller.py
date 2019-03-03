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

def calculate_velcoties(msg):
    rospy.loginfo(msg)



if __name__ == '__main__':

    rospy.init_node('wamv_nav_twosets_buoys', anonymous=True, log_level=rospy.WARN)

    # Create the Gym environment
    env = gym.make('WamvNavTwoSetsBuoys-v0')
    rospy.loginfo("Gym environment done")


    # GOAL VALUES NEED TO BE FOUND SOMEHOW (fomr environment?)
    goal_x = rospy.get_param("~goal_x", 10)
    goal_y = rospy.get_param("~goal_y", 10)


    # wait for messages where relavent
    rospy.wait_for_message("/wamv/odom", Odometry, timeout=1.0)

    # ROS subscribers
    rospy.Subscriber("/wamv/odom", Odometry, calculate_velcoties)

    # ROS publishers
    cmd_drive = rospy.Publisher('/cmd_drive', UsvDrive, queue_size=0)

    # some relavent equations for differential kinematics of a diff drive
