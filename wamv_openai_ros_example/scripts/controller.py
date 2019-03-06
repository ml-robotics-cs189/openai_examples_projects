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
from openai_ros import robot_gazebo_env

# messages
from robotx_gazebo.msg import UsvDrive
from nav_msgs.msg import Odometry


class Controller:

    def __init__(self):
        #copied structure from pa2
        print("Controller")

        #create the gym environment
        #rospy.init_node('wamv_nav_twosets_buoys', anonymous=True, log_level=rospy.WARN)

        # env = gym.make('WamvNavTwoSetsBuoys-v0')
        # rospy.loginfo("Gym environment done")

        # Set the logging system
        # rospack = rospkg.RosPack()
        # pkg_path = rospack.get_path('wamv_openai_ros_example')
        # outdir = pkg_path + '/training_results'
        # env = wrappers.Monitor(env, outdir, force=True)
        # rospy.loginfo("Monitor Wrapper started")


        self.usv_data = None
        self.odom_data = None
        self.current_pos = None
        
        self.check_odom_ready()

        # ROS subscribers
        self.odom = rospy.Subscriber('/wamv/odom', Odometry, self.odom_callback)

        # ROS publishers
        self.cmd_drive = rospy.Publisher('/cmd_drive', UsvDrive, queue_size=1)

    def odom_callback(self, odom):
        print("odom callback")
        #self.odom_data = data
        #self.current_pos = odom.pose.pose.position

        #print("Current position: ". self.current_pos)
        

    def check_odom_ready(self):
        self.odom = None
        rospy.loginfo("Waiting for /wamv/odom")
        while self.odom is None and not rospy.is_shutdown():
            try:
                self.odom = rospy.wait_for_message("/wamv/odom", Odometry, timeout=1.0)
                rospy.loginfo("/wamv/odom READY")

            except:
                rospy.logerr("Current /wamv/odom not ready yet, retrying for getting odom")
        return self.odom


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










# import gym
# import numpy
# import time
# from gym import wrappers
# import rospy
# import rospkg
# import math

# # import our training environment
# from openai_ros.task_envs.wamv import wamv_nav_twosets_buoys
# # from gym import spaces
# # from openai_ros.robot_envs import wamv_env
# # from gym.envs.registration import register

# # messages
# from robotx_gazebo.msg import UsvDrive
# from nav_msgs.msg import Odometry

# def calculate_velcoties(msg):
#     print("here4")
#     rospy.loginfo(msg)



# def check_odom_ready():
#     odom = None
#     rospy.loginfo("Waiting for /wamv/odom to be READY...")
#     while odom is None and not rospy.is_shutdown():
#         try:
#             odom = rospy.wait_for_message("/wamv/odom", Odometry, timeout=1.0)
#             rospy.loginfo("Current /wamv/odom READY=>")

#         except:
#             rospy.logerr("Current /wamv/odom not ready yet, retrying for getting odom")
#     return odom


# if __name__ == '__main__':

#     rospy.init_node('wamv_nav_twosets_buoys', anonymous=True, log_level=rospy.WARN)

#     # # Create the Gym environment
#     # env = gym.make('WamvNavTwoSetsBuoys-v0')
#     # rospy.loginfo("Gym environment done")


#     # GOAL VALUES NEED TO BE FOUND SOMEHOW (fomr environment?)
#     # goal_x = rospy.get_param("~goal_x", 10)
#     # goal_y = rospy.get_param("~goal_y", 10)

#     #     # Get Desired Point to Get
#     # self.desired_point = Point()
#     # self.desired_point.x = rospy.get_param("/wamv/desired_point/x")
#     # self.desired_point.y = rospy.get_param("/wamv/desired_point/y")
#     # self.desired_point.z = rospy.get_param("/wamv/desired_point/z")
#     # self.desired_point_epsilon = rospy.get_param("/wamv/desired_point_epsilon")


        
#     check_odom_ready()
#     # We Start all the ROS related Subscribers and publishers
    
#     # print("here1")
#     # # wait for messages where relavent
#     # rospy.wait_for_message("/wamv/odom", Odometry, timeout=5.0)

#     # print("here2")

#     # ROS subscribers
#     rospy.Subscriber("/wamv/odom", Odometry, calculate_velcoties)


#     # print("here3")

#     # # ROS publishers
#     # cmd_drive = rospy.Publisher('/cmd_drive', UsvDrive, queue_size=1)

#     # # some relavent equations for differential kinematics of a diff drive

