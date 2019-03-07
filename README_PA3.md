# Programming Assignment 3

Improved Q-learning paramters in code provided with OpenAI examples. Implemented double Q-learning based on the regular Q-learning code. Implemented a differential drive for the robot.

## Getting Started
This package requires Ubuntu 16.04 or later and Gazebo.

First set up the package in your environment. This can be done by cloning the repository into a catkin workspace. Execute the command:

`https://github.com/ml-robotics-cs189/openai_examples_projects.git`

Then run:

`>> catkin_make`

`>> roslaunch robotx_gazebo sandisland.launch`

To run each python file, type:

`>> roslaunch wamv_openai_ros_example {launch file name}.launch`

## Running the tests

The commands for each file are as follows:

Q-learning:

`>>roslaunch wamv_openai_ros_example start_training.launch`

Double Q-Learning:

`>>roslaunch wamv_openai_ros_example start_training_qq.launch`

Differential Drive:

`>>roslaunch wamv_openai_ros_example controller.launch`
