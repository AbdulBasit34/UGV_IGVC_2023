# Unmanned Ground Vehicle - DTU
<img src="Images/Slack%20Logo.png" width="250" height="250" align="right"/>

[![Build Status](https://travis-ci.org/joemccann/dillinger.svg?branch=master)](https://travis-ci.org/joemccann/dillinger) [![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause) 

>Development of this Stack is a stepping stone for the \
>IGVC AutoNav Challange, Oakland University, Michigan.
>
>This is not the Complete solution for the challange but rather an attempt \
>to understand the various primitive techniques to make a self driving vehicle \
>and to learn how the Robot Operating System Environment works.

## Key Features !!
Includes Custom Scripts for :
  - Visualizing 2d postion Covariance of the Robot as an ellipse in RVIZ.
  - Converting Lanes from Camera Image to fake_laser Message to visualize in RVIZ,\
     which can also be used in gmapping, MoveBase and Costmap2d packages.
  - Change in heading (yaw) calculation, using basics of group theory (to be exact modular arithmetic), \
     for turning exact angles in degrees. 
  - Contains a pre-build IGVC world for testing purposes.

## Initial Setup
Keep in mind that this Package is developed on **ROS-Melodic**.

1. Navigate to your src folder 
```sh 
cd ~/catkin_ws/src 
```
2. Clone this package into src
```sh
git clone https://github.com/harsh-kaushal/UGV-DTU_ROS_Stack.git
```
3. Rename the "UGV-DTU_ROS_Stack" folder to "ugv_bot"
```sh
mv UGV-DTU_ROS_Stack ugv_bot
```
4. Navigate back to catkin workspace
```sh
cd ~/catkin_ws
```
5. Build the package using
  ```sh
  catkin_make
  ```
  or alternatively you could install catkin tools and then use catkin build 
  ```sh
  sudo apt-get install python-catkin-tools
  catkin build ugv_bot 
  ```
6. Give paths to Gazebo for Custom world models
```sh
echo "export GAZEBO_MODEL_PATH=~/catkin_ws/src/ugv_bot/models" >> ~/catkin_ws/devel/setup.bash
```
7. Now you are all set to run the launch file.
```sh
source ~/catkin_ws/devel/setup.bash
roslaunch ugv_bot ugvbot_world.launch
```

## Notes
- The setup assumes that you have catkin_ws folder on your home directory if not, do changes accordingly.

- If the gazebo shows a black screen after launching, do
  ```sh
  echo $GAZEBO_MODEL_PATH 
  ```
  and make sure that the path is correct.

  Further, it may take some time in launch for the first time, dont panic enjoy the dark world for few minutes,
  and then  **ctrl+c** in the terminal, and then try relaunching.









