# 2020R2 MAEG57550 Robotics Project 2 Bin Picking
------

## Overview
This is the MAEG57550 Robotics Project 2 Bin Picking.
The camera get the depth image and sent to the Dex-Net 4.0 to get the grasp candidates.
Then combine with inverse kinematics, the most approrpiate candidate is selected as the grasp pose.
Finally, run the robot by using inverse kinematics to achieve the pick and place task.

## QuickStart

### 1. Baxter simulation
```bash
git clone https://github.com/robot-chenwei/baxter_pykdl.git
git clone https://github.com/robot-chenwei/MAEG5755_Robotics_Project.git
```
+ terminal 1
```bash
roslaunch baxter_gazebo baxter_camera.launch
```
+ terminal 2
```bash
roslaunch moveit_task_constructor_dexnet dexnet_baxter_simulation.launch load_image:=false
```

### 2. Baxter robot
```bash
./baxter.sh
roslaunch moveit_task_constructor_dexnet dexnet_baxter.launch load_image:=false
```

## Structure
+ moveit support packages
+ baxter related packages
+ baxter_pykdl: for solve the inverse kinematics of Baxter based on KDL
+ MAEG5755_Robotics_Project: dex-net grasp task

## Video
[![](https://img.youtube.com/vi/rpCKKMZ19CY/0.jpg)](https://www.youtube.com/watch?v=rpCKKMZ19CY)

## FAQ

### Platfrom
+ Ubuntu 20.04 LTS
+ ros: noetic 
+ nvidia driver 460
+ cuda toolkit 10.0
+ cudnn 7.4

### Install dependency
#### 1 Install nvidia driver and cuda
https://phoenixnap.com/kb/install-nvidia-drivers-ubuntu

#### 2 Install ROS
https://github.com/rojas70/learning_ros_setup_scripts

#### 3 Install dependencies
+ python3.6
https://blog.csdn.net/weixin_42856871/article/details/108352958
+ kdl
https://github.com/orocos/orocos_kinematics_dynamics

#### 4 deep grasp
##### 4.1 Step setup
- dex-net 4.0
https://github.com/PickNikRobotics/deep_grasp_demo
- moveit
https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html
- baxter
https://sdk.rethinkrobotics.com/wiki/Workstation_Setup

##### 4.2 Fully setup
https://github.com/mfkenson/MAEG5755-2021-Team-PARK

