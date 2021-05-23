# 2020R2 MAEG57550 Robotics Project 2 Bin Picking
======

## Team members
Wei CHEN, Jichun YANG, Zhi CHEN, Yihu LING

## Platfrom
+ Ubuntu 20.04 LTS
+ ros: noetic 
+ nvidia driver 460
+ cuda toolkit 10.0
+ cudnn 7.4

## Installation

### 1 Install nvidia driver and cuda
https://phoenixnap.com/kb/install-nvidia-drivers-ubuntu

### 2 Install ROS
https://github.com/rojas70/learning_ros_setup_scripts

### 3 Install dependencies
+ python3.6
https://blog.csdn.net/weixin_42856871/article/details/108352958
+ kdl
https://github.com/orocos/orocos_kinematics_dynamics

### 4 deep grasp
+ dex-net 4.0
https://github.com/PickNikRobotics/deep_grasp_demo
https://github.com/mfkenson/MAEG5755-2021-Team-PARK
+ simulation and robot control code
```bash
git clone https://github.com/robot-chenwei/moveit_task_constructor.git
git clone https://github.com/robot-chenwei/MAEG5755_Robotics_Project.git
```

## Test

### 1. Baxter simulation
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
