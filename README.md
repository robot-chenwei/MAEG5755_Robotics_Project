# 2020R2 MAEG57550 Robotics Project 2 Bin Picking
======

## Team members
Wei CHEN, Jichun YANG, Zhi CHEN, Yihu LING

## Platfrom
+ Ubuntu 18.04 LTS
+ rosdistro: melodic 1.14.10
+ nvidia driver 460
+ cuda toolkit 10

## Installation

### 1 Install nvidia driver and cuda
https://phoenixnap.com/kb/install-nvidia-drivers-ubuntu
+ 2 Install nvidia driver and cuda
https://phoenixnap.com/kb/install-nvidia-drivers-ubuntu

### 2 Install ROS
https://github.com/rojas70/learning_ros_setup_scripts

### 3 Install dependencies
+ python3.6
https://blog.csdn.net/weixin_42856871/article/details/108352958
+ kdl
https://github.com/orocos/orocos_kinematics_dynamics

### 4 deep grasp
```bash
git clone --recursive https://github.com/mfkenson/MAEG5755-2021-Team-PARK.git team-park
git clone https://github.com/robot-chenwei/moveit_task_constructor.git
git clone https://github.com/PickNikRobotics/deep_grasp_demo.git
```

## Test

### 1. Franka panda
```bash
roslaunch deep_grasp_task panda_world.launch 
```
```bash
roslaunch moveit_task_constructor_dexnet dexnet_demo.launch 
```

### 2. Baxter
```bash
roslaunch baxter_gazebo baxter_world.launch
```
```bash
roslaunch moveit_task_constructor_dexnet dexnet_baxter.launch 
```
