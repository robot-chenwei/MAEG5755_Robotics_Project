# 2020R2 MAEG57550 Robotics Project 2 Bin Picking
=====

## Team members
Wei CHEN, Jichun YANG, Zhi CHEN, Yihu LING

## Platfrom
+ Ubuntu 18.04 LTS
+ rosdistro: melodic 1.14.10
+ nvidia driver 460
+ cuda toolkit 10

## Installation


## Test






## Problems
### Install nvidia driver and cuda fail
remove the nvidia driver
```bash
sudo apt purge nvidia-*
sudo apt autoremove
```
install the nvidia driver and cuda-toolkit
```bash
sudo add-apt-repository ppa:graphics-drivers/ppa
sudo apt update
sudo apt-get install nvidia-driver-460 nvidia-cuda-toolkit
```
