# PX4-Swarm-Optitrack
A simple formation framework for Pixhawk(PX4) rotorcrafts with optitrack positioning.

The software is developed under **Ubuntu 18.04** and **ROS melodic**

The communication is achieved via UDP.

# 1 Primity

## 1.1 Network Configuration
All of devices are with static IP in a same WLAN.
Here is our spercific configuration: 
- Optitrack PC:  192.168.1.200
- Center Ubuntu: 102.168.1.41
- Onboard Ubuntu of UAV1: 192.168.1.11
- Onboard Ubuntu of UAV2: 192.168.1.12
- ...

## 1.2 Rigid Body Creation in Optitrack
Set the z+ for the up direction of optitrack coordinate.

Lay the head direction of UAV alone the x direction of optitrack coordinate.

## 1.3 Parameter Setting for PX4
- Set **EKF2_AID_MASK** to **24** (The latest QGC may fail to modify. Please use the old version of QGC)
- Set **EKF2_HGT_MODE** to **VISION**

For detial, refer http://docs.px4.io/master/zh/computer_vision/motion_capture.html


# 2 Dependence
For Center and all of Onboard Ubuntu.

- ROS melodic: http://wiki.ros.org/melodic/Installation/Ubuntu
- Install VRPN
```
sudo apt-get install ros-melodic-vrpn
```

# 3 Quick Start
## 3.1 Transplant to your spercific system

## 3.2 Center Ubuntu
```
./swarm.sh
```
Input number for command.
## 3.3 Onboard Ubuntu
```
./single.sh
```







