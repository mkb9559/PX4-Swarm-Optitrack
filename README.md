# PX4-Swarm-Optitrack
A simple formation framework for Pixhawk(PX4) rotorcrafts with optitrack positioning.

The software is developed under **Ubuntu 18.04** and **ROS melodic**

The communication is achieved via UDP.

# 1 Primity

## 1.1 Network configuration
All of devices are with static IP in a same WLAN.
Here is our spercific configuration: 
- Optitrack PC:  192.168.1.200
- Center Ubuntu: 102.168.1.41
- Onboard Ubuntu of UAV1: 192.168.1.11
- Onboard Ubuntu of UAV2: 192.168.1.12
- ...

## 1.2 Rigid body creation in optitrack
Set the z+ for the up direction of optitrack coordinate.

Lay the head direction of UAV alone the x direction of optitrack coordinate.

Created rigid body should be named "UAV1", "UAV2" ...  corresponding to IDs of UAVs. Or topics in source code need modify.

## 1.3 Parameter setting for PX4
- Set **EKF2_AID_MASK** to **24** (The latest QGC may fail to modify. Please use the old version of QGC)
- Set **EKF2_HGT_MODE** to **VISION**

For detial, you may refer http://docs.px4.io/master/zh/computer_vision/motion_capture.html


# 2 Dependence
For Center and all of Onboard Ubuntu.

- ROS melodic: http://wiki.ros.org/melodic/Installation/Ubuntu
- Install VRPN
```
sudo apt-get install ros-melodic-vrpn
```
- Install Mavros
```
sudo apt-get install ros-melodic-mavros ros-melodic-mavros-extras
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh   
```
For detial, you may refer to https://docs.px4.io/master/en/ros/mavros_installation.html

# 3 Quick start
## 3.1 Transplant to your spercific system
- Modify UAV IPs in swarm.cpp
```

```
Change to your Onboard Ubuntu IPs and UAV ID.
- Distinguishment of different UAVs.
In 
```

```
Change to your Onboard Ubuntu IPs and UAV ID.
## 3.2 Center Ubuntu
```
./swarm.sh
```
Input number for command.
## 3.3 Onboard Ubuntu
```
./single.sh
```

# 4 Demo







