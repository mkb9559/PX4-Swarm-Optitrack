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

Created rigid body should be named "UAV1", "UAV2" ...  corresponding to IDs of UAVs. Or topics in launch file need modify.

## 1.3 Parameter setting for PX4
- Set **EKF2_AID_MASK** to **24** (Only select **vision position fusion** and **vision yaw fusion**) (The latest QGC may fail to modify. Please use the old version of QGC)
- Set **EKF2_HGT_MODE** to **VISION**
- Make sure your mavlink port has been configured.

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
- Modify optitrack IP and VRPN toptic names in launchs:
UAV1_single.launch
UAV2_single.launch
...
- Modify UAV IPs in swarm_cmd.cpp
```
 new std::thread(&UdpServer,"192.168.1.11",12001,1);
 new std::thread(&UdpServer,"192.168.1.12",12001,2);
 ...
```
## 3.2 Center Ubuntu
```
source devel/setup.bash
roslaunch px4_zxz swarm.launch
```
Input number for command.
## 3.3 Onboard Ubuntu
Launch corresponded launch, such as for UAV2:
```
source devel/setup.bash
roslaunch px4_zxz UAV2_single.launch
```
SSH remote login from center Ubuntu may help you.
## 3.4 Input cmd in center Ubuntu
When the program used starts, please wait to ensure that the mavros is successfully connected to the PX4.

- input  9 for takeoff
- input 10 for Low speed in aim +x 
- input 20 for Low speed in aim -x 
- input 30 for Low speed in aim +y 
- input 40 for Low speed in aim -y 
- input 50 for Low speed in aim +z 
- input 60 for Low speed in aim -z 
- input  4 for land
- input 99 for end



# 4 Demo

https://www.bilibili.com/video/BV1xR4y1t7A2/





