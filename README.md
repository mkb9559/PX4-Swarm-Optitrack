# PX4-Swarm-Optitrack
A simple formation framework for Pixhawk(PX4) rotorcrafts with optitrack positioning.

The software is developed under **Ubuntu 18.04** and **ROS melodic**

# Primity

## Network Configuration
All of devices are with static IP in a same WLAN.
Here is our spercific configuration: 
- Optitrack PC:  192.168.1.200
- Center Ubuntu: 102.168.1.41
- Onboard Ubuntu of UAV1: 192.168.1.11
- Onboard Ubuntu of UAV2: 192.168.1.12
- ...

## Rigid Body Creation in Optitrack
Set the z+ for the up direction of optitrack coordinate.

Lay the head direction of UAV alone the x direction of optitrack coordinate.

## Parameter Setting for PX4
- Set **EKF2_AID_MASK** to **24** (The latest QGC may fail to modify. Please use the old version of QGC)
- Set **EKF2_HGT_MODE** to **VISION**

# Dependence

