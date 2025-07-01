# Crack-Filling-Robot-ROS

## Work In Progress

## Introduction

I run the latest Ubuntu Noble 24.04 on my PC. For the computer on the robot you can either use a Raspberry Pi (RPi) or Jetson NANO

If youre using a RPi you can directly intall Ubuntu 24.4 live server on it using the RPi Imager and install the latest ROS2 Jazzy distro. Which will match your PC. 

If you're using a Jetson NANO, setting up the environment is a bit more complicated. The current highest version of Ubuntu supported is Ubuntu Focal 20.04. You can follow the below steps to setup up your system. 

## Setup Software

### PC 
To intall humble ROS2 distro, you can use Docker. To install docker please follow the this [link](https://docs.docker.com/engine/install/ubuntu/). 

You can install `ros2` by running the below commmand in your terminal
```console
$ bash bash create_ros_docker.bash
```
Ths script maps the `$PWD` to `/home/ros/` inside the container. So the you can make the ROS2 workspace persisant and accessed outside the container.<br>

To connect and shell into the container 
```console
$ bash connect_ros_docker.bash
```
### Jetson NANO

Updage Jetson NANO to Ubuntu 20.04

#### Method 1: Flash Image
Flash the provided Ubuntu 20.04 OS image
[Follow Link](https://github.com/Qengineering/Jetson-Nano-Ubuntu-20-image)

#### Mothod 2: Manual Upgrade 
Upgrade it manually by yourself, follow this link. 
[Follow Link](https://qengineering.eu/install-ubuntu-20.04-on-jetson-nano.html)

Install ROS2

#### Method 1: Docker

#### Method 2: Source Build
Ubuntu 20.04 is a Tier 3 OS for ROS2 humble 
We need to install ROS2 humble on Jetson NANO by building from the source.
[Follow Link](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html)