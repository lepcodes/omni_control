# Omni_Control

The `Omni_Control` package is designed to facilitate interaction with the `Name of Robot`. It uses the `Name of Drivers` drivers to manage and control the robot’s motors. Additionally, it uses Arduino as an interface between the computer and the motors.

This package has been tested in `ROS Melodic` and `Noetic`.

## Installation

To run this package use

    mkdir -p omni_ros_ws/src
    git clone https://github.com/Luispre99/omni_control.git
    cd ..
    catkin build omni_control

## Development

You need to have this ros-packages

    sudo apt-get install ros-$ROS_DISTRO-rosserial-python
    sudo apt-get install ros-$ROS_DISTRO-rosserial-arduino
    sudo apt-get install ros-$ROS_DISTRO-rosserial

If you want to control the robot using joysticks you need to have the following packages installed

    sudo apt-get install ros-$ROS_DISTRO-joy
    sudo apt-get install ros-$ROS_DISTRO-teleop-twist-joy

The SaberTooth drivers for the motors can be found [here](https://www.dimensionengineering.com/info/arduino) in case you want to modify the robot behaviour.

## Running

This package is prepared to control the robot using a ps3 controller by running the following launch file:

    roslaunch omni_control joy_omni.launch

If you are using other controller perhaps you need to create or modify the [config](/config/ps3-holonomic.config.yaml) file and adjust the drivers to detect the gamepad as `/dev/input/jsX`.