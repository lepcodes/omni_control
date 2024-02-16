# Omni_Control

## Installation

To run this package use

    mkdir -p omni_ros_ws/src
    git clone https://github.com/Luispre99/omni_control.git
    cd ..
    catkin_make

## Development

To modify the Arduino code or if you change the structure of the custom message, you have to install Arduino and the rosserial library

Besides, you need to have this ros-packages

    sudo apt-get install ros-melodic-rosserial-python
    sudo apt-get install ros-melodic-rosserial-arduino
    sudo apt-get install ros-melodic-rosserial

Then run
    
    cd ~/omni_ros_ws/src/omni_control
    rosrun rosserial_client make_libraries ~/Arduino/libraries omni_control
    sudo cp -R ~/Arduino/libraries/ros_lib/omni_control ~/Arduino/libraries/Rosserial_Arduino_Library/src

To generate and move the header files from custom messages in this case OmniVel.h

Finally, install the SaberTooth libraries from [here]()