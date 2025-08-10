#!/usr/bin/env bash
# ---- PS4 controller workspace ------------------------------------------------

# Create and populate your ROS 2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Core diagnostics & joystick drivers (ROS 2 branch)
if [ ! -d diagnostics ]; then
  git clone https://github.com/ros/diagnostics.git -b ros2
fi
if [ ! -d joystick_drivers ]; then
  git clone https://github.com/ros-drivers/joystick_drivers.git -b ros2
fi

# PS4 interface — default branch should work on Humble
if [ ! -d PlayStation-JoyInterface-ROS2 ]; then
  git clone https://github.com/HarvestX/PlayStation-JoyInterface-ROS2.git
fi

# Install Bluetooth & gamepad support libs
sudo apt install -y libspnav-dev libbluetooth-dev libcwiid-dev

# Before building the workspace, you need to resolve the package dependencies. 
# You may have all the dependencies already, but best practice is to check for dependencies every 
# time you clone. You wouldn’t want a build to fail after a long wait only to realize that you have missing dependencies.
sudo rosdep init
rosdep update
rosdep install -i --from-path . --rosdistro humble -y
# I now should says: #All required rosdeps installed successfully

# Build 
colcon build --symlink-install

# Sourcing ros2 workspace (ros2ws)
source install/setup.bash

# Check Installation
ros2 pkg list

## Other way for pairing the Controller
################# Make a Pair and Connect the PS4 Controller for the first time
# Wireless (Bluetooth) Connection
# Make sure Bluetooth is enabled on your Jetson:

# Start (Bluetooth) Service
sudo systemctl start bluetooth

# Bluetooth configuration over Terminal:
bluetoothctl

# Commands to write in bluetoothctl 
power on
agent on
default-agent
scan on


#Press and hold the PS + Share buttons on the PS4 controller until the light flashes.
#After a moment, you should see the controller show up (e.g., Wireless Controller).

#Pair and connect:

pair <MAC_ADDRESS>
connect <MAC_ADDRESS>
trust <MAC_ADDRESS>
quit

#You should now have a joystick at /dev/input/js0.

#Testing with Controlling the cmd_vel node with the ps4 Controller
ros2 launch p9n_bringup teleop.launch.py topic_name:=cmd_vel hw_type:=DualSense linear_speed:=1.0 angular_speed:=1.0
