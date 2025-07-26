# Add workspaces to the bash so that Terminals will recognize them.

# ROS2 Humble setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Your own Workspace 
echo "source ~/ros2_ws/src/install/setup.bash" >> ~/.bashrc

# First, source #ROS2, then your workspace. Check with:
nano ~/.bashrc
