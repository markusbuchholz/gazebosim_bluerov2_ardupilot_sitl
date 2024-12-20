#!/usr/bin/env bash

# Modify this for your environment

# Add results of ArduSub build
export PATH=$HOME/ardupilot/build/sitl/bin:$PATH

# Optional: add autotest to the PATH, helpful for running sim_vehicle.py
export PATH=$HOME/ardupilot/Tools/autotest:$PATH

# Add results of colcon build
source $HOME/colcon_ws/install/setup.bash

# Add ardupilot_gazebo plugin
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:$Z_SIM_SYSTEM_PLUGIN_PATH

# Optional: add ardupilot_gazebo models and worlds
export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH

# Build ros_gz on the humble branch for Gazebo Garden
export GZ_VERSION=garden
