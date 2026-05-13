#!/bin/bash

# Get the current directory
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Set Gazebo resource path to include our models directory
# Gazebo looks for models inside the directories listed in GZ_SIM_RESOURCE_PATH
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$DIR/models

# Run Gazebo Sim with the warehouse world
echo "Launching Gazebo Jazzy (Harmonic) with warehouse world..."
gz sim -r $DIR/worlds/warehouse.sdf
