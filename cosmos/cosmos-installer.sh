#!/bin/bash

# Install dependencies
sudo apt update && sudo apt upgrade -y
sudo apt-get install cmake build-essential -y

cd cosmos
mkdir -p cosmos/source/projects
 
# ========================== Download COSMOS ==========================
echo "Cloning repositories..."

# Clone Cosmos Resources
git clone https://github.com/hsfl/cosmos-resources.git resources 

echo "Cosmos Resources repsitory downloaded"

# Clone Cosmos Core
cd source
git clone https://github.com/hsfl/cosmos-core.git core

echo "Cosmos Core repository downloaded"

echo "Installing Cosmos Core"
cd core/build  
./do_cmake_linux
cd linux
make -j8 install

echo "Cosmos Core Installed"
