#!/bin/bash

# Get the current working directory
orbslam3_directory=$(pwd)
export_command="export ORBSLAM3_PATH=\"$orbslam3_directory\""
# Check if the new path is already in .bashrc
if ! grep -Fxq "$export_command" ~/.bashrc
then
    # If the line does not exist, add it to .bashrc
    echo "export ORBSLAM3_PATH=\"$orbslam3_directory\"" >> ~/.bashrc
    echo "The new PATH has been added to ~/.bashrc"
else
    echo "The PATH already exists in ~/.bashrc"
fi

# Reload the .bashrc file to apply changes
source ~/.bashrc

echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

cd ../../Sophus

echo "Configuring and building Thirdparty/Sophus ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

cd ../../../

echo "Uncompress vocabulary ..."

cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..

echo "Configuring and building ORB_SLAM3 ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j15
