#!/bin/bash

# # Update the package list
# sudo apt update

# # Install required dependencies

sudo apt install -y \
    build-essential \
    cmake \
    git \
    libeigen3-dev \
    libgtk-3-dev \
    pkg-config \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libv4l-dev \
    libxvidcore-dev \
    libx264-dev \
    openexr \
    libatlas-base-dev \
    libopenexr-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer1.0-dev \
    python3-dev \
    python3-numpy \
    libtbb2 \
    libtbb-dev \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    libdc1394-dev \
    gfortran \
    libgtkglext1 \
    libgtkglext1-dev \
    libglu1-mesa-dev \
    freeglut3-dev \
    mesa-common-dev \
    libgl1-mesa-dev \
    libglfw3-dev \
    libglew-dev \
    libqt5opengl5-dev \
    mesa-utils \
    libgoogle-glog-dev \
    libgflags-dev \
    libatlas-base-dev \
    libsuitesparse-dev \
    libspdlog-dev \
    libsuitesparse-dev \
    qtdeclarative5-dev \
    qt5-qmake \
    libqglviewer-dev-qt5 \
    sudo apt-get install libssl-dev \
    sudo apt install libboost-all-dev \

# Clean up after installation
sudo apt autoremove -y
sudo apt clean

# Clone the OpenCV repository
OPENCV_VERSION="4.9.0"
git clone -b $OPENCV_VERSION https://github.com/opencv/opencv.git
git clone -b $OPENCV_VERSION https://github.com/opencv/opencv_contrib.git

git clone https://github.com/gabime/spdlog.git
# # Clone the g2o repository
git clone https://github.com/RainerKuemmerle/g2o.git

# # Clone the ceres repository
git clone https://github.com/ceres-solver/ceres-solver.git

# # Clone the fmt repository
git clone https://github.com/fmtlib/fmt.git

# # Clone the pangolin repository
git clone https://github.com/stevenlovegrove/Pangolin.git


# # Create a build directory for OpenCV
cd ./opencv
mkdir build
cd build

# # Run cmake to configure the build with the contrib modules
cmake cmake -D CMAKE_BUILD_TYPE=RELEASE \
            -D CMAKE_INSTALL_PREFIX=/usr/local \
            -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
            -D BUILD_TIFF=ON \
            -D WITH_FFMPEG=ON \
            -D WITH_GSTREAMER=ON \
            -D WITH_TBB=ON \
            -D WITH_GTK=ON \
            -D BUILD_TBB=ON \
            -D WITH_EIGEN=ON \
            -D WITH_V4L=ON \
            -D WITH_LIBV4L=ON \
            -D WITH_VTK=ON \
            -D WITH_QT=ON \
            -D WITH_OPENGL=ON \
            -D BUILD_opencv_highgui=ON \
            -D BUILD_opencv_imgcodecs=ON \
            -D OPENCV_ENABLE_NONFREE=ON \
            -D INSTALL_C_EXAMPLES=OFF \
            -D INSTALL_PYTHON_EXAMPLES=OFF \
            -D BUILD_NEW_PYTHON_SUPPORT=OFF \
            -D OPENCV_GENERATE_PKGCONFIG=ON \
            -D OPENCV_DNN_CUDA=OFF \
            -D BUILD_TESTS=OFF \
            -D WITH_LAPACK=OFF \
            -D BUILD_EXAMPLES=OFF ../


# # Build OpenCV with all available CPU cores
make -j$(nproc)

# # Install OpenCV
sudo make install
sudo ldconfig

echo "OpenCV 4.8 has been successfully installed."


cd ../../fmt
mkdir build
cd build


cmake cmake ../

make -j$(nproc)

sudo make install
sudo ldconfig

echo "fmt has been successfully installed."

#########################################

cd ../../spdlog
mkdir build
cd build


cmake  cmake -D CMAKE_POSITION_INDEPENDENT_CODE=ON ../

make -j$(nproc)

sudo make install
sudo ldconfig

echo "spdlog has been successfully installed."

#########################################


cd ../../Pangolin
mkdir build
cd build

cmake cmake ../

make -j$(nproc)

sudo make install
sudo ldconfig

echo "Pangolin has been successfully installed."


#########################################

cd ../../g2o
mkdir build
cd build

cmake cmake ../

make -j$(nproc)

sudo make install
sudo ldconfig

echo "g2o has been successfully installed."

#########################################

cd ../../ceres
mkdir build
cd build

cmake cmake ../

make -j$(nproc)

sudo make install
sudo ldconfig

echo "ceres has been successfully installed."

# sudo apt install libgl1-mesa-dev libglew-dev cmake  pkg-config libegl1-mesa-dev libwayland-dev libxkbcommon-dev wayland-protocols ffmpeg libavcodec-dev libavutil-dev libavformat-dev libswscale-dev libavdevice-dev libjpeg-dev libtiff5-dev libopenexr-dev python3-pip g++ git gcc