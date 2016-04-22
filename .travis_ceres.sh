#!/bin/sh
#http://ceres-solver.org/

# Dependency for Ceres
sudo apt-get -qq install -y libeigen3-dev

# Save current directory
dir=`pwd`
cd ..

# glfags
wget https://github.com/gflags/gflags/archive/v2.1.2.zip
unzip v2.1.2.zip > /dev/null
rm v2.1.2.zip
cd gflags-2.1.2
mkdir build
cd build
cmake .. -DBUILD_SHARED_LIBS=ON
make -j2
sudo make -j2 install > /dev/null
cd ../..

# glog
wget https://github.com/google/glog/archive/v0.3.4.zip
unzip v0.3.4.zip > /dev/null
rm v0.3.4.zip
cd glog-0.3.4/
./configure --with-gflags=/usr/local/
make -j2
sudo make -j2 install > /dev/null
cd ../..

# Ceres solver
wget https://github.com/ceres-solver/ceres-solver/archive/1.11.0.zip
unzip 1.11.0.zip > /dev/null
rm 1.11.0.zip
cd ceres-solver-1.11.0
mkdir build
cd build
cmake .. -DBUILD_SHARED_LIBS=ON
make -j2
sudo make -j2 install > /dev/null
cd ../..

# Go back to initial directory
cd $dir
