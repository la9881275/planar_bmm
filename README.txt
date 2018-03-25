Planar SLAM
Michael Kaess, 2014

----
Dependencies:

PCL 1.6
OpenCV
OpenNI2
Eigen3  (iSAM)
SuiteSparse  (iSAM)
SDL  (iSAM)
boost (unit tests)

For Ubuntu 12.04:

sudo apt-get install libpcl-1.6-all-dev libopencv-dev libopenni2-dev libeigen-dev libsuitesparse-dev libsdl1.2-dev libboost-test-dev

For Ubuntu 14.04 (with ROS Indigo installed, for OpenCV and PCL):

sudo apt-get install libpcl-1.7-all-dev libopenni2-dev libeigen3-dev libsuitesparse-dev libsdl1.2-dev libboost-test-dev

Install LCM:
sudo apt-get install libglib2.0-dev openjdk-6-jdk
wget -c https://github.com/lcm-proj/lcm/releases/download/v1.3.1/lcm-1.3.1.zip
unzip lcm-1.3.1.zip
cd lcm-1.3.1.zip
./configure
make -j`nproc`
sudo make install 
sudo ldconfig

----
Building:

mkdir build
cd build
cmake ..
make -j4

----
Execution:

./Test
  unit test, should return "No errors detected"

./simulate
  random path and randomly sampled planes (for ICRA paper)

./planar -h
  planar SLAM main executable, try:
./planar -i -f -c data/2014-09-30.10.klg

----
Documentation

ICRA 2015 submission in paper/
