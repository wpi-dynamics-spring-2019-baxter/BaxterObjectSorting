# BaxterObjectSorting
Object Sorting with Baxter

Dependencies: ROS Kinetic, Eigen3


To install:

install eigen library from http://eigen.tuxfamily.org/index.php?title=Main_Page#Download

sudo apt-get install gazebo7 ros-kinetic-qt-build ros-kinetic-gazebo-ros-control ros-kinetic-gazebo-ros-pkgs ros-kinetic-ros-control ros-kinetic-control-toolbox ros-kinetic-realtime-tools ros-kinetic-ros-controllers ros-kinetic-xacro python-wstool ros-kinetic-tf-conversions ros-kinetic-kdl-parser ros-kinetic-libfreenect ros-kinetic-freenect-camera ros-kinetic-freenect-launch ros-kinetic-octopmap-server

git clone https://github.com/wpi-dynamics-spring-2019-baxter/BaxterObjectSorting

cd BaxterObjectSorting/catkin_ws/src

wstool init .

wstool merge https://raw.githubusercontent.com/RethinkRobotics/baxter_simulator/kinetic-devel/baxter_simulator.rosinstall

wstool update

cd ..

catkin_make

cp src/baxter/baxter.sh .

edit baxter.sh to have your local IP and ROS version of "Kinetic"
