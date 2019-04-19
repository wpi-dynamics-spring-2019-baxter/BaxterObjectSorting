    //
// Created by akshay on 3/27/19.
//
// dont see the frames in gazebo
//left is x negative
//farther increases z

#include "processPoint.hpp"

int main(int argc, char** argv){
    init(argc, argv, "baxter_kinect_ros_node");

    ros::NodeHandle nh;

    processPoint pp(nh);
    ROS_INFO_STREAM("inside main function");

    ros::spin();
    return 0;
}
