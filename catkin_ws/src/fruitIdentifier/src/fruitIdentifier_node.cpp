#include "findFruits.h"

int main(int argc, char** argv){
    init(argc, argv, "fruitIdentifier_node");

    ros::NodeHandle nh2D;

    Fruits f(nh2D);
    ROS_INFO_STREAM("inside main function");

    ros::spin();

    return 0;
}

