#include <ros/ros.h>
#include <controller.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller_node");

    ros::NodeHandle nh;

    ros::NodeHandle pnh("~");

    Baxter::Controller cont(nh, pnh);

    while(ros::ok)
    {
        ros::spin();
    }

    return 0;

}
