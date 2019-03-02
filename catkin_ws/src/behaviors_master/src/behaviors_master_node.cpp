#include <ros/ros.h>
#include <behaviors_master.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "behaviors_master_node");

    ros::NodeHandle nh;

    ros::NodeHandle pnh("~");

    Baxter::BehaviorsMaster bh(nh, pnh);

    while(ros::ok)
    {
        ros::spin();
    }

    return 0;

}
