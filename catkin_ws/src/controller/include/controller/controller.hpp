#pragma once
#include <ros/ros.h>
#include <baxter_core_msgs/JointCommand.h>
#include <controller/ExecuteTrajectory.h>
#include <moveit_msgs/RobotTrajectory.h>

namespace Baxter
{

class Controller
{
public:
    Controller(ros::NodeHandle &nh, ros::NodeHandle &pnh);
    ~Controller(){}


private:
    bool executeTrajectoryRequestCallback(controller::ExecuteTrajectory::Request &req, controller::ExecuteTrajectory::Response &res);
    bool executeTrajectories(const moveit_msgs::RobotTrajectory &right_traj, const moveit_msgs::RobotTrajectory &left_traj);
    void publishControls(const baxter_core_msgs::JointCommand &right, const baxter_core_msgs::JointCommand &left);

    ros::ServiceServer m_controller_server;
    ros::Publisher m_right_controller_pub;
    ros::Publisher m_left_controller_pub;

    int m_control_rate;
    const int m_num_joints = 7;
};

}
