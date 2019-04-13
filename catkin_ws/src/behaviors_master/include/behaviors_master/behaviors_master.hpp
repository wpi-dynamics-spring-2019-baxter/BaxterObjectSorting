#pragma once
#include <ros/ros.h>
#include <baxter_core_msgs/EndEffectorCommand.h>
#include <controller/ExecuteTrajectory.h>
#include <planner/PlanTrajectory.h>

namespace Baxter
{

class BehaviorsMaster
{
public:
    BehaviorsMaster(ros::NodeHandle &nh, ros::NodeHandle &pnh);
    ~BehaviorsMaster(){}

private:

    void sort();
    const bool planTrajectory(const sensor_msgs::JointState &state, const std::string &arm, moveit_msgs::RobotTrajectory &traj);
    const bool executeTrajectories(const moveit_msgs::RobotTrajectory &right_traj, const moveit_msgs::RobotTrajectory &left_traj);
    void openRightGripper();
    void closeRightGripper();
    void openLeftGripper();
    void closeLeftGripper();

    ros::ServiceClient m_planner_client;
    ros::ServiceClient m_controller_client;

    ros::Publisher m_right_gripper_pub;
    ros::Publisher m_left_gripper_pub;

};

}
