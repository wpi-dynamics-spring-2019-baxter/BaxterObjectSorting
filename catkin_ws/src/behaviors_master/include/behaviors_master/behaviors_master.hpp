#pragma once
#include <ros/ros.h>
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

    void test();
    const bool planTrajectory(const sensor_msgs::JointState &state, const std::string &arm, moveit_msgs::RobotTrajectory &traj);
    const bool executeTrajectories(const moveit_msgs::RobotTrajectory &right_traj, const moveit_msgs::RobotTrajectory &left_traj);

    ros::ServiceClient m_planner_client;
    ros::ServiceClient m_controller_client;

};

}
