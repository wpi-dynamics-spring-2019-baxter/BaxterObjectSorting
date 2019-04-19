#pragma once
#include <ros/ros.h>
#include <baxter_core_msgs/EndEffectorCommand.h>
#include <controller/ExecuteTrajectory.h>
#include <fruitIdentifier/findCentroid.h>
#include <kinematic_engine/GetIK.h>
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
    const std::vector<geometry_msgs::PoseStamped> getFruits();
    const sensor_msgs::JointState getIK(const geometry_msgs::Pose &goal);
    const bool planTrajectory(const sensor_msgs::JointState &state, const std::string &arm, moveit_msgs::RobotTrajectory &traj);
    const bool executeTrajectories(const moveit_msgs::RobotTrajectory &right_traj, const moveit_msgs::RobotTrajectory &left_traj);
    void openRightGripper();
    void closeRightGripper();
    void openLeftGripper();
    void closeLeftGripper();

    ros::ServiceClient m_fruit_client;
    ros::ServiceClient m_ik_client;
    ros::ServiceClient m_planner_client;
    ros::ServiceClient m_controller_client;

    ros::Publisher m_right_gripper_pub;
    ros::Publisher m_left_gripper_pub;

};

}
