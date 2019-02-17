#pragma once
#include <ros/ros.h>
#include <kinematics.hpp>
#include <moveit_msgs/RobotTrajectory.h>
#include <planner/PlanTrajectory.h>
#include <planner_types.hpp>
#include <sensor_msgs/JointState.h>

namespace Baxter
{

class Planner
{
public:
    Planner(ros::NodeHandle &nh, ros::NodeHandle &pnh);
    ~Planner();


private:
    bool planRequestCallback(planner::PlanTrajectory::Request &req, planner::PlanTrajectory::Response &res);
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg);
    moveit_msgs::RobotTrajectory planTrajectory(const sensor_msgs::JointState &goal_state, const std::string &arm);


    ros::ServiceServer m_planner_server;
    ros::Subscriber m_joint_state_sub;    

    Kinematics *m_fkin;
    sensor_msgs::JointState::ConstPtr m_joint_states;

};

}
