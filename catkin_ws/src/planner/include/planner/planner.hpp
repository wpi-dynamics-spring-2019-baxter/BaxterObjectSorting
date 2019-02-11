#pragma once
#include <ros/ros.h>
#include <kinematics.hpp>
#include <sensor_msgs/JointState.h>

namespace Baxter
{

class Planner
{
public:
    Planner(ros::NodeHandle &nh, ros::NodeHandle &pnh);
    ~Planner();


private:
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg);


    ros::Subscriber m_joint_state_sub;

    Kinematics *m_fkin;
    sensor_msgs::JointState::ConstPtr m_joint_states;

};

}
