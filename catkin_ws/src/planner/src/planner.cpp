#include <planner.hpp>


namespace Baxter
{

Planner::Planner(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
    m_joint_state_sub = nh.subscribe<sensor_msgs::JointState>("/robot/joint_states", 10, &Planner::jointStateCallback, this);

}

Planner::~Planner()
{
    delete m_fkin;
}




void Planner::jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    m_joint_states = msg;
}

}
