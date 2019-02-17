#include <planner.hpp>


namespace Baxter
{

Planner::Planner(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
    m_planner_server = nh.advertiseService("plan_trajectory", &Planner::planRequestCallback, this);
    m_joint_state_sub = nh.subscribe<sensor_msgs::JointState>("/robot/joint_states", 10, &Planner::jointStateCallback, this);

}

Planner::~Planner()
{
    delete m_fkin;
}

moveit_msgs::RobotTrajectory Planner::planTrajectory(const sensor_msgs::JointState &goal_state, const std::string &arm)
{

}

bool Planner::planRequestCallback(planner::PlanTrajectory::Request &req, planner::PlanTrajectory::Response &res)
{
    res.traj = planTrajectory(req.goal_pose, req.arm);
    return true;
}

void Planner::jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    m_joint_states = msg;
}

}
