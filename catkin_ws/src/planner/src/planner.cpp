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

moveit_msgs::RobotTrajectory Planner::planTrajectory(const std::string &arm)
{

}

const Spline1d Planner::calcSpline(const int &joint_id, const double &joint_angle) const
{
    const double &trajectory_time = calcTimeToGoal(joint_id, joint_angle);
    const double &start_angle = joint_angle;
    const double &start_time = 0;
    const double &start_time_ = 0.001 * trajectory_time;
    const double &end_angle = m_goal_state.position[joint_id];
    const double &end_time = trajectory_time;
    const double &end_time_ = 0.999 * end_time;
    Eigen::MatrixXd points(2, 4);
    points << start_angle, start_angle, end_angle, end_angle,
              start_time, start_time_, end_time_, end_time;
    const Spline1d &spline = Spline1dFitting::Interpolate(points, m_spline_order);
    return spline;
}

const double Planner::calcTimeToGoal(const int &joint_id, const double &joint_angle) const
{
    double max_angular_vel;
    if(joint_id < 2)
    {
        max_angular_vel = m_max_angular_velocity_shoulder;
    }
    else if(joint_id > 1 && joint_id < 4)
    {
        max_angular_vel = m_max_angular_velocity_elbow;
    }
    else
    {
        max_angular_vel = m_max_angular_velocity_wrist;
    }
    const double &angular_error = fabs(joint_angle - m_goal_state.position[joint_id]);
    return angular_error / max_angular_vel;
}

bool Planner::planRequestCallback(planner::PlanTrajectory::Request &req, planner::PlanTrajectory::Response &res)
{
    m_goal_state = req.goal_pose;
    res.traj = planTrajectory(req.arm);
    return true;
}

void Planner::jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    m_joint_states = msg;
}

}
