#include <controller.hpp>

namespace Baxter
{

Controller::Controller(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
    m_controller_server = nh.advertiseService("/execute_trajectory", &Controller::executeTrajectoryRequestCallback, this);
    m_right_controller_pub = nh.advertise<baxter_core_msgs::JointCommand>("/robot/limb/right/joint_command", 10);
    m_left_controller_pub = nh.advertise<baxter_core_msgs::JointCommand>("/robot/limb/left/joint_command", 10);
}

Controller::~Controller()
{

}

bool Controller::executeTrajectories(const moveit_msgs::RobotTrajectory &right_traj, const moveit_msgs::RobotTrajectory &left_traj)
{
    const int &num_right_points = right_traj.joint_trajectory.points.size();
    const int &num_left_points = left_traj.joint_trajectory.points.size();
}

bool Controller::executeTrajectoryRequestCallback(controller::ExecuteTrajectory::Request &req, controller::ExecuteTrajectory::Response &res)
{
    return executeTrajectories(req.right_trajectory, req.left_trajectory);
}

}
