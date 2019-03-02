#include <controller.hpp>

namespace Baxter
{

Controller::Controller(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
    m_controller_server = nh.advertiseService("/execute_trajectory", &Controller::executeTrajectoryRequestCallback, this);
    m_right_controller_pub = nh.advertise<baxter_core_msgs::JointCommand>("/robot/limb/right/joint_command", 10);
    m_left_controller_pub = nh.advertise<baxter_core_msgs::JointCommand>("/robot/limb/left/joint_command", 10);
    pnh.getParam("control_rate", m_control_rate);
}

bool Controller::executeTrajectories(const moveit_msgs::RobotTrajectory &right_traj, const moveit_msgs::RobotTrajectory &left_traj)
{
    ros::Rate rate(m_control_rate);
    const int &num_right_points = right_traj.joint_trajectory.points.size();
    const int &num_left_points = left_traj.joint_trajectory.points.size();
    bool have_right = false;
    bool have_left = false;
    if(num_right_points > 0)
    {
        have_right = true;
    }
    if(num_left_points > 0)
    {
        have_left = true;
    }
    if(!have_right && !have_left)
    {
        return false;
    }
    const ros::Time start_time = ros::Time::now();
    int right_it = 0;
    int left_it = 0;
    baxter_core_msgs::JointCommand right_command;
    baxter_core_msgs::JointCommand left_command;
    right_command.mode = baxter_core_msgs::JointCommand::POSITION_MODE;
    left_command.mode = baxter_core_msgs::JointCommand::POSITION_MODE;
    while(right_it < num_right_points || left_it < num_left_points)
    {
        for(int joint_id = 0; joint_id < m_num_joints; joint_id++)
        {
            if(have_right)
            {
                right_command.command.push_back(right_traj.joint_trajectory.points[right_it].positions[joint_id]);
                right_command.names.push_back(right_traj.joint_trajectory.joint_names[joint_id]);
            }
            if(have_left)
            {
                left_command.command.push_back(right_traj.joint_trajectory.points[left_it].positions[joint_id]);
                left_command.names.push_back(left_traj.joint_trajectory.joint_names[joint_id]);
            }
        }
        const ros::Duration time_from_start = ros::Time::now() - start_time;
        if(have_right)
        {
            ros::Duration right_dur = right_traj.joint_trajectory.points[right_it].time_from_start;
            if(time_from_start > right_dur)
            {
                right_it++;
            }
        }
        if(have_left)
        {
            ros::Duration left_dur = left_traj.joint_trajectory.points[left_it].time_from_start;
            if(time_from_start > left_dur)
            {
                left_it++;
            }
        }
        publishControls(right_command, left_command);
        rate.sleep();
    }
    return true;
}

void Controller::publishControls(const baxter_core_msgs::JointCommand &right, const baxter_core_msgs::JointCommand &left)
{
    m_right_controller_pub.publish(right);
    m_left_controller_pub.publish(left);
}

bool Controller::executeTrajectoryRequestCallback(controller::ExecuteTrajectory::Request &req, controller::ExecuteTrajectory::Response &res)
{
    return executeTrajectories(req.right_trajectory, req.left_trajectory);
}

}
