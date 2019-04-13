#include <behaviors_master.hpp>

namespace Baxter
{

BehaviorsMaster::BehaviorsMaster(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
    m_planner_client = nh.serviceClient<planner::PlanTrajectory>("/plan_trajectory");
    m_controller_client = nh.serviceClient<controller::ExecuteTrajectory>("/execute_trajectory");
    m_right_gripper_pub = nh.advertise<baxter_core_msgs::EndEffectorCommand>("/robot/end_effector/right_gripper/command", 10);
    m_left_gripper_pub = nh.advertise<baxter_core_msgs::EndEffectorCommand>("/robot/end_effector/left_gripper/command", 10);
    sort();
}

void BehaviorsMaster::sort()
{
    sensor_msgs::JointState state;
    state.position.push_back(0);
    state.position.push_back(0.5);
    state.position.push_back(0.5);
    state.position.push_back(0.5);
    state.position.push_back(0.5);
    state.position.push_back(0.5);
    state.position.push_back(0.5);
    moveit_msgs::RobotTrajectory right_traj, left_traj;
    planTrajectory(state, "right", right_traj);
    executeTrajectories(right_traj, left_traj);
}

const bool BehaviorsMaster::planTrajectory(const sensor_msgs::JointState &state, const std::string &arm, moveit_msgs::RobotTrajectory &traj)
{
    planner::PlanTrajectory srv;
    srv.request.goal_pose = state;
    srv.request.arm = arm;
    if(!m_planner_client.call(srv))
    {
        ROS_ERROR_STREAM("Failed to Plan Trajectory for " << arm << " Arm");
        return false;
    }
    traj = srv.response.traj;
    return true;
}

const bool BehaviorsMaster::executeTrajectories(const moveit_msgs::RobotTrajectory &right_traj, const moveit_msgs::RobotTrajectory &left_traj)
{
    controller::ExecuteTrajectory srv;
    srv.request.right_trajectory = right_traj;
    srv.request.left_trajectory = left_traj;
    if(!m_controller_client.call(srv))
    {
        ROS_ERROR_STREAM("Failed to Execute Trajectories");
        return false;
    }
    return true;
}

void BehaviorsMaster::openRightGripper()
{
    baxter_core_msgs::EndEffectorCommand cmd;
    cmd.command = baxter_core_msgs::EndEffectorCommand::CMD_GO;
    cmd.args = "{\"position\": 100.0}";
    m_right_gripper_pub.publish(cmd);
}

void BehaviorsMaster::closeRightGripper()
{
    baxter_core_msgs::EndEffectorCommand cmd;
    cmd.command = baxter_core_msgs::EndEffectorCommand::CMD_GO;
    cmd.args = "{\"position\": 0.0}";
    m_right_gripper_pub.publish(cmd);
}

void BehaviorsMaster::openLeftGripper()
{
    baxter_core_msgs::EndEffectorCommand cmd;
    cmd.command = baxter_core_msgs::EndEffectorCommand::CMD_GO;
    cmd.args = "{\"position\": 100.0}";
    m_left_gripper_pub.publish(cmd);
}

void BehaviorsMaster::closeLeftGripper()
{
    baxter_core_msgs::EndEffectorCommand cmd;
    cmd.command = baxter_core_msgs::EndEffectorCommand::CMD_GO;
    cmd.args = "{\"position\": 0.0}";
    m_left_gripper_pub.publish(cmd);
}


}
