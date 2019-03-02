#include <behaviors_master.hpp>

namespace Baxter
{

BehaviorsMaster::BehaviorsMaster(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
    m_planner_client = nh.serviceClient<planner::PlanTrajectory>("/plan_trajectory");
    m_controller_client = nh.serviceClient<controller::ExecuteTrajectory>("/execute_trajectory");
    test();
}

void BehaviorsMaster::test()
{
    ROS_INFO_STREAM("calling plan thing");
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
    ROS_INFO_STREAM("executing");
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


}
