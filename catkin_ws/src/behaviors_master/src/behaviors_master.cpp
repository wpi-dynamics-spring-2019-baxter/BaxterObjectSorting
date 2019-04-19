#include <behaviors_master.hpp>

namespace Baxter
{

BehaviorsMaster::BehaviorsMaster(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
    m_fruit_client = nh.serviceClient<fruitIdentifier::findCentroid>("/get_fruits");
    m_ik_client = nh.serviceClient<kinematic_engine::GetIK>("/get_ik");
    m_planner_client = nh.serviceClient<planner::PlanTrajectory>("/plan_trajectory");
    m_controller_client = nh.serviceClient<controller::ExecuteTrajectory>("/execute_trajectory");
    m_right_gripper_pub = nh.advertise<baxter_core_msgs::EndEffectorCommand>("/robot/end_effector/right_gripper/command", 10);
    m_left_gripper_pub = nh.advertise<baxter_core_msgs::EndEffectorCommand>("/robot/end_effector/left_gripper/command", 10);
    sort();
}

void BehaviorsMaster::sort()
{
    const std::vector<geometry_msgs::PoseStamped> fruits = getFruits();
    for(const auto &fruit : fruits)
    {
        const sensor_msgs::JointState goal = getIK(fruit.pose);
        moveit_msgs::RobotTrajectory right_traj, left_traj;
        planTrajectory(goal, "right", right_traj);
        openRightGripper();
        executeTrajectories(right_traj, left_traj);
        closeRightGripper();
        geometry_msgs::Pose pose;
        if(fruit.header.frame_id == "/x01")
        {
            pose = fruit.pose;
            pose.position.x = pose.position.x - 0.5;
        }
        if(fruit.header.frame_id == "/x02")
        {
            pose = fruit.pose;
            pose.position.x = pose.position.x + 0.5;
        }
        const sensor_msgs::JointState goal_placed = getIK(pose);
        planTrajectory(goal, "right", right_traj);
        executeTrajectories(right_traj, left_traj);
        openRightGripper();
    }
}

const std::vector<geometry_msgs::PoseStamped> BehaviorsMaster::getFruits()
{
    fruitIdentifier::findCentroid srv;
    m_fruit_client.call(srv);
    std::vector<geometry_msgs::PoseStamped> fruits;
    fruits = srv.response.final_3d_points;
    return fruits;
}

const sensor_msgs::JointState BehaviorsMaster::getIK(const geometry_msgs::Pose &goal)
{

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
