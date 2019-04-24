#include <behaviors_master.hpp>

namespace Baxter
{

BehaviorsMaster::BehaviorsMaster(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
    m_fruit_client = nh.serviceClient<fruitIdentifier::findCentroid>("/get_fruits");
    m_ik_client = nh.serviceClient<baxter_core_msgs::SolvePositionIK>("/ExternalTools/left/PositionKinematicsNode/IKService");
    m_planner_client = nh.serviceClient<planner::PlanTrajectory>("/plan_trajectory");
    m_controller_client = nh.serviceClient<controller::ExecuteTrajectory>("/execute_trajectory");
    m_right_gripper_pub = nh.advertise<baxter_core_msgs::EndEffectorCommand>("/robot/end_effector/right_gripper/command", 10);
    m_left_gripper_pub = nh.advertise<baxter_core_msgs::EndEffectorCommand>("/robot/end_effector/left_gripper/command", 10);
    tf::TransformListener list;
    while(!list.canTransform("base", "camera_link", ros::Time(0)))
    {
        ros::Duration(0.1).sleep();
    }
    try
    {
        list.lookupTransform("base", "camera_link", ros::Time(0), m_base_to_camera_tf);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR_STREAM(ex.what());
    }
    sort();
}

void BehaviorsMaster::sort()
{
    const std::vector<geometry_msgs::PoseStamped> fruits = getFruits();
    ROS_INFO_STREAM("Found " << fruits.size() << " fruits");
    for(const auto &fruit : fruits)
    {
//        tf::StampedTransform tf;
//        tf.setOrigin(tf::Vector3(fruit.pose.position.z, fruit.pose.position.y, fruit.pose.position.x));
//        tf.setRotation(tf::createQuaternionFromYaw(0));
//        auto tf_final = m_base_to_camera_tf * tf;
//        geometry_msgs::Pose pose;
//        pose.position.x = tf_final.getOrigin().getX();
//        pose.position.y = tf_final.getOrigin().getY();
//        pose.position.z = tf_final.getOrigin().getZ();
//        ROS_INFO_STREAM(pose.position.x << " " << pose.position.y << " " << pose.position.z);



        geometry_msgs::Pose pose;
        pose.position.x = fruit.pose.position.z + 0.35;
        pose.position.y = fruit.pose.position.y;
        pose.position.z = 0.19 - fruit.pose.position.x;
        tf::Quaternion q = tf::createQuaternionFromRPY(0, M_PI / 2, 0);
        ROS_INFO_STREAM(pose.position.x << " " << pose.position.y << " " << pose.position.z);
        geometry_msgs::Quaternion fruit_q;
        tf::quaternionTFToMsg(q, fruit_q);
        pose.orientation = fruit_q;
        const sensor_msgs::JointState goal = getIK(pose);
        ROS_INFO_STREAM("got ik");
        moveit_msgs::RobotTrajectory right_traj, left_traj;
        planTrajectory(goal, "left", left_traj);
        ROS_INFO_STREAM("planned trajectory");
        openLeftGripper();
        executeTrajectories(right_traj, left_traj);
        closeLeftGripper();
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
        planTrajectory(goal, "left", left_traj);
        executeTrajectories(right_traj, left_traj);
        openLeftGripper();
    }
}

const std::vector<geometry_msgs::PoseStamped> BehaviorsMaster::getFruits()
{
    fruitIdentifier::findCentroid srv;
    if(!m_fruit_client.call(srv))
    {
        ROS_ERROR_STREAM("failed to get fruits");
    }
    std::vector<geometry_msgs::PoseStamped> fruits;
    fruits = srv.response.final_3d_points;
    return fruits;
}

const sensor_msgs::JointState BehaviorsMaster::getIK(const geometry_msgs::Pose &goal)
{
    baxter_core_msgs::SolvePositionIK srv;
    geometry_msgs::PoseStamped goal_pose;
    goal_pose.header.frame_id = "base";
    goal_pose.pose = goal;
    goal_pose.header.stamp = ros::Time::now();
    goal_pose.header.seq = 0;
    srv.request.pose_stamp.push_back(goal_pose);
    srv.request.seed_mode = 0;
    sensor_msgs::JointState seed;

    seed.name.push_back("right_e0");
    seed.name.push_back("right_e1");
    seed.name.push_back("right_s0");
    seed.name.push_back("right_s1");
    seed.name.push_back("right_w0");
    seed.name.push_back("right_w1");
    seed.name.push_back("right_w2");

    seed.position.push_back(0.4371845240478516);
    seed.position.push_back(1.8419274289489747);
    seed.position.push_back(0.4981602602966309);
    seed.position.push_back(-1.3483691110107423);
    seed.position.push_back(-0.11850001572875977);
    seed.position.push_back(1.18768462366333);
    seed.position.push_back(-0.002300971179199219);

    srv.request.seed_angles.push_back(seed);
    if(!m_ik_client.call(srv))
    {
        ROS_ERROR_STREAM("Failed to get IK");
    }
    ROS_INFO_STREAM(srv.response.joints.begin()->position.size());
    return  srv.response.joints[0];
}

const bool BehaviorsMaster::planTrajectory(const sensor_msgs::JointState &state, const std::string &arm, moveit_msgs::RobotTrajectory &traj)
{
    planner::PlanTrajectory srv;


    sensor_msgs::JointState seed;

    seed.name.push_back("left_e0");
    seed.name.push_back("left_e1");
    seed.name.push_back("left_s0");
    seed.name.push_back("left_s1");
    seed.name.push_back("left_w0");
    seed.name.push_back("left_w1");
    seed.name.push_back("left_w2");

////    seed.position.push_back(0);
////    seed.position.push_back(0);
////    seed.position.push_back(0);
////    seed.position.push_back(0);
////    seed.position.push_back(0);
////    seed.position.push_back(M_PI / 2);
////    seed.position.push_back(0);
///
///
///
///
///
//-1.3642788923947198, -0.5376273957306275, 0.251016411153666, 0.8053979152866118, -2.792382976592562, -1.150662675258322, 1.7703924645355424
    seed.position.push_back(-1.3642788923947198);
   seed.position.push_back(-0.5376273957306275);
   seed.position.push_back(0.251016411153666);
   seed.position.push_back(0.8053979152866118);
   seed.position.push_back(-2.792382976592562);
   seed.position.push_back(-1.150662675258322);
   seed.position.push_back(1.7703924645355424);


    srv.request.goal_pose = seed;
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
