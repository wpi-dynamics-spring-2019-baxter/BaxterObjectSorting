#pragma once
#include <ros/ros.h>
#include <kinematics.hpp>
#include <moveit_msgs/RobotTrajectory.h>
#include <planner/PlanTrajectory.h>
#include <planner_types.hpp>
#include <queue>
#include <sensor_msgs/JointState.h>
#include <unsupported/Eigen/Splines>

namespace Baxter
{

typedef Eigen::Spline<double, 2> Spline1d;
typedef Eigen::SplineFitting<Spline1d> Spline1dFitting;

class Planner
{
public:
    Planner(ros::NodeHandle &nh, ros::NodeHandle &pnh);
    ~Planner();


private:
    bool planRequestCallback(planner::PlanTrajectory::Request &req, planner::PlanTrajectory::Response &res);
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg);
    moveit_msgs::RobotTrajectory planTrajectory(const std::string &arm);
    const Spline1d calcSpline(const int &joint_id, const double &joint_angle) const;
    const double calcTimeToGoal(const int &joint_id, const double &joint_angle) const;


    ros::ServiceServer m_planner_server;
    ros::Subscriber m_joint_state_sub;    

    std::priority_queue<GraphNode, std::vector<GraphNode>, GraphNode::CheaperCost> m_frontier;
    Kinematics *m_fkin;
    sensor_msgs::JointState::ConstPtr m_joint_states;


    sensor_msgs::JointState m_goal_state;

    //params
    double m_max_angular_velocity_shoulder;
    double m_max_angular_velocity_elbow;
    double m_max_angular_velocity_wrist;
    double m_th1_min;
    double m_th2_min;
    double m_th3_min;
    double m_th4_min;
    double m_th5_min;
    double m_th6_min;
    double m_th7_min;
    double m_th1_max;
    double m_th2_max;
    double m_th3_max;
    double m_th4_max;
    double m_th5_max;
    double m_th6_max;
    double m_th7_max;
    double m_spline_order;


};

}
