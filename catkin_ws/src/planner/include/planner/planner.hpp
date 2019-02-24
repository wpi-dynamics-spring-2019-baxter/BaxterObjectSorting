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
    void getParams(ros::NodeHandle &pnh);
    moveit_msgs::RobotTrajectory planTrajectory(const std::string &arm);
    void expandFrontier(const GraphNode &node);
    const double calcG(const ArmState &state, const GraphNode &parent_node);
    const double calcH(const ArmState &state, const std::vector<Spline1d> &splines, const double &start_time);
    const double calcTargetJointAngle(const Spline1d &spline, const double &start_time);
    const std::vector<double> calcPossibleVelocities(const ArmState &state, const int &joint_id);
    const std::vector<double> calcPossibleAngles(const ArmState &state, const std::vector<double> &velocities, const int &joint_id);
    const bool checkForGoal(const GraphNode &node);
    void openNode(const GraphNode &node);
    void closeNode(const GraphNode &node);
    const ArmState getCurrentState(const std::string &arm);
    const Spline1d calcSpline(const int &joint_id, const double &joint_angle, const double &start_time) const;
    const double calcTimeToGoal(const int &joint_id, const double &joint_angle) const;


    ros::ServiceServer m_planner_server;
    ros::Subscriber m_joint_state_sub;    

    std::priority_queue<GraphNode, std::vector<GraphNode>, GraphNode::CheaperCost> m_frontier;
    std::vector<GraphNode> m_open_nodes;
    std::vector<GraphNode> m_closed_nodes;
    Kinematics *m_fkin;
    sensor_msgs::JointState::ConstPtr m_joint_states;


    ArmState *m_goal_state;

    //params
    double m_max_angular_velocity_shoulder;
    double m_max_angular_velocity_elbow;
    double m_max_angular_velocity_wrist;
    std::vector<double> m_angle_mins;
    std::vector<double> m_angle_maxes;
    double m_time_step_ms;
    double m_velocity_res;
    double m_max_angular_accel;
    int m_spline_order;


};

}
