#pragma once
#include <ros/ros.h>
#include <kinematics.hpp>
#include <moveit_msgs/RobotTrajectory.h>
#include <nav_msgs/Path.h>
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
    const std::vector<GraphNode> getNeighbors(const GraphNode &current_node);
    const double calcG(const ArmState &state, const GraphNode &parent_node) const;
    const double calcH(const ArmState &state);
    const std::vector<double> calcPossibleAngles(const ArmState &state, const int &joint_id);
    const bool checkForGoal(const GraphNode &node);
    void openNode(const GraphNode &node);
    void closeNode(const GraphNode &node);
    const ArmState getCurrentState(const std::string &arm);
    const moveit_msgs::RobotTrajectory reconstructTrajectory(const GraphNode &current_node_, const GraphNode &start_node);
    const moveit_msgs::RobotTrajectory reverseTrajectory(const moveit_msgs::RobotTrajectory &reverse_traj);
    const Spline1d calcSpline(const int &joint_id, const double &joint_angle, const double &joint_vel, const double &start_time) const;


    ros::ServiceServer m_planner_server;
    ros::Subscriber m_joint_state_sub;
    ros::Publisher m_spline_pub;

    std::priority_queue<GraphNode, std::vector<GraphNode>, GraphNode::CheaperCost> m_frontier;
    std::vector<GraphNode> m_open_nodes;
    std::vector<GraphNode> m_closed_nodes;
    Kinematics *m_fkin;
    sensor_msgs::JointState::ConstPtr m_joint_states;


    ArmState *m_goal_state;

    //params
    std::vector<double> m_angle_mins;
    std::vector<double> m_angle_maxes;
    double m_angular_joint_res;
    double m_angular_velocity;
    int m_spline_order;
    double m_spline_search_res;
    double m_pos_error_tol;

    int m_num_joints = 7;


};

}
