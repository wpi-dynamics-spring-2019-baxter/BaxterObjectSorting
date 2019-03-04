#pragma once
#include <ros/ros.h>
#include <kinematics.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <planner_types.hpp>
#include <tf/tf.h>

namespace Baxter
{

class CollisionDetector
{

public:
    CollisionDetector(ros::NodeHandle &pnh, const ArmState &start_state, const std::string &arm, const octomap_msgs::Octomap::ConstPtr &tree);
    ~CollisionDetector();

    const bool checkForCollision(const ArmState &state);


private:
    void getParams(ros::NodeHandle &pnh);
    void createCollisionPoints();
    void createLinks();
    std::vector<Point> transformColPoints(std::vector<Eigen::Matrix4d> &tfs);

    Kinematics *m_fkin;
    octomap::OcTree *m_oc_tree;
    std::vector<Point> m_collision_pts;
    std::vector<std::vector<Eigen::Vector4d>> m_links;

    int m_num_joints = 7;
    double m_collision_buffer;
    double m_arm_radius;
    double m_collision_cart_res;
    double m_collision_ang_res;
    std::vector<double> m_joint_lengths;
};

}
