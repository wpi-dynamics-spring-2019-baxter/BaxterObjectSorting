#pragma once
#include <ros/ros.h>
#include <kinematics.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <planner_types.hpp>

namespace Baxter
{

class CollisionDetector
{

public:
    CollisionDetector(ros::NodeHandle &pnh, const ArmState &start_state, const octomap_msgs::Octomap::ConstPtr &tree);
    ~CollisionDetector();

    void createCollisionPoints();
    const bool checkForCollision(const ArmState &state);

private:





    Kinematics *m_fkin;
    octomap::OcTree *m_oc_tree;
    std::vector<Point> m_collision_pts;

    double m_collision_buffer;
    double m_arm_radius;
};

}
