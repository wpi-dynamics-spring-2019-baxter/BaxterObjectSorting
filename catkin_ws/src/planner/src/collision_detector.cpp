#include <collision_detector.hpp>

namespace Baxter
{

CollisionDetector::CollisionDetector(ros::NodeHandle &pnh, const ArmState &start_state, const octomap_msgs::Octomap::ConstPtr &tree)
{
    m_oc_tree = dynamic_cast<octomap::OcTree*>(octomap_msgs::binaryMsgToMap(*tree));
    pnh.getParam("collision_buffer", m_collision_buffer);
    pnh.getParam("arm_radius", m_arm_radius);
    m_fkin = new Kinematics(start_state);
}

CollisionDetector::~CollisionDetector()
{
    delete m_fkin;
}


void CollisionDetector::createCollisionPoints()
{
    m_collision_pts.clear();
    for(octomap::OcTree::leaf_iterator it = m_oc_tree->begin_leafs(), end = m_oc_tree->end_leafs(); it != end; it++)
    {
        if(m_oc_tree->isNodeOccupied(*it))
        {
            m_collision_pts.push_back(Point(it.getX(), it.getY(), it.getZ()));
        }
    }
}

const bool CollisionDetector::checkForCollision(const ArmState &state)
{

}
}
