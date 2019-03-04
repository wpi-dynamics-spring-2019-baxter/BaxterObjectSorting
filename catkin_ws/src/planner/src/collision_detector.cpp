#include <collision_detector.hpp>

namespace Baxter
{

CollisionDetector::CollisionDetector(ros::NodeHandle &pnh, const ArmState &start_state, const std::string &arm, const octomap_msgs::Octomap::ConstPtr &tree)
{
    m_oc_tree = dynamic_cast<octomap::OcTree*>(octomap_msgs::binaryMsgToMap(*tree));
    m_fkin = new Kinematics(pnh, start_state, arm);
    getParams(pnh);
    createCollisionPoints();
    createLinks();
}

CollisionDetector::~CollisionDetector()
{
    delete m_fkin;
}

void CollisionDetector::getParams(ros::NodeHandle &pnh)
{
    pnh.getParam("collision_buffer", m_collision_buffer);
    pnh.getParam("arm_radius", m_arm_radius);    
    pnh.getParam("collision_cart_res", m_collision_cart_res);
    pnh.getParam("collision_ang_res", m_collision_ang_res);
    m_joint_lengths.resize(m_num_joints);
    pnh.getParam("l0", m_joint_lengths[0]);
    pnh.getParam("l1", m_joint_lengths[1]);
    pnh.getParam("l2", m_joint_lengths[2]);
    pnh.getParam("l3", m_joint_lengths[3]);
    pnh.getParam("l4", m_joint_lengths[4]);
    pnh.getParam("l5", m_joint_lengths[5]);
    pnh.getParam("l6", m_joint_lengths[6]);
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

void CollisionDetector::createLinks()
{
    const int num_angular_pts = 2 * M_PI / m_collision_ang_res;
    std::vector<Eigen::Vector4d> cylinder_pts;
    for(int ang_it = 0; ang_it < num_angular_pts; ang_it++)
    {
        const double &cylinder = ang_it * m_collision_ang_res;
        const double &x = m_arm_radius * cos(cylinder);
        const double &y = m_arm_radius * sin(cylinder);
        Eigen::Vector4d new_pt;
        new_pt << x, y, 0, 1;
        cylinder_pts.push_back(new_pt);
    }
    for(int joint_id = 0; joint_id < m_num_joints; joint_id++)
    {
        std::vector<Eigen::Vector4d> link_pts = cylinder_pts;
        Eigen::Vector4d zero_pt;
        zero_pt << 0, 0, 0, 1;
        link_pts.push_back(zero_pt);
        const int num_z_pts = m_joint_lengths[joint_id] / m_collision_cart_res;
        for(int z_it = 0; z_it < num_z_pts; z_it++)
        {
            ROS_INFO_STREAM("iterator " << z_it << " of " << num_z_pts);
            for(const auto & pt : link_pts)
            {
                Eigen::Vector4d new_pt;
                new_pt << pt(0, 0), pt(1, 0), z_it * m_collision_cart_res, 1;
                link_pts.push_back(new_pt);
            }
        }
        Eigen::Vector4d top_pt;
        top_pt << 0, 0, m_joint_lengths[joint_id], 1;
        link_pts.push_back(top_pt);
        m_links.push_back(link_pts);
    }
}

const bool CollisionDetector::checkForCollision(const ArmState &state)
{
    std::vector<Eigen::Matrix4d> tfs = m_fkin->getTransforms(state);
    std::vector<Point> transformed_pts = transformColPoints(tfs);
    for(const auto &pt : transformed_pts)
    {
        for(const auto &col_pt : m_collision_pts)
        {
            if(fabs(pt.x - col_pt.x) < m_collision_buffer &&
               fabs(pt.y - col_pt.y) < m_collision_buffer &&
               fabs(pt.z - col_pt.z) < m_collision_buffer)
            {
                return true;
            }
        }
    }
    return false;
}

std::vector<Point> CollisionDetector::transformColPoints(std::vector<Eigen::Matrix4d> &tfs)
{
    std::vector<Point> points;
    for(int joint_id = 0; joint_id < m_num_joints; joint_id++)
    {
        for(const auto &pt : m_links[joint_id])
        {
            Eigen::Vector4d tf_pt = tfs[joint_id] * pt;
            points.push_back(Point(tf_pt(0, 0), tf_pt(1, 0), tf_pt(2, 0)));
        }
    }
    return points;
}

}
