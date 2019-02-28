#include <planner.hpp>


namespace Baxter
{

Planner::Planner(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
    m_planner_server = nh.advertiseService("/plan_trajectory", &Planner::planRequestCallback, this);
    m_joint_state_sub = nh.subscribe<sensor_msgs::JointState>("/robot/joint_states", 10, &Planner::jointStateCallback, this);
    m_spline_pub = nh.advertise<nav_msgs::Path>("/spline_viz", 10);
    getParams(pnh);

}

Planner::~Planner()
{
    delete m_fkin;
    delete m_goal_state;
}

void Planner::getParams(ros::NodeHandle &pnh)
{
    m_angle_mins.resize(7);
    m_angle_maxes.resize(7);
    pnh.getParam("th1_min", m_angle_mins[0]);    
    pnh.getParam("th2_min", m_angle_mins[1]);
    pnh.getParam("th3_min", m_angle_mins[2]);
    pnh.getParam("th4_min", m_angle_mins[3]);
    pnh.getParam("th5_min", m_angle_mins[4]);
    pnh.getParam("th6_min", m_angle_mins[5]);
    pnh.getParam("th7_min", m_angle_mins[6]);
    pnh.getParam("th1_max", m_angle_maxes[0]);
    pnh.getParam("th2_max", m_angle_maxes[1]);
    pnh.getParam("th3_max", m_angle_maxes[2]);
    pnh.getParam("th4_max", m_angle_maxes[3]);
    pnh.getParam("th5_max", m_angle_maxes[4]);
    pnh.getParam("th6_max", m_angle_maxes[5]);
    pnh.getParam("th7_max", m_angle_maxes[6]);
    pnh.getParam("angular_joint_res", m_angular_joint_res);
    pnh.getParam("angular_joint_velocity", m_angular_velocity);
    pnh.getParam("spline_order", m_spline_order);
    pnh.getParam("spline_search_res", m_spline_search_res);
    pnh.getParam("pos_error_tol", m_pos_error_tol);
}

moveit_msgs::RobotTrajectory Planner::planTrajectory(const std::string &arm)
{

    const ArmState start_state = getCurrentState(arm);
    GraphNode current_node(start_state, start_state, 0, std::numeric_limits<double>::infinity());
    const GraphNode &start_node = current_node;
    openNode(current_node);
    ros::Time start_time = ros::Time::now();
    ros::Duration dur;
    while(true)
    {
        dur = ros::Time::now() - start_time;
        current_node = m_frontier.top();
        ArmState current_state_ = current_node.current_state;
        m_frontier.pop();
        if(checkForGoal(current_node))
        {
            ROS_INFO_STREAM("path found in " << dur.toSec() << " seconds");
            //return reconstructTrajectory(current_node, start_node);
        }
        expandFrontier(current_node);
        closeNode(current_node);
    }
}

void Planner::expandFrontier(const GraphNode &current_node)
{
    const std::vector<GraphNode> &neighbors = getNeighbors(current_node);
    for(const auto &neighbor : neighbors)
    {
        const std::vector<GraphNode>::iterator &it_closed = std::find(m_closed_nodes.begin(), m_closed_nodes.end(), neighbor);
        if(it_closed != m_closed_nodes.end())
        {
           continue;
        }
        const std::vector<GraphNode>::iterator &it_open = std::find(m_open_nodes.begin(), m_open_nodes.end(), neighbor);
        if(it_open != m_open_nodes.end())
        {
            if(neighbor.cost >= (*it_open).cost)
            {
                continue;
            }
        }
        else
        {
            openNode(neighbor);
        }
    }
}

const std::vector<GraphNode> Planner::getNeighbors(const GraphNode &current_node)
{
    std::vector<GraphNode> neighbors = {};
    std::vector<std::vector<double>> possible_angles;
    for(int joint_id = 0; joint_id < m_num_joints; joint_id++)
    {
        possible_angles.push_back(calcPossibleAngles(current_node.current_state, joint_id));
    }
    int vel1_it = 0;
    for(const auto &angle1 : possible_angles[0])
    {
        int vel2_it = 0;
        for(const auto &angle2 : possible_angles[1])
        {
            int vel3_it = 0;
            for(const auto &angle3 : possible_angles[2])
            {
                int vel4_it = 0;
                for(const auto &angle4 : possible_angles[3])
                {
                    int vel5_it = 0;
                    for(const auto &angle5 : possible_angles[4])
                    {
                        int vel6_it = 0;
                        for(const auto &angle6 : possible_angles[5])
                        {
                            int vel7_it = 0;
                            for(const auto &angle7 : possible_angles[6])
                            {
                                std::vector<double> positions{angle1, angle2, angle3, angle4, angle5, angle6, angle7};
                                const ArmState state(positions);
                                const double &g = calcG(state, current_node);
                                const double &h = calcH(state);
                                const double &cost = g + h;
                                GraphNode new_node(state, current_node.current_state, g, g + cost);
                                neighbors.push_back(new_node);
                                vel7_it++;
                            }
                            vel6_it++;
                        }
                        vel5_it++;
                    }
                    vel4_it++;
                }
                vel3_it++;
            }
            vel2_it++;
        }
        vel1_it++;
    }
    return neighbors;
}

const double Planner::calcG(const ArmState &state, const GraphNode &parent_node) const
{
    double g = 0;
    for(int i = 0; i < 6; i++)
    {
        g += fabs(state.positions[i] - parent_node.current_state.positions[i]);
    }
    g += parent_node.g;
    return g;
}

const double Planner::calcH(const ArmState &state)
{
    double position_heuristic = 0;
    for(int joint_id = 0; joint_id < m_num_joints; joint_id++)
    {
        position_heuristic += fabs(m_goal_state->positions[joint_id] - state.positions[joint_id])  / (2 * M_PI) * 100;
    }
    return position_heuristic;
}

const std::vector<double> Planner::calcPossibleAngles(const ArmState &state, const int &joint_id)
{
    std::vector<double> positions;
    positions.push_back(state.positions[joint_id] - m_angular_joint_res);
    positions.push_back(state.positions[joint_id]);
    positions.push_back(state.positions[joint_id] + m_angular_joint_res);
    return positions;
}

const bool Planner::checkForGoal(const GraphNode &node)
{
    for(int joint_id = 0; joint_id < m_num_joints; joint_id++)
    {
        if(fabs(node.current_state.positions[joint_id] - m_goal_state->positions[joint_id]) > m_pos_error_tol)
        {
            return false;
        }
    }
    return true;
}

void Planner::openNode(const GraphNode &node)
{
    m_frontier.push(node);
    m_open_nodes.push_back(node);
}

void Planner::closeNode(const GraphNode &node)
{
    std::vector<GraphNode>::iterator it = std::find(m_open_nodes.begin(), m_open_nodes.end(), node);
    m_open_nodes.erase(it);
    m_closed_nodes.push_back(node);
}

const ArmState Planner::getCurrentState(const std::string &arm)
{
    std::vector<double> positions;
    if(arm == "left")
    {
        for(int i = 0; i < m_num_joints; i++)
        {
            positions.push_back(m_joint_states->position[i + 3]);
        }
    }
    if(arm == "right")
    {
        for(int i = 0; i < m_num_joints; i++)
        {
            positions.push_back(m_joint_states->position[i + 12]);
        }
    }
    return ArmState(positions);
}

const moveit_msgs::RobotTrajectory Planner::reconstructTrajectory(const GraphNode &current_node_, const GraphNode &start_node)
{
    moveit_msgs::RobotTrajectory reverse_traj;
    GraphNode current_node = current_node_;
    ArmState current_state = current_node.current_state;
    int traj_it = 0;
    while(current_node != start_node)
    {
        const std::vector<GraphNode>::iterator &it_open = std::find(m_open_nodes.begin(), m_open_nodes.end(), current_node);
        if(it_open != m_open_nodes.end())
        {
            reverse_traj.joint_trajectory.points.push_back({});
            for(int joint_id = 0; joint_id < m_num_joints; joint_id++)
            {
                reverse_traj.joint_trajectory.points[traj_it].positions.push_back(it_open->current_state.positions[joint_id]);
            }
            traj_it++;
            for(const auto &node : m_open_nodes)
            {
                if(current_node.parent_state == node.current_state)
                {
                    current_node = node;
                    break;
                }
            }
            continue;
        }
        const std::vector<GraphNode>::iterator &it_closed = std::find(m_closed_nodes.begin(), m_closed_nodes.end(), current_node);
        if(it_closed != m_closed_nodes.end())
        {
            reverse_traj.joint_trajectory.points.push_back({});
            for(int joint_id = 0; joint_id < m_num_joints; joint_id++)
            {
                reverse_traj.joint_trajectory.points[traj_it].positions.push_back(it_closed->current_state.positions[joint_id]);
            }
            traj_it++;
            for(const auto &node : m_closed_nodes)
            {
                if(current_node.parent_state == node.current_state)
                {
                    current_node = node;
                    break;
                }
            }
        }
    }
    return reverseTrajectory(reverse_traj);
}

const moveit_msgs::RobotTrajectory Planner::reverseTrajectory(const moveit_msgs::RobotTrajectory &reverse_traj)
{
    moveit_msgs::RobotTrajectory traj;
    traj.joint_trajectory.header.stamp = ros::Time::now();
    traj.joint_trajectory.points.resize(reverse_traj.joint_trajectory.points.size());
    for(int i = reverse_traj.joint_trajectory.points[0].positions.size() - 1; i > 0; i--)
    {
        for(int joint_id = 0; joint_id < m_num_joints; joint_id++)
        {
            traj.joint_trajectory.points[i].positions.push_back(reverse_traj.joint_trajectory.points[i].positions[joint_id]);
            traj.joint_trajectory.points[i].time_from_start = ros::Duration(i * m_angular_joint_res / m_angular_velocity);
        }
    }
    return traj;
}

const Spline1d Planner::calcSpline(const int &joint_id, const double &joint_angle, const double &joint_vel, const double &start_time) const
{

}

bool Planner::planRequestCallback(planner::PlanTrajectory::Request &req, planner::PlanTrajectory::Response &res)
{
    std::vector<double> positions;
    std::vector<double> velocities;
    for(int i = 0; i < m_num_joints; i++)
    {
        positions.push_back(req.goal_pose.position[i]);
    }
    for(int i = 1; i < 7; i++)
    {
        positions.push_back(0);
    }
    m_goal_state = new ArmState(positions);
    res.traj = planTrajectory(req.arm);
    return true;
}

void Planner::jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    m_joint_states = msg;
}

}
