#include <planner.hpp>


namespace Baxter
{

Planner::Planner(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
    m_planner_server = nh.advertiseService("/plan_trajectory", &Planner::planRequestCallback, this);
    m_joint_state_sub = nh.subscribe<sensor_msgs::JointState>("/robot/joint_states", 10, &Planner::jointStateCallback, this);
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
    pnh.getParam("spline_res", m_spline_res);
    pnh.getParam("pos_error_tol", m_pos_error_tol);
}

void Planner::initializePlanner()
{
    m_frontier = std::priority_queue<GraphNode, std::vector<GraphNode>, GraphNode::CheaperCost>();
    m_nodes.clear();
    m_open_nodes.clear();
    m_closed_nodes.clear();
    m_joint_names.clear();
}

moveit_msgs::RobotTrajectory Planner::planTrajectory(const std::string &arm)
{
    initializePlanner();
    const ArmState start_state = getCurrentState(arm);
    m_fkin = new Kinematics(start_state);
//    tf::TransformListener list;
//    tf::StampedTransform trans;
//    ros::Duration(5).sleep();
//    list.lookupTransform("left_arm_mount", "l_gripper_l_finger_tip", ros::Time(0), trans);
//    ROS_INFO_STREAM(trans.getOrigin().getX() << " " << trans.getOrigin().getY() << " " << trans.getOrigin().getZ());
//    auto transforms = m_fkin->getCartesianPositions(start_state);
//    ROS_INFO_STREAM(transforms.back().x << " " << transforms.back().y << " " << transforms.back().z);
    GraphNode current_node(m_graph_id, m_graph_id, start_state, 0, std::numeric_limits<double>::infinity());
    m_graph_id++;
    openNode(current_node);
    ros::Time start_time = ros::Time::now();
    ros::Duration dur;
    while(true)
    {
        dur = ros::Time::now() - start_time;
        current_node = m_frontier.top();        
        if(checkForGoal(current_node))
        {
            ROS_INFO_STREAM("Trajectory for " << arm << " Arm Found in " << dur.toSec() << " seconds");
            return reconstructTrajectory();
        }
        m_frontier.pop();
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
                                if(state == current_node.current_state)
                                {
                                    continue;
                                }
                                const double &g = calcG(state, current_node);
                                const double &h = calcH(state);
                                const double &cost = g + h;
                                GraphNode new_node(m_graph_id, current_node.id, state, g, g + cost);
                                m_graph_id++;
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
    m_nodes.insert(std::make_pair(node.id, node));
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
            m_joint_names.push_back(m_joint_states->name[i + 3]);
        }
    }
    if(arm == "right")
    {
        for(int i = 0; i < m_num_joints; i++)
        {
            positions.push_back(m_joint_states->position[i + 12]);
            m_joint_names.push_back(m_joint_states->name[i + 12]);
        }
    }
    std::vector<double> positions_ = positions;
    std::vector<std::string> names = m_joint_names;
    positions[0] = positions_[2];
    positions[1] = positions_[3];
    positions[2] = positions_[0];
    positions[3] = positions_[1];
    m_joint_names[0] = names[2];
    m_joint_names[1] = names[3];
    m_joint_names[2] = names[0];
    m_joint_names[3] = names[1];
    return ArmState(positions);
}

const moveit_msgs::RobotTrajectory Planner::reconstructTrajectory()
{
    std::vector<ArmState> reverse_states;
    GraphNode current_node = m_frontier.top();
    while(current_node.id != 0)
    {
        reverse_states.push_back(current_node.current_state);
        current_node = m_nodes[current_node.parent_id];
    }
    std::vector<ArmState> states = reverseStates(reverse_states);
    std::vector<std::vector<double>> joint_angles;
    joint_angles.resize(m_num_joints);
    std::vector<Spline1d> splines;
    for(const auto &state : states)
    {
        for(int joint_id = 0; joint_id < m_num_joints; joint_id++)
        {
            joint_angles[joint_id].push_back(state.positions[joint_id]);
        }
    }
    for(const auto &joint : joint_angles)
    {
        splines.push_back(calcSpline(joint));
    }
    return trajFromSplines(splines);
}

const std::vector<ArmState> Planner::reverseStates(const std::vector<ArmState> &reverse_states)
{
    std::vector<ArmState> states;
    for(int state = reverse_states.size() - 1; state >= 0; state--)
    {
        states.push_back(reverse_states[state]);
    }
    states.push_back(*m_goal_state);
    return states;
}

const moveit_msgs::RobotTrajectory Planner::trajFromSplines(const std::vector<Spline1d> &splines)
{
    moveit_msgs::RobotTrajectory traj;
    traj.joint_trajectory.header.stamp = ros::Time::now();
    for(int joint_id = 0; joint_id < m_num_joints; joint_id ++)
    {
        traj.joint_trajectory.joint_names.push_back(m_joint_names[joint_id]);
    }
    std::vector<std::vector<double>> points;
    std::vector<double> times;
    double spline_it = 0;
    while(spline_it <= 1)
    {
        std::vector<double> angles;
        Eigen::MatrixXd spline_pt;
        for(const auto spline : splines)
        {
            spline_pt = spline(spline_it);
            angles.push_back(spline_pt(1));
        }
        points.push_back(angles);
        times.push_back(spline_pt(0));
        spline_it += m_spline_res;
    }
    traj.joint_trajectory.points.resize(points.size());
    for(int point_it = 0; point_it < points.size(); point_it++)
    {
        for(const auto &angle : points[point_it])
        {
            traj.joint_trajectory.points[point_it].positions.push_back(angle);
        }
        traj.joint_trajectory.points[point_it].time_from_start = ros::Duration(times[point_it]);
    }
    return traj;
}

const Spline1d Planner::calcSpline(const std::vector<double> &angles)
{
    Eigen::MatrixXd points(2, angles.size());
    for(int angle_it = 0; angle_it < angles.size(); angle_it++)
    {
        points(1, angle_it) = angles[angle_it];
    }
    for(int time_it = 0; time_it < angles.size(); time_it++)
    {
        points(0, time_it) = time_it * m_angular_joint_res / m_angular_velocity;
    }
    const Spline1d &spline = Spline1dFitting::Interpolate(points, m_spline_order);
    return spline;
}

bool Planner::planRequestCallback(planner::PlanTrajectory::Request &req, planner::PlanTrajectory::Response &res)
{
    std::vector<double> positions;
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
