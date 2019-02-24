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
    pnh.getParam("time_step_ms", m_time_step_ms);
    pnh.getParam("velocity_res", m_velocity_res);
    pnh.getParam("max_angular_accel", m_max_angular_accel);
    pnh.getParam("spline_order", m_spline_order);
    pnh.getParam("max_angular_velocity_shoulder", m_max_angular_velocity_shoulder);
    pnh.getParam("max_angular_velocity_eblow", m_max_angular_velocity_elbow);
    pnh.getParam("max_angular_velocity_wrist", m_max_angular_velocity_wrist);
}

moveit_msgs::RobotTrajectory Planner::planTrajectory(const std::string &arm)
{
    const ArmState current_state = getCurrentState(arm);
    GraphNode current_node(current_state, current_state, 0, 0, std::numeric_limits<double>::infinity());
    openNode(current_node);
    while(true)
    {
        ROS_INFO_STREAM("open nodes: " << m_open_nodes.size() << " closed nodes: " << m_closed_nodes.size() << " frontier size: " << m_frontier.size());
        const GraphNode current_node = m_frontier.top();
        m_frontier.pop();
        closeNode(current_node);
        if(checkForGoal(current_node))
        {
            ROS_INFO_STREAM("path found");
            break;
        }
        expandFrontier(current_node);

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
    std::vector<std::vector<double>> possible_velocities;
    std::vector<std::vector<double>> possible_angles;
    std::vector<Spline1d> splines;
    for(int joint_id = 0; joint_id < 7; joint_id++)
    {
        possible_velocities.push_back(calcPossibleVelocities(current_node.current_state, joint_id));
        possible_angles.push_back(calcPossibleAngles(current_node.current_state, possible_velocities[joint_id], joint_id));
        splines.push_back(calcSpline(joint_id, current_node.current_state.positions[joint_id], current_node.time_from_start));
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
                                const std::vector<std::vector<double>> &vels = possible_velocities;
                                std::vector<double> velocities{vels[0][vel1_it], vels[1][vel2_it], vels[2][vel3_it],
                                                               vels[3][vel4_it], vels[4][vel5_it], vels[5][vel6_it],
                                                               vels[6][vel7_it]};
                                const ArmState state(positions, velocities);
                                const double &time_from_start = current_node.time_from_start + m_time_step_ms / 1000;
                                const double &g = calcG(state, current_node);
                                const double &h = calcH(state, splines, current_node.time_from_start);
                                const double &cost = g + h;
                                GraphNode new_node(state, current_node.current_state, time_from_start, g, cost);
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

const double Planner::calcG(const ArmState &state, const GraphNode &parent_node)
{
    double g = 0;
    for(int i = 0; i < 6; i++)
    {
        g += fabs(state.positions[i] - parent_node.current_state.positions[i]);
    }
    g += parent_node.g;
    return g;
}

const double Planner::calcH(const ArmState &state, const std::vector<Spline1d> &splines, const double &start_time)
{
    double position_heuristic = 0;
    double velocity_heuristic = 0;
    for(int joint_id = 0; joint_id < 7; joint_id++)
    {
        const Spline1d &target_spline = splines[joint_id];
        const double &target_angle = calcTargetJointAngle(target_spline, start_time);
        const double &d_th = fabs(target_angle - state.positions[joint_id]);
        position_heuristic += d_th / (2 * M_PI) * 100;
        const double &d_v = fabs(state.velocities[joint_id] - d_th / (m_time_step_ms / 1000));
        double max_vel;
        if(joint_id < 2)
        {
            max_vel = m_max_angular_velocity_shoulder;
        }
        else if(joint_id < 4 && joint_id > 1)
        {
            max_vel = m_max_angular_velocity_elbow;
        }
        else
        {
            max_vel = m_max_angular_velocity_wrist;
        }
        velocity_heuristic += d_v / (2 * max_vel) * 100;
    }
    return position_heuristic + velocity_heuristic;
}

const double Planner::calcTargetJointAngle(const Spline1d &spline, const double& start_time)
{
    const double &spline_res = 0.01;
    double spline_time = start_time;
    double spline_value = 0;
    double th;
    while(spline_time < start_time + m_time_step_ms / 1000)
    {
        const Eigen::MatrixXd &spline_pt = spline(spline_value);
        th = spline_pt(0);
        spline_time = spline_pt(1);
        spline_value += spline_res;
    }
    return th;
}

const std::vector<double> Planner::calcPossibleVelocities(const ArmState &state, const int &joint_id) //note so self: increase resolution around goal point for convergence
{
    std::vector<double> velocities;
    const double &current_velocity = state.velocities[joint_id];
    const double &max_vel = current_velocity + m_max_angular_accel * m_time_step_ms / 1000;
    const double &min_vel = current_velocity - m_max_angular_accel * m_time_step_ms / 1000;
    const double &vel_range = fabs(max_vel - min_vel);
    const int &num_possible_velocities = int(vel_range / m_velocity_res);
    for(int i = 0; i < num_possible_velocities; i++)
    {
        double velocity = min_vel + double(i) / double(num_possible_velocities) * vel_range;
        double max_vel_;
        if(joint_id < 2)
        {
            max_vel_ = m_max_angular_velocity_shoulder;
        }
        else if(joint_id < 4 && joint_id > 1)
        {
            max_vel_ = m_max_angular_velocity_elbow;
        }
        else
        {
            max_vel_ = m_max_angular_velocity_wrist;
        }
        if(velocity > max_vel_)
        {
            velocity = max_vel_;
        }
        if(velocity < -max_vel_)
        {
            velocity = -max_vel_;
        }
        velocities.push_back(velocity);
    }
    return velocities;
}

const std::vector<double> Planner::calcPossibleAngles(const ArmState &state, const std::vector<double> &velocities, const int &joint_id)
{
    std::vector<double> positions;
    for(const auto &vel : velocities)
    {
        double position = state.positions[joint_id] + vel * m_time_step_ms / 1000;
        if(position > m_angle_maxes[joint_id])
        {
            position = m_angle_maxes[joint_id];
        }
        if(position < m_angle_mins[joint_id])
        {
            position = m_angle_mins[joint_id];
        }
        positions.push_back(position);
    }
    return positions;
}

const bool Planner::checkForGoal(const GraphNode &node)
{
    return node.current_state == *m_goal_state;
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
    std::vector<double> velocities;
    if(arm == "left")
    {
        for(int i = 0; i < 7; i++)
        {
            positions.push_back(m_joint_states->position[i + 3]);
            velocities.push_back(m_joint_states->velocity[i + 3]);
        }
    }
    if(arm == "right")
    {
        for(int i = 0; i < 7; i++)
        {
            positions.push_back(m_joint_states->position[i + 12]);
            velocities.push_back(m_joint_states->velocity[i + 12]);
        }
    }
    return ArmState(positions, velocities);
}

const Spline1d Planner::calcSpline(const int &joint_id, const double &joint_angle, const double &start_time) const
{
    const double &trajectory_time = calcTimeToGoal(joint_id, joint_angle);
    const double &start_angle = joint_angle;
    const double &start_time_ = 0.001 * trajectory_time;
    const double &end_angle = m_goal_state->positions[joint_id];
    const double &end_time = trajectory_time;
    const double &end_time_ = 0.999 * end_time;
    Eigen::MatrixXd points(2, 4);
    points << start_angle, start_angle, end_angle, end_angle,
              start_time, start_time_, end_time_, end_time;
    const Spline1d &spline = Spline1dFitting::Interpolate(points, m_spline_order);
    return spline;
}

const double Planner::calcTimeToGoal(const int &joint_id, const double &joint_angle) const
{
    double max_angular_vel;
    if(joint_id < 2)
    {
        max_angular_vel = m_max_angular_velocity_shoulder;
    }
    else if(joint_id > 1 && joint_id < 4)
    {
        max_angular_vel = m_max_angular_velocity_elbow;
    }
    else
    {
        max_angular_vel = m_max_angular_velocity_wrist;
    }
    const double &angular_error = fabs(joint_angle - m_goal_state->positions[joint_id]);
    return angular_error / max_angular_vel;
}

bool Planner::planRequestCallback(planner::PlanTrajectory::Request &req, planner::PlanTrajectory::Response &res)
{
    std::vector<double> positions;
    std::vector<double> velocities;
    for(int i = 0; i < 7; i++)
    {
        positions.push_back(req.goal_pose.position[i]);
        positions.push_back(req.goal_pose.velocity[i]);
    }
    m_goal_state = new ArmState(positions, velocities);
    res.traj = planTrajectory(req.arm);
    return true;
}

void Planner::jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    m_joint_states = msg;
}

}
