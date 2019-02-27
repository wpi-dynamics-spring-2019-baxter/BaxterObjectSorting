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
    pnh.getParam("time_step_ms", m_time_step_ms);
    pnh.getParam("velocity_res", m_velocity_res);
    pnh.getParam("max_angular_accel", m_max_angular_accel);
    pnh.getParam("spline_order", m_spline_order);
    pnh.getParam("spline_search_res", m_spline_search_res);
    pnh.getParam("pos_error_tol", m_pos_error_tol);
    pnh.getParam("vel_error_tol", m_vel_error_tol);
    pnh.getParam("max_angular_velocity_shoulder", m_max_angular_velocity_shoulder);
    pnh.getParam("max_angular_velocity_elbow", m_max_angular_velocity_elbow);
    pnh.getParam("max_angular_velocity_wrist", m_max_angular_velocity_wrist);
}

moveit_msgs::RobotTrajectory Planner::planTrajectory(const std::string &arm)
{
    const ArmState current_state = getCurrentState(arm);
    GraphNode current_node(current_state, current_state, 0, 0, std::numeric_limits<double>::infinity());
    openNode(current_node);
    while(true)
    {
        ROS_INFO_STREAM("open nodes: " << m_open_nodes.size() << " closed nodes: " << m_closed_nodes.size() << " frontier size: " << m_frontier.size() << " current cost: " <<current_node.cost << " current g: " << current_node.g);
        current_node = m_frontier.top();
        ArmState current_state_ = current_node.current_state;
        ROS_INFO_STREAM(current_state_.positions[0] << " " << current_state_.positions[1] << " " << current_state_.positions[2] << " " << current_state_.positions[3] << " " << current_state_.positions[4] << " " << current_state_.positions[5] << " " << current_state_.positions[6]);
        ROS_INFO_STREAM(current_state_.velocities[0] << " " << current_state_.velocities[1] << " " << current_state_.velocities[2] << " " << current_state_.velocities[3] << " " << current_state_.velocities[4] << " " << current_state_.velocities[5] << " " << current_state_.velocities[6]);
        m_frontier.pop();
        if(checkForGoal(current_node))
        {
            ROS_INFO_STREAM("path found");
            break;
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
    std::vector<std::vector<double>> possible_velocities;
    std::vector<std::vector<double>> possible_angles;
    std::vector<Spline1d> splines;
    std::vector<double> target_angles;
    std::vector<double> target_velocities;
    for(int joint_id = 0; joint_id < m_num_joints; joint_id++)
    {
        possible_velocities.push_back(calcPossibleVelocities(current_node.current_state, joint_id));
        possible_angles.push_back(calcPossibleAngles(current_node.current_state, possible_velocities[joint_id], joint_id));
        splines.push_back(calcSpline(joint_id, current_node.current_state.positions[joint_id],
                                     current_node.current_state.velocities[joint_id],
                                     current_node.time_from_start));
        if(joint_id == 0)
        {
            pubSpline(splines[0]);
        }
        double target_angle, target_velocity;
        calcTargetStates(splines[joint_id], current_node.time_from_start, target_angle, target_velocity);
        target_angles.push_back(target_angle);
        target_velocities.push_back(target_velocity);
    }
    for(int i = 0; i < m_num_joints; i++)
    {
        ROS_INFO_STREAM("joint: " << i + 1 << " target angle: " << target_angles[i] << " target vel: " << target_velocities[i]);
    }
    int vel1_it = 0;
    for(const auto &angle1 : possible_angles[0])
    {
//        int vel2_it = 0;
//        for(const auto &angle2 : possible_angles[1])
//        {
//            int vel3_it = 0;
//            for(const auto &angle3 : possible_angles[2])
//            {
//                int vel4_it = 0;
//                for(const auto &angle4 : possible_angles[3])
//                {
//                    int vel5_it = 0;
//                    for(const auto &angle5 : possible_angles[4])
//                    {
//                        int vel6_it = 0;
//                        for(const auto &angle6 : possible_angles[5])
//                        {
//                            int vel7_it = 0;
//                            for(const auto &angle7 : possible_angles[6])
//                            {
                                std::vector<double> positions{angle1, 0, 0, 0, 0, 0, 0};//angle2, angle3, angle4, angle5, angle6, angle7};
                                const std::vector<std::vector<double>> &vels = possible_velocities;
                                std::vector<double> velocities{vels[0][vel1_it], 0,  0, 0, 0, 0, 0};


//                                            vels[1][vel2_it], vels[2][vel3_it],
//                                                               vels[3][vel4_it], vels[4][vel5_it], vels[5][vel6_it],
//                                                               vels[6][vel7_it]};
                                const ArmState state(positions, velocities);
                                const double &time_from_start = current_node.time_from_start + m_time_step_ms / 1000;
                                const double &g = calcG(state, current_node);
                                const double &h = calcH(state, target_angles, target_velocities);
                                const double &cost = g + h;
                                GraphNode new_node(state, current_node.current_state, time_from_start, g, g + cost);
                                neighbors.push_back(new_node);
//                                vel7_it++;
//                            }
//                            vel6_it++;
//                        }
//                        vel5_it++;
//                    }
//                    vel4_it++;
//                }
//                vel3_it++;
//            }
//            vel2_it++;
//        }
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

const double Planner::calcH(const ArmState &state, const std::vector<double> &target_angles, const std::vector<double> &target_velocities)
{
    double position_heuristic = 0;
    double velocity_heuristic = 0;
    for(int joint_id = 0; joint_id < m_num_joints; joint_id++)
    {
        position_heuristic += fabs(target_angles[joint_id] - state.positions[joint_id]) * 100;
        velocity_heuristic += fabs(state.velocities[joint_id] - target_velocities[joint_id]) * 100;
    }
    return position_heuristic + velocity_heuristic;
}

void Planner::calcTargetStates(const Spline1d &spline, const double& start_time, double &angle, double &velocity) const
{
    const double &spline_res = m_spline_search_res;
    double spline_time = start_time;
    double spline_value = 0;
    double th;
    while(spline_time < start_time + m_time_step_ms / 1000)
    {
        if(spline_value > 1)
        {
            th = std::numeric_limits<double>::infinity();
            velocity = std::numeric_limits<double>::infinity();
            return;
        }
        const Eigen::MatrixXd &spline_pt = spline(spline_value);
        spline_time = spline_pt(0);
        th = spline_pt(1);
        spline_value += spline_res;
    }
    const Eigen::MatrixXd &spline_pt_v = spline(spline_value);
    const double &spline_time_v = spline_pt_v(0);
    const double &th_ = spline_pt_v(1);
    const double &dt = fabs(spline_time_v - spline_time);
    const double &dth = th_ - th;
    const double &vel = dth / dt;
    angle = th;
    velocity = vel;
}

const std::vector<double> Planner::calcPossibleVelocities(const ArmState &state, const int &joint_id) //note so self: increase resolution around goal point for convergence
{
    std::vector<double> velocities;
    const double &current_velocity = state.velocities[joint_id];
    const double &max_vel = current_velocity + m_max_angular_accel * m_time_step_ms / 1000;
    const double &min_vel = current_velocity - m_max_angular_accel * m_time_step_ms / 1000;
    const double &vel_range = fabs(max_vel - min_vel);
    const int &num_possible_velocities = int(vel_range / m_velocity_res);
    for(int i = 0; i < num_possible_velocities + 1; i++)
    {
        double velocity = min_vel + i * m_velocity_res;
        double max_vel_ = getMaxJointVel(joint_id);
        if(velocity > max_vel_)
        {
            velocity = max_vel_;
        }
        else if(velocity < -max_vel_)
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

const double Planner::getMaxJointVel(const double &joint_id)
{
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
    return max_vel;
}

const bool Planner::checkForGoal(const GraphNode &node)
{
    for(int joint_id = 0; joint_id < m_num_joints; joint_id++)
    {
        if(fabs(node.current_state.positions[joint_id] - m_goal_state->positions[joint_id]) > m_pos_error_tol ||
           fabs(node.current_state.velocities[joint_id] - m_goal_state->velocities[joint_id]) > m_vel_error_tol)
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
    std::vector<double> velocities;
    if(arm == "left")
    {
        for(int i = 0; i < m_num_joints; i++)
        {
            positions.push_back(m_joint_states->position[i + 3]);
            velocities.push_back(m_joint_states->velocity[i + 3]);
        }
    }
    if(arm == "right")
    {
        for(int i = 0; i < m_num_joints; i++)
        {
            positions.push_back(m_joint_states->position[i + 12]);
            velocities.push_back(m_joint_states->velocity[i + 12]);
        }
    }
    return ArmState(positions, velocities);
}

const Spline1d Planner::calcSpline(const int &joint_id, const double &joint_angle, const double &joint_vel, const double &start_time) const
{
    const double &trajectory_time = calcTimeToGoal(joint_id, joint_angle);
    const double &start_angle = joint_angle;
    double start_angle_ = joint_angle;
    if(start_time != 0)
    {
        start_angle_ += 0.001 * joint_vel * m_time_step_ms / 1000;
    }
    const double &start_time_ = start_time + 0.001 * trajectory_time;
    const double &end_angle = m_goal_state->positions[joint_id];
    const double &end_time = start_time + trajectory_time;
    const double &end_time_ = end_time - 0.001 * trajectory_time;
    Eigen::MatrixXd points(2, 4);
    points << start_time, start_time_, end_time_, end_time,
              start_angle, start_angle_, end_angle, end_angle;
    const Spline1d &spline = Spline1dFitting::Interpolate(points, m_spline_order);
    return spline;
}

void Planner::pubSpline(const Spline1d &spline)
{
    double it =0;
    double rez = 0.01;
    nav_msgs::Path path;
    path.header.frame_id = "base";
    while(it < 1)
    {
        const Eigen::MatrixXd &spline_pt = spline(it);
        const double &x = spline_pt(0);
        const double &y = spline_pt(1);
        it += rez;
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        path.poses.push_back(pose);
    }
    m_spline_pub.publish(path);
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
    return 4 * angular_error / max_angular_vel;
}

bool Planner::planRequestCallback(planner::PlanTrajectory::Request &req, planner::PlanTrajectory::Response &res)
{
    std::vector<double> positions;
    std::vector<double> velocities;
    for(int i = 0; i < m_num_joints; i++)
    {
        positions.push_back(req.goal_pose.position[i]);
        velocities.push_back(req.goal_pose.velocity[i]);
    }
    for(int i = 1; i < 7; i++)
    {
        positions.push_back(0);
        velocities.push_back(0);
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
