#include <planner.hpp>


namespace Baxter
{

Planner::Planner(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
    m_planner_server = nh.advertiseService("plan_trajectory", &Planner::planRequestCallback, this);
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
}

moveit_msgs::RobotTrajectory Planner::planTrajectory(const std::string &arm)
{
    const ArmState current_state = getCurrentState(arm);
    GraphNode current_node(current_state, current_state, 0, std::numeric_limits<double>::infinity());
    openNode(current_node);
    while(true)
    {
        const GraphNode &current_node = m_frontier.top();
        if(checkForGoal(current_node))
        {

        }
        expandFrontier(current_node);

    }
}

void Planner::expandFrontier(const GraphNode &current_node)
{
    std::vector<std::vector<double>> possible_velocities;
    std::vector<std::vector<double>> possible_angles;
    for(int joint_id = 0; joint_id < 6; joint_id++)
    {
        possible_velocities.push_back(calcPossibleVelocities(current_node.current_state, joint_id));
        possible_angles.push_back(calcPossibleAngles(current_node.current_state, possible_velocities[joint_id], joint_id));
    }
    int angle_set_id;
    for(const auto &angle_set : possible_velocities)
    {
        for(const auto &angle : angle_set)
        {

        }
    }
}

const std::vector<double> Planner::calcPossibleVelocities(const ArmState &state, const int &joint_id)
{
    std::vector<double> velocities;
    const double &current_velocity = state.velocities[joint_id];
    const double &max_vel = current_velocity + m_max_angular_accel * m_time_step_ms / 1000;
    const double &min_vel = current_velocity - m_max_angular_accel * m_time_step_ms / 1000;
    const double &vel_range = fabs(max_vel - min_vel);
    const int &num_possible_velocities = int(vel_range / m_velocity_res);
    for(int i = 1; i < num_possible_velocities; i++)
    {
        double velocity = min_vel + i / num_possible_velocities * vel_range;
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
    std::vector<GraphNode>::iterator it = std::find(m_closed_nodes.begin(), m_closed_nodes.end(), node);
    m_closed_nodes.erase(it);
}

const ArmState Planner::getCurrentState(const std::string &arm)
{
    std::vector<double> positions;
    std::vector<double> velocities;
    if(arm == "left")
    {
        for(int i = 0; i < 6; i++)
        {
            positions.push_back(m_joint_states->position[i + 3]);
            velocities.push_back(m_joint_states->velocity[i + 3]);
        }
    }
    if(arm == "right")
    {
        for(int i = 0; i < 6; i++)
        {
            positions.push_back(m_joint_states->position[i + 12]);
            velocities.push_back(m_joint_states->velocity[i + 12]);
        }
    }
    return ArmState(positions, velocities);
}

const Spline1d Planner::calcSpline(const int &joint_id, const double &joint_angle) const
{
    const double &trajectory_time = calcTimeToGoal(joint_id, joint_angle);
    const double &start_angle = joint_angle;
    const double &start_time = 0;
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
    for(int i = 0; i < 6; i++)
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
