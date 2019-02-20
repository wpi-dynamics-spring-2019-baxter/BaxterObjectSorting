#include <vector>

namespace Baxter
{

template <typename Object>

struct Point
{
    Point(const Object &x_, const Object &y_, const Object &z_) :
        x(x_),
        y(y_),
        z(z_)
        {}
    ~Point() = default;
      bool operator==(const Point<Object> &rhs) const
      {
          return (rhs.x == x && rhs.y == y && rhs.z == z);
      }
      Object x;
      Object y;
      Object z;
};

struct ArmState
{
    ArmState(const std::vector<double> &joint_angles, const std::vector<double> &joint_velocities) :
        th1(joint_angles[0]),
        th2(joint_angles[1]),
        th3(joint_angles[2]),
        th4(joint_angles[3]),
        th5(joint_angles[4]),
        th6(joint_angles[5]),
        th7(joint_angles[6]),
        th1_dot(joint_velocities[0]),
        th2_dot(joint_velocities[1]),
        th3_dot(joint_velocities[2]),
        th4_dot(joint_velocities[3]),
        th5_dot(joint_velocities[4]),
        th6_dot(joint_velocities[5]),
        th7_dot(joint_velocities[6])
        {}
    ~ArmState() = default;
    bool operator==(const ArmState &rhs) const
    {
        return (rhs.th1 == th1 &&
                rhs.th2 == th2 &&
                rhs.th3 == th3 &&
                rhs.th4 == th4 &&
                rhs.th5 == th5 &&
                rhs.th6 == th6 &&
                rhs.th7 == th7 &&
                rhs.th1_dot == th1_dot &&
                rhs.th2_dot == th2_dot &&
                rhs.th3_dot == th3_dot &&
                rhs.th4_dot == th4_dot &&
                rhs.th5_dot == th5_dot &&
                rhs.th6_dot == th6_dot &&
                rhs.th7_dot == th7_dot);
    }
    double th1;
    double th2;
    double th3;
    double th4;
    double th5;
    double th6;
    double th7;
    double th1_dot;
    double th2_dot;
    double th3_dot;
    double th4_dot;
    double th5_dot;
    double th6_dot;
    double th7_dot;
};

struct GraphNode
{
    GraphNode(const ArmState &current_state_, const ArmState &parent_state_, const double &g_, const double &cost_) :
        current_state(current_state_),
        parent_state(parent_state_),
        g(g_),
        cost(cost_)
        {}
    ~GraphNode() = default;

    ArmState current_state;
    ArmState parent_state;
    double g;
    double cost;

    bool operator==(const GraphNode &rhs) const
    {
        return (rhs.current_state == current_state) &&
               (rhs.parent_state == parent_state);
    }
    struct CheaperCost
    {
        bool operator()(const GraphNode &lhs, const GraphNode &rhs) const
        {
            return lhs.cost > rhs.cost;
        }
    };

};

}
