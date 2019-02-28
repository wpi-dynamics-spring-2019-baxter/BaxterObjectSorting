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
    ArmState(const std::vector<double> &positions_) : positions(positions_){}
    bool operator==(const ArmState &rhs) const
    {
        for(int i = 0; i < positions.size(); i++)
        {
            if(rhs.positions[i] != positions[i])
            {
                return false;
            }
        }
        return true;
    }
    bool operator!=(const ArmState &rhs) const
    {
        for(int i = 0; i < positions.size(); i++)
        {
            if(rhs.positions[i] != positions[i])
            {
                return true;
            }
        }
        return false;
    }
    std::vector<double> positions;
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
    bool operator!=(const GraphNode &rhs) const
    {
        return (rhs.current_state != current_state) &&
               (rhs.parent_state != parent_state);
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
