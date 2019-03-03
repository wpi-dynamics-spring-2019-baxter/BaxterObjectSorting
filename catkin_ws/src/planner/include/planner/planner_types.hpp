#pragma once
#include <vector>

namespace Baxter
{

struct Point
{
    Point(const double &x_, const double &y_, const double &z_) :
        x(x_),
        y(y_),
        z(z_)
        {}
    ~Point() = default;
      bool operator==(const Point &rhs) const
      {
          return (rhs.x == x && rhs.y == y && rhs.z == z);
      }

      double x;
      double y;
      double z;

};

struct ArmState
{
    ArmState(){}
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
    GraphNode(){}
    GraphNode(const int &id_, const int &parent_id_, const ArmState &current_state_, const double &g_, const double &cost_) :
        id(id_),
        parent_id(parent_id_),
        current_state(current_state_),
        g(g_),
        cost(cost_)
        {}
    ~GraphNode() = default;

    int id;
    int parent_id;
    ArmState current_state;
    double g;
    double cost;

    bool operator==(const GraphNode &rhs) const
    {
        return (rhs.id == id) &&
               (rhs.parent_id == parent_id) &&
               (rhs.current_state == current_state) &&
               (rhs.g == cost) &&
               (rhs.g == g);
    }
    bool operator!=(const GraphNode &rhs) const
    {
        return  (rhs.id != id) ||
                (rhs.parent_id != parent_id) ||
                (rhs.current_state != current_state) ||
                (rhs.g != cost) ||
                (rhs.g != g);
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
