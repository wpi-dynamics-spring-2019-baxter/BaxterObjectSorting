#pragma once
#include <ros/ros.h>
#include <Eigen/Dense>
#include <geometry_msgs/Point.h>
#include <planner_types.hpp>

namespace Baxter
{

class Kinematics
{

public:
    Kinematics(const ArmState &state);
    ~Kinematics();

    std::vector<geometry_msgs::Point> getCartesianPositions(const ArmState &state);

private:
    void createTransforms(const std::vector<double> &angles);
    void updateTransforms(const std::vector<double> &angles);
    const std::vector<Eigen::Matrix4d> getJointTransforms() const;
    const geometry_msgs::Point extractPointFromTf(const Eigen::Matrix4d &tf) const;
    void createTb0();
    void createT01(const double &angle);
    void createT12(const double &angle);
    void createT23(const double &angle);
    void createT34(const double &angle);
    void createT45(const double &angle);
    void createT56(const double &angle);
    void createT67(const double &angle);
    void createT7g();
    void updateT01(const double &angle);
    void updateT12(const double &angle);
    void updateT23(const double &angle);
    void updateT34(const double &angle);
    void updateT45(const double &angle);
    void updateT56(const double &angle);
    void updateT67(const double &angle);

    Eigen::Matrix4d tb0;
    Eigen::Matrix4d t01;
    Eigen::Matrix4d t12;
    Eigen::Matrix4d t23;
    Eigen::Matrix4d t34;
    Eigen::Matrix4d t45;
    Eigen::Matrix4d t56;
    Eigen::Matrix4d t67;
    Eigen::Matrix4d t7g;

    const double &l0 = 0.27035;
    const double &l1 = 0.06900;
    const double &l2 = 0.36435;
    const double &l3 = 0.06900;
    const double &l4 = 0.37429;
    const double &l5 = 0.01000;
    const double &l6 = 0.38735;
};

}
