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
    Kinematics(ros::NodeHandle &pnh, const ArmState &state, const std::string &arm);
    ~Kinematics();

    std::vector<Eigen::Matrix4d> getTransforms(const ArmState &state);

private:
    void createTransforms(const std::vector<double> &angles);
    void updateTransforms(const std::vector<double> &angles);
    const std::vector<Eigen::Matrix4d> getJointTransforms() const;
    void createTbab();
    void createTab0();
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

    std::string m_arm;

    Eigen::Matrix4d tbab;
    Eigen::Matrix4d tab0;
    Eigen::Matrix4d t01;
    Eigen::Matrix4d t12;
    Eigen::Matrix4d t23;
    Eigen::Matrix4d t34;
    Eigen::Matrix4d t45;
    Eigen::Matrix4d t56;
    Eigen::Matrix4d t67;
    Eigen::Matrix4d t7g;

    double l0;
    double l1;
    double l2;
    double l3;
    double l4;
    double l5;
    double l6;
    double H;
    double L;
    double h;
};

}
