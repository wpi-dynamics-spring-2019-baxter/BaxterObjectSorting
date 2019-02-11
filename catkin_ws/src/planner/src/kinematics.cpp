#include <kinematics.hpp>

namespace Baxter
{

Kinematics::Kinematics(const std::vector<double> &angles)
{
    createTransforms(angles);
}

Kinematics::~Kinematics(){}


void Kinematics::createTransforms(const std::vector<double> &angles)
{
    createT01(angles[0]);
    createT12(angles[1]);
    createT23(angles[2]);
    createT34(angles[3]);
    createT45(angles[4]);
    createT56(angles[5]);
    createT67(angles[6]);
}

std::vector<geometry_msgs::Point> Kinematics::getCartesianPositions(const std::vector<double> &angles)
{
    updateTransforms(angles);
    const std::vector<Eigen::Matrix4d> &tfs = getJointTransforms();
    std::vector<geometry_msgs::Point> pts;
    for(const auto &tf : tfs)
    {
        pts.push_back(extractPointFromTf(tf));
    }
    return pts;
}

void Kinematics::updateTransforms(const std::vector<double> &angles)
{
    updateT01(angles[0]);
    updateT12(angles[1]);
    updateT23(angles[2]);
    updateT34(angles[3]);
    updateT45(angles[4]);
    updateT56(angles[5]);
    updateT67(angles[6]);
}

const std::vector<Eigen::Matrix4d> Kinematics::getJointTransforms()
{
    std::vector<Eigen::Matrix4d> tfs;
    const Eigen::Matrix4d &t02 = t01 * t12;
    const Eigen::Matrix4d &t03 = t02 * t23;
    const Eigen::Matrix4d &t04 = t03 * t34;
    const Eigen::Matrix4d &t05 = t04 * t45;
    const Eigen::Matrix4d &t06 = t05 * t56;
    const Eigen::Matrix4d &t07 = t06 * t67;
    tfs.push_back(t01);
    tfs.push_back(t02);
    tfs.push_back(t03);
    tfs.push_back(t04);
    tfs.push_back(t05);
    tfs.push_back(t06);
    tfs.push_back(t07);
    return tfs;
}

const geometry_msgs::Point Kinematics::extractPointFromTf(const Eigen::Matrix4d &tf)
{
    geometry_msgs::Point pt;
    pt.x = tf(0, 3);
    pt.y = tf(1, 3);
    pt.z = tf(2, 3);
    return pt;
}

void Kinematics::createT01(const double &angle)
{
    t01(0, 0) = cos(angle);
    t01(0, 1) = -sin(angle);
    t01(0, 2) = 0;
    t01(0, 3) = 0;
    t01(1, 0) = sin(angle);
    t01(1, 1) = cos(angle);
    t01(1, 2) = 0;
    t01(1, 3) = 0;
    t01(2, 0) = 0;
    t01(2, 1) = 0;
    t01(2, 2) = 1;
    t01(2, 3) = 0;
    t01(3, 0) = 0;
    t01(3, 1) = 0;
    t01(3, 2) = 0;
    t01(3, 3) = 1;
}

void Kinematics::createT12(const double &angle)
{
    const double &angle_ = angle + M_PI / 2;
    t12(0, 0) = -sin(angle_);
    t12(0, 1) = -cos(angle_);
    t12(0, 2) = 0;
    t12(0, 3) = l1;
    t12(1, 0) = 0;
    t12(1, 1) = 0;
    t12(1, 2) = 1;
    t12(1, 3) = 0;
    t12(2, 0) = -cos(angle_);
    t12(2, 1) = sin(angle_);
    t12(2, 2) = 0;
    t12(2, 3) = 0;
    t12(3, 0) = 0;
    t12(3, 1) = 0;
    t12(3, 2) = 0;
    t12(3, 3) = 1;
}

void Kinematics::createT23(const double &angle)
{
    t23(0, 0) = cos(angle);
    t23(0, 1) = -sin(angle);
    t23(0, 2) = 0;
    t23(0, 3) = 0;
    t23(1, 0) = 0;
    t23(1, 1) = 0;
    t23(1, 2) = -1;
    t23(1, 3) = -l2;
    t23(2, 0) = sin(angle);
    t23(2, 1) = cos(angle);
    t23(2, 2) = 0;
    t23(2, 3) = 0;
    t23(3, 0) = 0;
    t23(3, 1) = 0;
    t23(3, 2) = 0;
    t23(3, 3) = 1;
}

void Kinematics::createT34(const double &angle)
{
    t34(0, 0) = cos(angle);
    t34(0, 1) = -sin(angle);
    t34(0, 2) = 0;
    t34(0, 3) = l3;
    t34(1, 0) = 0;
    t34(1, 1) = 0;
    t34(1, 2) = 1;
    t34(1, 3) = 0;
    t34(2, 0) = -sin(angle);
    t34(2, 1) = -cos(angle);
    t34(2, 2) = 0;
    t34(2, 3) = 0;
    t34(3, 0) = 0;
    t34(3, 1) = 0;
    t34(3, 2) = 0;
    t34(3, 3) = 1;
}

void Kinematics::createT45(const double &angle)
{
    t45(0, 0) = cos(angle);
    t45(0, 1) = -sin(angle);
    t45(0, 2) = 0;
    t45(0, 3) = 0;
    t45(1, 0) = 0;
    t45(1, 1) = 0;
    t45(1, 2) = -1;
    t45(1, 3) = -l4;
    t45(2, 0) = sin(angle);
    t45(2, 1) = cos(angle);
    t45(2, 2) = 0;
    t45(2, 3) = 0;
    t45(3, 0) = 0;
    t45(3, 1) = 0;
    t45(3, 2) = 0;
    t45(3, 3) = 1;
}

void Kinematics::createT56(const double &angle)
{
    t56(0, 0) = cos(angle);
    t56(0, 1) = -sin(angle);
    t56(0, 2) = 0;
    t56(0, 3) = l5;
    t56(1, 0) = 0;
    t56(1, 1) = 0;
    t56(1, 2) = 1;
    t56(1, 3) = 0;
    t56(2, 0) = -sin(angle);
    t56(2, 1) = -cos(angle);
    t56(2, 2) = 0;
    t56(2, 3) = 0;
    t56(3, 0) = 0;
    t56(3, 1) = 0;
    t56(3, 2) = 0;
    t56(3, 3) = 1;
}

void Kinematics::createT67(const double &angle)
{
    t67(0, 0) = cos(angle);
    t67(0, 1) = -sin(angle);
    t67(0, 2) = 0;
    t67(0, 3) = 0;
    t67(1, 0) = 0;
    t67(1, 1) = 0;
    t67(1, 2) = -1;
    t67(1, 3) = 0;
    t67(2, 0) = sin(angle);
    t67(2, 1) = cos(angle);
    t67(2, 2) = 0;
    t67(2, 3) = 0;
    t67(3, 0) = 0;
    t67(3, 1) = 0;
    t67(3, 2) = 0;
    t67(3, 3) = 1;
}

void Kinematics::updateT01(const double &angle)
{
    t01(0, 0) = cos(angle);
    t01(0, 1) = -sin(angle);
    t01(0, 2) = 0;
    t01(0, 3) = 0;
    t01(1, 0) = sin(angle);
    t01(1, 1) = cos(angle);
    t01(1, 2) = 0;
    t01(1, 3) = 0;
    t01(2, 0) = 0;
    t01(2, 1) = 0;
    t01(2, 2) = 1;
    t01(2, 3) = 0;
    t01(3, 0) = 0;
    t01(3, 1) = 0;
    t01(3, 2) = 0;
    t01(3, 3) = 1;
}

void Kinematics::updateT12(const double &angle)
{
    const double &angle_ = angle + M_PI / 2;
    t12(0, 0) = -sin(angle_);
    t12(0, 1) = -cos(angle_);
    t12(2, 0) = -cos(angle_);
    t12(2, 1) = sin(angle_);
}

void Kinematics::updateT23(const double &angle)
{
    t23(0, 0) = cos(angle);
    t23(0, 1) = -sin(angle);
    t23(2, 0) = sin(angle);
    t23(2, 1) = cos(angle);
}

void Kinematics::updateT34(const double &angle)
{
    t34(0, 0) = cos(angle);
    t34(0, 1) = -sin(angle);
    t34(2, 0) = -sin(angle);
    t34(2, 1) = -cos(angle);
}

void Kinematics::updateT45(const double &angle)
{
    t45(0, 0) = cos(angle);
    t45(0, 1) = -sin(angle);
    t45(2, 0) = sin(angle);
    t45(2, 1) = cos(angle);
}

void Kinematics::updateT56(const double &angle)
{
    t56(0, 0) = cos(angle);
    t56(0, 1) = -sin(angle);
    t56(2, 0) = -sin(angle);
    t56(2, 1) = -cos(angle);
}

void Kinematics::updateT67(const double &angle)
{
    t67(0, 0) = cos(angle);
    t67(0, 1) = -sin(angle);
    t67(2, 0) = sin(angle);
    t67(2, 1) = cos(angle);
}

}
