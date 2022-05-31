#include "drone.hpp"
#include <cmath>
#include "Eigen/Core"

using Eigen::Vector3d;
using Eigen::Matrix3d;

Drone::Drone(DroneParameters params) : 
m_fix{ params },
m_pos{ Vector3d::Zero() },
m_vel{ Vector3d::Zero() },
m_acc{ Vector3d::Zero() },
m_ang_pos{ Vector3d::Zero() },
m_ang_vel{ Vector3d::Zero() },
m_ang_acc{ Vector3d::Zero() },
m_motor_vel{ Vector3d::Zero() }
{

}

Vector3d Drone::getTotalThrust() {
    double total_thrust = 0.0;

    for (const double& vel : m_motor_vel)
        total_thrust += m_fix.k * vel * vel;
 
    return Vector3d(0.0, 0.0, total_thrust);
}

Matrix3d Drone::getRotationMatrix() {
    double Cphi = std::cos(m_ang_pos(0));
    double Sphi = std::sin(m_ang_pos(0));

    double Ctheta = std::cos(m_ang_pos(1));
    double Stheta = std::sin(m_ang_pos(1));
    
    double Cpsi = std::cos(m_ang_pos(2));
    double Spsi = std::sin(m_ang_pos(2));

    return Matrix3d{
        { Cpsi*Ctheta, Cpsi*Stheta*Sphi - Spsi*Cphi, Cpsi*Stheta*Cphi + Spsi*Sphi },
        { Spsi*Ctheta, Spsi*Stheta*Sphi + Cpsi*Cphi, Spsi*Stheta*Cphi - Cpsi*Sphi },
        {  -Stheta,               Ctheta*Sphi,                  Ctheta*Cphi       }
    };
}

void Drone::step() {
    Vector3d TB = getTotalThrust();
    Matrix3d R = getRotationMatrix();
    Vector3d G = Vector3d(0, 0, -m_fix.g);

    m_acc = (G + R * TB) / m_fix.m;
}
