#include "drone.hpp"
#include <cmath>
#include "Eigen/Core"

using Eigen::Vector3d;
using Eigen::Matrix3d;
using Eigen::Array3d;

Drone::Drone(DroneParameters params) : 
m_params{ params }, 
m_xyz_dd{ Vector3d::Zero() }, 
m_eta{ Vector3d::Zero() },
m_w_motors { Array3d::Zero() }
{

}

Vector3d Drone::getTotalThrust() {
    double total_thrust = 0.0;

    for (const double& w : m_w_motors)
        total_thrust += m_params.k * w * w;
 
    return Vector3d(0.0, 0.0, total_thrust);
}

Matrix3d Drone::getRotationMatrix() {
    double Cphi = std::cos(m_eta(0));
    double Sphi = std::sin(m_eta(0));

    double Ctheta = std::cos(m_eta(1));
    double Stheta = std::sin(m_eta(1));
    
    double Cpsi = std::cos(m_eta(2));
    double Spsi = std::sin(m_eta(2));

    return Matrix3d{
        { Cpsi*Ctheta, Cpsi*Stheta*Sphi - Spsi*Cphi, Cpsi*Stheta*Cphi + Spsi*Sphi },
        { Spsi*Ctheta, Spsi*Stheta*Sphi + Cpsi*Cphi, Spsi*Stheta*Cphi - Cpsi*Sphi },
        { -Stheta,          Ctheta*Sphi,                   Ctheta*Cphi }
    };
}

void Drone::step() {
    Vector3d TB = getTotalThrust();
    Matrix3d R = getRotationMatrix();
    Vector3d G = Vector3d(0, 0, -m_params.g);

    m_xyz_dd = (G + R * TB) / m_params.m;
}
