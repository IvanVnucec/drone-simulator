#include "drone.hpp"
#include <cmath>
#include <iostream>
#include "Eigen/Core"
#include "Eigen/LU"
#include "rk2ndODE.hpp"


using Eigen::Vector3d;
using Eigen::Matrix3d;


Vector3d forward_pos_vel(double t, Vector3d y, Vector3d y_dot);
Vector3d forward_ang_pos_vel(double t, Vector3d y, Vector3d y_dot);


Drone::Drone(DroneParameters params) : 
m_fix{ params },
m_pos{ Vector3d::Zero() },
m_vel{ Vector3d::Zero() },
m_acc{ Vector3d::Zero() },
m_ang_pos{ Vector3d::Zero() },
m_ang_vel{ Vector3d::Zero() },
m_ang_acc{ Vector3d::Zero() },
m_motor_vel{ Vector4d::Zero() },
m_rk_pos_vel{ Rk2ndODE<Vector3d>(m_fix.dt, forward_pos_vel, m_pos, m_vel, 0.0) },
m_rk_ang_pos_vel{ Rk2ndODE<Vector3d>(m_fix.dt, forward_ang_pos_vel, m_ang_pos, m_ang_vel, 0.0) }
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

Matrix3d Drone::getInerToBodyTransfMatrixForAngVel() {
    double Cphi = std::cos(m_ang_pos(0));
    double Sphi = std::sin(m_ang_pos(0));

    double Ctheta = std::cos(m_ang_pos(1));
    double Ttheta = std::tan(m_ang_pos(1));

    return Matrix3d{
        { 1.0, Sphi*Ttheta, Cphi*Ttheta },
        { 0.0, Cphi, -Sphi },
        { 0.0, Sphi/Ctheta, Cphi/Ctheta }
    };
}

Matrix3d Drone::getInertiaMatrix() {
    return Matrix3d{
        { m_fix.Ixx, 0.0, 0.0 },
        { 0.0, m_fix.Iyy, 0.0 },
        { 0.0, 0.0, m_fix.Izz }
    }; 
}

Matrix3d Drone::getJacobian() {
    Matrix3d w = getInerToBodyTransfMatrixForAngVel();
    Matrix3d I = getInertiaMatrix();

    return w.transpose() * I * w;
}

Matrix3d Drone::getCoriolisTerm() {
    double Cphi = std::cos(m_ang_pos(0));
    double Sphi = std::sin(m_ang_pos(0));

    double Ctheta = std::cos(m_ang_pos(1));
    double Stheta = std::sin(m_ang_pos(1));
    
    double phi_dot = m_ang_vel(0);
    double theta_dot = m_ang_vel(1);
    double psi_dot = m_ang_vel(2);

    double Ixx = m_fix.Ixx;
    double Iyy = m_fix.Iyy;
    double Izz = m_fix.Izz;

    double c11 = 0.0;
    double c12 = (Iyy - Izz)*(theta_dot*Cphi*Sphi + psi_dot*Sphi*Sphi*Ctheta) + (Izz - Iyy)*psi_dot*Cphi*Cphi*Ctheta - Ixx*psi_dot*Ctheta;
    double c13 = (Izz - Iyy)*psi_dot*Cphi*Sphi*Ctheta*Ctheta;

    double c21 = (Izz - Iyy)*(theta_dot*Cphi*Sphi + psi_dot*Sphi*Ctheta) + (Iyy - Izz)*psi_dot*Cphi*Cphi*Ctheta + Ixx*psi_dot*Ctheta;;
    double c22 = (Izz - Iyy)*phi_dot*Cphi*Sphi;
    double c23 = -Ixx*psi_dot*Stheta*Ctheta + Iyy*psi_dot*Sphi*Sphi*Stheta*Ctheta + Izz*psi_dot*Cphi*Cphi*Stheta*Ctheta;

    double c31 = (Iyy - Izz)*psi_dot*Ctheta*Ctheta*Sphi*Cphi-Ixx*theta_dot*Ctheta;
    double c32 = (Izz - Iyy)*(theta_dot*Cphi*Sphi*Stheta + phi_dot*Sphi*Sphi*Ctheta) + (Iyy - Izz)*phi_dot*Cphi*Cphi*Ctheta + Ixx*psi_dot*Stheta*Ctheta - Iyy*psi_dot*Sphi*Sphi*Stheta*Ctheta - Izz*psi_dot*Cphi*Cphi*Stheta*Ctheta;
    double c33 = (Iyy - Izz)*phi_dot*Cphi*Sphi*Ctheta*Ctheta - Iyy*theta_dot*Sphi*Sphi*Ctheta*Stheta - Izz*theta_dot*Cphi*Cphi*Ctheta*Stheta + Ixx*theta_dot*Ctheta*Stheta;

    return Matrix3d{
        {c11, c12, c13},
        {c21, c22, c23},
        {c31, c32, c33}
    };
}

Vector3d Drone::getExternalTorque() {
    double l = m_fix.l;
    double k = m_fix.k;
    double w1 = m_motor_vel(0);
    double w2 = m_motor_vel(1);
    double w3 = m_motor_vel(2);
    double w4 = m_motor_vel(3);
    double b = m_fix.b;
    double IM = m_fix.IM;
    double tau_m1 = b * w1*w1 + IM*w1;
    double tau_m2 = b * w2*w2 + IM*w2;
    double tau_m3 = b * w3*w3 + IM*w3;
    double tau_m4 = b * w4*w4 + IM*w4;

    return Vector3d{
        l*k*(w4*w4 - w2*w2),
        l*k*(w3*w3 - w1*w1),
        tau_m1 + tau_m2 + tau_m3 + tau_m4
    };
}

void Drone::step() {
    Vector3d TB = getTotalThrust();
    Matrix3d R = getRotationMatrix();
    Vector3d G = Vector3d(0, 0, -m_fix.g);
    m_acc = (G + R * TB) / m_fix.m;

    Vector3d TauB = getExternalTorque();
    Matrix3d J = getJacobian();
    Matrix3d C = getCoriolisTerm();
    m_ang_acc = J.inverse() * (TauB - C * m_ang_vel);

    m_rk_pos_vel.step();
    m_pos = m_rk_pos_vel.get_y();
    m_vel = m_rk_pos_vel.get_y_dot();

    m_rk_ang_pos_vel.step();
    Vector3d m_ang_pos = m_rk_ang_pos_vel.get_y();
    Vector3d m_ang_vel = m_rk_ang_pos_vel.get_y_dot();
    
    std::cout << "t: " << m_rk_pos_vel.get_time() << std::endl;
    std::cout << "pos: " << m_pos.transpose() << std::endl;
    std::cout << "vel: " << m_vel.transpose() << std::endl;
    std::cout << "ang_pos: " << m_ang_pos.transpose() << std::endl;
    std::cout << "ang_vel: " << m_ang_vel.transpose() << std::endl;
    std::cout << std::endl;
}

Vector3d forward_pos_vel(double t, Vector3d y, Vector3d y_dot) {
    // TODO: add correct equation
    return Vector3d::Constant(1.0);
}

Vector3d forward_ang_pos_vel(double t, Vector3d y, Vector3d y_dot) {
    // TODO: add correct equation
    return Vector3d::Constant(1.0);
}
