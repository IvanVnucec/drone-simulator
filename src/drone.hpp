#pragma once

#include "Eigen/Core"
#include "RungeKutta/rk2ndODE.hpp"

using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::Matrix3d;

struct DroneParameters {
    double g;    // m/s2
    double m;    // kg
    double l;    // m
    double k;    //
    double b;    //
    double IM;   // kg m2
    double Ixx;  // kg m2
    double Iyy;  // kg m2
    double Izz;  // kg m2
    double Ax;   // kg/s
    double Ay;   // kg/s
    double Az;   // kg/s
    double dt;   // s
};

class Drone {
private:
    DroneParameters m_fix;
    Vector3d m_pos;
    Vector3d m_vel;
    Vector3d m_acc;

    Vector3d m_ang_pos;
    Vector3d m_ang_vel;
    Vector3d m_ang_acc;
    
    Vector4d m_motor_vel;

    Rk2ndODE<Vector3d> m_rk_pos_vel;
    Rk2ndODE<Vector3d> m_rk_ang_pos_vel;

    Vector3d getTotalThrust();
    Matrix3d getRotationMatrix();
    Matrix3d getJacobian();
    Matrix3d getCoriolisTerm();
    Matrix3d getInerToBodyTransfMatrixForAngVel();
    Matrix3d getInertiaMatrix();
    Vector3d getExternalTorque();


public:
    Drone(DroneParameters params);
    void step();

};