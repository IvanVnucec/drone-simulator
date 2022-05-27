#pragma once

#include "Eigen/Core"

using Eigen::Vector3d;
using Eigen::Matrix3d;
using Eigen::Array3d;

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
};

class Drone {
private:
    DroneParameters m_params;
    Vector3d m_xyz_dd;
    Vector3d m_eta;
    Array3d m_w_motors;

    Vector3d getTotalThrust();
    Matrix3d getRotationMatrix();

public:
    Drone(DroneParameters params);
    void step();

};