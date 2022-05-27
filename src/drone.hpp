#pragma once

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

public:
    Drone(DroneParameters params);

};