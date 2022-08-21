#pragma once

#include "chrono/physics/ChLinkMotorRotationTorque.h"

using namespace chrono;

class MyTorqueCurve : public ChFunction
{
public:
    double E2;
    double R2;
    double X2;
    double ns;
    std::shared_ptr<ChLinkMotorRotationTorque> mymotor;

    virtual MyTorqueCurve *Clone() const override { return new MyTorqueCurve(*this); }

    virtual double Get_y(double x) const override
    {
        // The three-phase torque(speed) model
        double w = mymotor->GetMotorRot_dt();
        double s = (ns - w) / ns; // slip
        double T =
            (3.0 / 2 * CH_C_PI * ns) * (s * E2 * E2 * R2) / (R2 * R2 + pow(s * X2, 2));
        T -= w * 5;
        return T;
    }
};
