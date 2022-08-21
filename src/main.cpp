#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotorRotationTorque.h"

#include "chrono/core/ChRealtimeStep.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

// Use the namespaces of Chrono
using namespace chrono;
using namespace chrono::irrlicht;

collision::ChCollisionSystemType collision_type = collision::ChCollisionSystemType::BULLET;

// Shortcut function that creates two bodies (a stator and a rotor) in a given position,
// just to simplify the creation of multiple linear motors in this demo
void CreateStatorRotor(std::shared_ptr<ChBody> &stator,
                       std::shared_ptr<ChBody> &rotor,
                       std::shared_ptr<ChMaterialSurface> material,
                       ChSystem &sys,
                       const ChVector<> &mpos)
{
    stator = chrono_types::make_shared<ChBodyEasyCylinder>(0.5, 0.1, 1000, material, collision_type);
    stator->SetPos(mpos);
    stator->SetRot(Q_from_AngAxis(CH_C_PI_2, VECT_X));
    stator->SetBodyFixed(true);
    sys.Add(stator);

    rotor = chrono_types::make_shared<ChBodyEasyBox>(1, 0.1, 0.1, 1000, material, collision_type);
    rotor->SetPos(mpos + ChVector<>(0.5, 0, -0.15));
    rotor->GetVisualShape(0)->SetColor(ChColor(0.6f, 0.6f, 0.0f));
    sys.Add(rotor);
}

int main(int argc, char *argv[])
{
    SetChronoDataPath(CHRONO_DATA_DIR);

    // Create a ChronoENGINE physical system
    ChSystemNSC sys;
    sys.SetCollisionSystemType(collision_type);

    // Contact material shared among all objects
    auto material = chrono_types::make_shared<ChMaterialSurfaceNSC>();

    // Create a floor that is fixed (that is used also to represent the absolute reference)
    auto floorBody = chrono_types::make_shared<ChBodyEasyBox>(20, 2, 20, 3000, material, collision_type);
    floorBody->SetPos(ChVector<>(0, -2, 0));
    floorBody->SetBodyFixed(true);
    floorBody->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/blue.png"));
    sys.Add(floorBody);

    // Use a ChLinkMotorRotationTorque and compute
    // torque by a custom function. In this example we implement a
    // basic torque(speed) model of a three-phase induction electric motor..
    ChVector<> positionA(-3, 2, 0);
    std::shared_ptr<ChBody> stator;
    std::shared_ptr<ChBody> rotor;
    CreateStatorRotor(stator, rotor, material, sys, positionA);

    // Create the motor
    auto rotmotor = chrono_types::make_shared<ChLinkMotorRotationTorque>();

    // Connect the rotor and the stator and add the motor to the system:
    rotmotor->Initialize(rotor,               // body A (slave)
                          stator,              // body B (master)
                          ChFrame<>(positionA) // motor frame, in abs. coords
    );
    sys.Add(rotmotor);

    // Implement our custom  torque function.
    // We could use pre-defined ChFunction classes like sine, constant, ramp, etc.,
    // but in this example we show how to implement a custom function: a
    // torque(speed) function that represents a three-phase electric induction motor.
    // Just inherit from ChFunction and implement Get_y() so that it returns different
    // values (regrdless of time x) depending only on the slip speed of the motor:
    class MyTorqueCurve : public ChFunction
    {
    public:
        // put here some data that you need when evaluating y(x):
        double E2; // voltage on coil, etc.
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
                (3.0 / 2 * CH_C_PI * ns) * (s * E2 * E2 * R2) / (R2 * R2 + pow(s * X2, 2)); // electric torque curve
            T -= w * 5;                                                                     // simulate also a viscous brake
            return T;
        }
    };
    // Create the function object from our custom class, and initialize its data:
    auto mtorquespeed = chrono_types::make_shared<MyTorqueCurve>();
    mtorquespeed->E2 = 120;
    mtorquespeed->R2 = 80;
    mtorquespeed->X2 = 1;
    mtorquespeed->ns = 6;
    mtorquespeed->mymotor = rotmotor;

    // Let the motor use this motion function as a motion profile:
    rotmotor->SetTorqueFunction(mtorquespeed);

    // Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(&sys);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Motors");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector<>(1, 3, -7));
    vis->AddTypicalLights();
    vis->AddLightWithShadow(ChVector<>(20.0, 35.0, -25.0), ChVector<>(0, 0, 0), 55, 20, 55, 35, 512,
                            ChColor(0.6f, 0.8f, 1.0f));
    vis->EnableShadows();

    // Modify some setting of the physical system for the simulation, if you want
    sys.SetSolverType(ChSolver::Type::PSOR);
    sys.SetSolverMaxIterations(50);

    double timestep = 0.005;
    ChRealtimeStepTimer realtime_timer;
    while (vis->Run())
    {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        sys.DoStepDynamics(timestep);
        realtime_timer.Spin(timestep);

        GetLog() << "angular speed:  " << rotmotor->GetMotorRot_dt() << " [rad/s] \n";
        GetLog() << "torque: " << rotmotor->GetMotorTorque() << " [Ns] \n";
    }

    return 0;
}
