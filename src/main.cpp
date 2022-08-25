#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/core/ChRealtimeStep.h"

using namespace chrono;

int main()
{
    ChSystemNSC sys;
    sys.Set_G_acc(Vector(0, 0, -9.80665));

    auto body = std::make_shared<ChBody>();
    body->SetMass(1.0);
    body->SetPos(Vector(0.0, 0.0, 10.0));

    sys.Add(body);

    const double step_size = 5e-3;
    ChRealtimeStepTimer realtime_timer;
    while (body->GetPos().z() >= 0.0)
    {
        std::cout << "time: " << sys.GetChTime() << "\n";
        std::cout << "pos: " << body->GetPos() << "\n";

        sys.DoStepDynamics(step_size);
        realtime_timer.Spin(step_size);
    }

    return 0;
}
