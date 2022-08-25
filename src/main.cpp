#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChLinkDistance.h"

using namespace chrono;

class Drone
{
private:
    std::shared_ptr<ChSystemNSC> m_sys;

    std::shared_ptr<ChBody> m_body_n;
    std::shared_ptr<ChBody> m_body_e;
    std::shared_ptr<ChBody> m_body_s;
    std::shared_ptr<ChBody> m_body_w;

    std::shared_ptr<ChLinkDistance> m_link_ne;
    std::shared_ptr<ChLinkDistance> m_link_es;
    std::shared_ptr<ChLinkDistance> m_link_sw;
    std::shared_ptr<ChLinkDistance> m_link_wn;

public:
    Drone(const std::shared_ptr<ChSystemNSC> &sys) : m_sys{sys}
    {
        m_body_n = std::make_shared<ChBody>();
        m_body_n->SetMass(1.0);
        m_body_n->SetPos(Vector(0.0, 5.0, 0.0));
        m_sys->Add(m_body_n);

        m_body_e = std::make_shared<ChBody>();
        m_body_e->SetMass(1.0);
        m_body_e->SetPos(Vector(5.0, 0.0, 0.0));
        m_sys->Add(m_body_e);

        m_body_s = std::make_shared<ChBody>();
        m_body_s->SetMass(1.0);
        m_body_s->SetPos(Vector(-5.0, 0.0, 0.0));
        m_sys->Add(m_body_s);

        m_body_w = std::make_shared<ChBody>();
        m_body_w->SetMass(1.0);
        m_body_w->SetPos(Vector(-5.0, 0.0, 0.0));
        m_sys->Add(m_body_w);

        m_link_ne = std::make_shared<ChLinkDistance>();
        m_link_ne->Initialize(m_body_n, m_body_e, true, Vector(0.0), Vector(0.0));
        m_sys->Add(m_link_ne);

        m_link_es = std::make_shared<ChLinkDistance>();
        m_link_es->Initialize(m_body_e, m_body_s, true, Vector(0.0), Vector(0.0));
        m_sys->Add(m_link_es);

        m_link_sw = std::make_shared<ChLinkDistance>();
        m_link_sw->Initialize(m_body_w, m_body_w, true, Vector(0.0), Vector(0.0));
        m_sys->Add(m_link_sw);

        m_link_wn = std::make_shared<ChLinkDistance>();
        m_link_wn->Initialize(m_body_w, m_body_n, true, Vector(0.0), Vector(0.0));
        m_sys->Add(m_link_wn);
    }

    Vector get_pos() const { return m_body_n->GetPos(); }
    friend std::ostream& operator<<(std::ostream& os, const Drone& drone);


};

std::ostream& operator<<(std::ostream& os, const Drone& drone) {
    return os << "Drone:\n" <<
        "  pos: " << drone.get_pos();
        
}

int main()
{
    auto sys = std::make_shared<ChSystemNSC>();
    sys->Set_G_acc(Vector(0, 0, -9.80665));

    Drone drone(sys);

    const double step_size = 5e-3;
    ChRealtimeStepTimer realtime_timer;
    while (true)
    {
        std::cout << "Time: " << sys->GetChTime() << "\n";
        std::cout << drone << "\n";

        sys->DoStepDynamics(step_size);
        realtime_timer.Spin(step_size);
    }

    return 0;
}
