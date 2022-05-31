#pragma once

template<typename T>
class Rk4 {
private:
    double     m_h;
    T (*m_fun)(double t, T y);
    T   m_wi;
    double     m_t0;
    unsigned   m_i;

public:
    Rk4(double time_step, T (*fun)(double t, T y), T y0, double t0) 
    : m_h{ time_step }, m_fun{ fun }, m_wi{ y0 }, m_t0{ t0 }, m_i{ 0 }
    {
    }

    double get_time() {
        return m_t0 + m_i * m_h;
    }

    T step() {
        const double ti = get_time();
        const T k1 = m_h * m_fun(ti, m_wi);
        const T k2 = m_h * m_fun(ti + m_h/2, m_wi + k1/2);
        const T k3 = m_h * m_fun(ti + m_h/2, m_wi + k2/2);
        const T k4 = m_h * m_fun(ti + m_h, m_wi + k3);

        m_wi = m_wi + 1.0 / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
        m_i++;

        return m_wi;
    }
};
