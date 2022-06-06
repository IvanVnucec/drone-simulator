#pragma once

template<typename T>
class Rk2ndODE {
private:
   double m_h;
   T (*m_fun)(double t, T y, T y_dot);
   T m_y;
   T m_y_dot;
   double m_ti;

public:
   Rk2ndODE(double time_step, T (*fun)(double t, T y, T y_dot), T y0, T y0_dot, double t0)
   : m_h{ time_step }, m_fun{ fun }, m_y{ y0 }, m_y_dot{ y0_dot }, m_ti{ t0 }
   {
   }

   double get_time() {
      return m_ti;
   }

   void step() {
      const T ych2 = m_y + m_y_dot * m_h/2;
      const T k1 = m_h * m_fun(m_ti, m_y, m_y_dot);
      const T k2 = m_h * m_fun(m_ti + m_h/2, ych2, m_y_dot + k1/2);
      const T k3 = m_h * m_fun(m_ti + m_h/2, ych2 + k1/4 * m_h, m_y_dot + k2/2);
      const T k4 = m_h * m_fun(m_ti + m_h, m_y + m_y_dot * m_h + m_h/2 * k2 , m_y_dot + k3);

      m_y = m_y + (m_y_dot + 1.0 / 6.0 * (k1 + k2 + k3)) * m_h;
      m_y_dot = m_y_dot + 1.0 / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4);

      m_ti += m_h;
   }

   T get_y() {
      return m_y;
   }

   T get_y_dot() {
      return m_y_dot;
   }
};
