#include "minunit.h"
#include "rk2ndODE.hpp"
#include <cmath>
#include <iostream>

/**
 * solve 2 y'' + 3y' + 4y = 5, y(0)=1, y'(0)=2
 * 
 * solved with https://www.wolframalpha.com/input?i=2+y%27%27%28t%29+%2B+3+y%27%28t%29+%2B+4+y%28t%29%3D+5%2C+y%280%29+%3D+1%2C+y%27%280%29+%3D+2
 * https://www.wolframalpha.com/input?i=derivative+of+29.0+%2F+4.0+%2F+sqrt%2823%29+*+exp%28-3.0+%2F+4+*+t%29+*+sin%28sqrt%2823%29+%2F+4+*+t%29+-+1.0+%2F+4+*+exp%28-3.0+%2F+4+*+t%29+*+cos%28sqrt%2823%29+%2F+4+*+t%29+%2B+5.0+%2F+4
 */

#define TIME_END  (10.0)
#define TIME_STEP (0.00001)

double f(double t, double y, double y_dot) {
    return -3.0 / 2 * y_dot - 2.0 * y + 5.0 / 2;
}

MU_TEST(test_rk) {
    double y = 1.0;
    double y_dot = 2.0;
    double t = 0.0;

	Rk2ndODE<double> rk = Rk2ndODE(TIME_STEP, &f, y, y_dot, t);
    for (; t < TIME_END; t += TIME_STEP) {
        rk.step();
    }

    y = rk.get_y();
    y_dot = rk.get_y_dot();

    const double t_end = TIME_END;
    const double y_end = 29.0 / 4.0 / std::sqrt(23) * std::exp(-3.0 / 4 * t_end) * std::sin(std::sqrt(23) / 4 * t_end) 
        - 1.0 / 4 * std::exp(-3.0 / 4 * t_end) * std::cos(std::sqrt(23) / 4 * t_end) 
        + 5.0 / 4;
    const double y_dot_end = std::exp(-0.75 * t_end) * (2.0 * std::cos(std::sqrt(23) / 4 * t_end) - 0.834058 * std::sin(std::sqrt(23) / 4 * t_end));

    mu_check(std::abs(y_dot - y_dot_end) < 0.01);
    mu_check(std::abs(y - y_end) < 0.01);
}

MU_TEST_SUITE(test_suite) {
	MU_RUN_TEST(test_rk);
}

int main(int argc, char *argv[]) {
	MU_RUN_SUITE(test_suite);

	MU_REPORT();

	return MU_EXIT_CODE;
}
