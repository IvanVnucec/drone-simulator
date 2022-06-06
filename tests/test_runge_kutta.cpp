#include "minunit.h"
#include "rk2ndODE.hpp"

#define TIME_END  (8.0)
#define TIME_STEP (0.01)
#define G         (9.80665)

double f(double t, double y, double y_dot) {
    return G;
}

MU_TEST(test_rk) {
    double y = 0.0;
    double y_dot = 0.0;
    double t = 0.0;

	Rk2ndODE<double> rk = Rk2ndODE(TIME_STEP, &f, y, y_dot, t);
    for (; t < TIME_END; t += TIME_STEP) {
        rk.step();
    }
    
    y = rk.get_y();
    y_dot = rk.get_y_dot();
    mu_assert_double_eq(78.45, y);
    mu_assert_double_eq(313.8, y_dot);
}

MU_TEST_SUITE(test_suite) {
	MU_RUN_TEST(test_rk);
}

int main(int argc, char *argv[]) {
	MU_RUN_SUITE(test_suite);

	MU_REPORT();

	return MU_EXIT_CODE;
}
