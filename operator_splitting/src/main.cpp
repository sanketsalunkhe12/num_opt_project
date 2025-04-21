#include <iostream>
#include "operator_splitting/operator_splitting.hpp"
#include <eigen3/Eigen/Dense>
#include "plot.h"

using namespace Eigen;

int main() {
    const int n = 10; // size of x
    const int m = 10; // size of z
    const int max_iter = 1000;
    const double rho = 1.0;
    const double alpha = 1.0;

    VectorXd x = VectorXd::Zero(n);
    VectorXd z = VectorXd::Zero(m);
    VectorXd u = VectorXd::Zero(m);

    MatrixXd A = MatrixXd::Identity(m, n);
    MatrixXd B = -MatrixXd::Identity(m, m);
    VectorXd c = VectorXd::Zero(m);

    for (int k = 0; k < max_iter; ++k) {
        // x-update: solve using a closed-form or call a QP/LS solver
        // Example (least squares): minimize ||Ax + (Bz - c + u)||^2
        VectorXd q = B * z - c + u;
        MatrixXd lhs = A.transpose() * A + rho * MatrixXd::Identity(n, n);
        VectorXd rhs = -A.transpose() * q;
        x = lhs.ldlt().solve(rhs);

        // z-update: proximal operator for g(z), assuming g = 0 for simplicity
        VectorXd v = A * x + u - c;
        z = v;  // or apply proximal operator if g ≠ 0

        // dual variable update
        u = u + A * x + B * z - c;

        // Stopping criteria (primal/dual residual norms) could go here
    }

    std::cout << "x =\n" << x << std::endl;

    plot(); 

    return 0;
}
