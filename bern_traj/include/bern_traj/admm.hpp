#ifndef ADMM_HPP
#define ADMM_HPP

#include <Eigen/Dense>

class ADMM
{
    public:
        ADMM();
        ~ADMM();

        // ADMM parameters
        double rho; // penalty parameter
        double alpha; // over-relaxation parameter
        int MAX_ITER; // maximum number of iterations
        double ABS_TOL, REL_TOL; // tolerance for convergence

        // ADMM variables
        Eigen::VectorXd x; // primal variable
        Eigen::VectorXd z; // dual variable
        Eigen::VectorXd u; // auxiliary variable

        // ADMM methods
        // void solve(const Eigen::MatrixXd &A, const Eigen::VectorXd &b);
};

#endif // ADMM_HPP