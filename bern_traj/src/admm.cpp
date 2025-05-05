#include "bern_traj/admm.hpp"

ADMM::ADMM()
{
    rho = 1.0; // penalty parameter
    alpha = 1.0; // over-relaxation parameter
    MAX_ITER = 1000; // maximum number of iterations
    ABS_TOL = 1e-4; // absolute tolerance for convergence
    REL_TOL = 1e-4; // relative tolerance for convergence
}

ADMM::~ADMM()
{
    // Destructor
}

