#ifndef OPTIMIZER_HPP
#define OPTIMIZER_HPP

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <tbb/tbb.h> // do we want to use this for threading?
#include <vector>
#include <utility>
#include <unordered_map>

class DQPSolver
{
    public:

        DQPSolver();

        // Also set iteration number to start (should we actually care about residuals? Probably...)
        // We're definitely not going to compute whether the problem is feasible or not, that's for sure
        void init(const Eigen::MatrixXd &Q, const Eigen::MatrixXd &A, const Eigen::VectorXd &lower, const Eigen::VectorXd &upper, const float alpha, const float rho, const float mu); // Pass in all relevant data and initialize all member vector/matrix variables with correct sizes
        void compute_KKT_rhs();
        void solve_KKT();
        void update_primals(); // Compute new rhs, solve KKT, set x and nu, update z, update s
        void update_global(const Eigen::VectorXd &w_new);
        void update_duals();
        int check_termination(); // should return 0 if not yet terminated, or 1 if terminated
        
        // Getters and setters
        float getPrimalResidual();
        float getDualResidual();
    
        // Data and constraints
        int m;
        int n;
        Eigen::MatrixXd Q; // PSD Objective Function matrix
        Eigen::MatrixXd A; // Equality+Inequality constraints
        // no q vector since we don't have a linear component to our problem's objective function
        // Lower and upper bounds stand in for the "b" vector in DQP, where s is projected onto these box constraints via min max clip operation
        Eigen::VectorXd lower; 
        Eigen::VectorXd upper; 
        float alpha = 1.0; // step size
        float rho = 1.0; // penalty param: constant, not adapting this at the moment
        float mu = 1.0; // penalty param: constant, not adapting this at the moment

        // cached variables
        float rhoinv = 1.0; // 1/rho
        Eigen::MatrixXd lhs;
        Eigen::MatrixXd rhs;
        Eigen::LDLT<Eigen::MatrixXd> ldlt_solver;

        // optimization variables (primals, duals)
        Eigen::VectorXd x;
        Eigen::VectorXd nu;
        Eigen::VectorXd z;
        Eigen::VectorXd s;
        Eigen::VectorXd w_tilde; // basically the local copy of relevant global parameters G(w, i, j) -> {w_l}
        Eigen::VectorXd lamb;
        Eigen::VectorXd y;

        // For most of these vars, need a way to store the previous value for updates
        Eigen::VectorXd x_prev;
        Eigen::VectorXd nu_prev;
        Eigen::VectorXd z_prev;
        Eigen::VectorXd s_prev;
        Eigen::VectorXd w_tilde_prev; // basically the local copy of relevant global parameters G(w, i, j) -> {w_l}
        Eigen::VectorXd lamb_prev;
        Eigen::VectorXd y_prev;

        // Residuals (norm?)
        float x_res = 0.0; // primal
        float y_res = 0.0; // dual

};

// We'll want to store a vector of unique pointers to the solvers, rather than the actual solvers themselves
// using upDQPSolver = std::unique_ptr<DQPSolver>;
using GlobalIndexMap = std::unordered_map<int, std::vector<std::pair<int, int>>>;

class DistributedOptimizer
{
    public:
        DistributedOptimizer(const int &nSolvers, const Eigen::MatrixXd &_Q, const Eigen::MatrixXd &_A, const Eigen::VectorXd &_lower, const Eigen::VectorXd &_upper, const float _alpha, const float _rho, const float _mu);
        ~DistributedOptimizer();

        // Set up N solvers = (# segments - 2)
        void setup_solvers(int chunk_size, int chunk_stride);

        // update in parallel all local x, nu, z, and s
        // should just return primal?
        void parallel_update_primals();

        // full global update with dual (x, y, and w)
        void init_update_globals();

        // simplified global update (x and w only)
        void update_globals();

        // update in parallel all local y
        void parallel_update_duals();

        // threading handling
        // void setup_threadpool();
        // void join_threads();

        // Setters and getters
        Eigen::VectorXd getPrimalResiduals(); // Just a vector of x?
        Eigen::VectorXd getDualResiduals(); // just a vector of y?

        // Public member variables
        int m_nSolvers = 1; // default to a single solver (which should still work...)
        std::vector<DQPSolver> m_solvers;

        // Full problem data and constraints (initialized in init function)
        const Eigen::MatrixXd Q;
        const Eigen::MatrixXd A;

        // Not sure exactly the structure of these globally
        const Eigen::VectorXd lower;
        const Eigen::VectorXd upper;

        float alpha = 1.0; // step size
        float rho = 1.0; // penalty param: constant, not adapting this at the moment
        float mu = 1.0; // penalty param: constant, not adapting this at the moment

        Eigen::VectorXd w;
        GlobalIndexMap Gmap;

};


#endif // OPTIMIZER_HPP