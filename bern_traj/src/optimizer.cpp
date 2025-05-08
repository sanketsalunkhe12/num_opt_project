#include "bern_traj/optimizer.hpp"
#include <iostream>
// ^Should include all relevant types I think?

// RH: ASSUMING ROW MAJOR MATRICES BUT SHOULD ALTER TO MATCH BERNSTEIN FILE!!!

// **********************************************************************************************
// **********************************************************************************************
struct LocalView {
    int start;
    int length;

    LocalView(int start, int length) : start(start), length(length) {}

};

GlobalIndexMap map_global_to_local_indices(
    int global_size,
    const std::vector<LocalView>& local_views
) {
    GlobalIndexMap index_map;

    // DEBUG
    // std::cout << "# OF W_l ENTRIES=" << global_size << std::endl;

    for (int j = 0; j < local_views.size(); ++j) {
        // std::cout <<  "j=" << j  << std::endl;

        const auto& view = local_views[j];
        for (int i = 0; i < view.length; ++i) {
            // std::cout << "i=" << i << std::endl;

            int global_index = view.start + i; //* view.stride;
            if (global_index >= 0 && global_index < global_size) {
                index_map[global_index].emplace_back(j, i);
            }
        }
    }

    // Gives us l -> vec(j,i)
    return index_map;
}

// **********************************************************************************************
// **********************************************************************************************
// ^ Courtesy of our friendly neighborhood Geppetto

// could potentially just have these be double typed...
//  template <typename DerivedA,typename DerivedB,typename Derivedc>
// void DQPSolver::init(Eigen::MatrixBase<DerivedA> &A, Eigen::MatrixBase<DerivedB> &B, Eigen::MatrixBase<Derivedc> &c, const float alpha, const float rho, const float mu) : A(A), B(B), c(c), alpha(alpha), rho(rho), mu(mu) {

DQPSolver::DQPSolver() {}

// Going with a non-templated version where we use Ref and double type matrices
void DQPSolver::init(const Eigen::MatrixXd &_Q, const Eigen::MatrixXd &_A, const Eigen::VectorXd &_lower, const Eigen::VectorXd &_upper, const float _alpha, const float _rho, const float _mu) {
    Q = _Q; 
    A=_A;
    lower=_lower; 
    upper=_upper;
    alpha=_alpha;
    rho =_rho; 
    mu = _mu;

    // cached variables
    rhoinv = 1.0 / rho; // 1/rho

    // Get correct dimensions
    m = A.rows();
    n = A.cols();

    // optimization variables (primals, duals)
    x = Eigen::VectorXd::Zero(n);
    nu = Eigen::VectorXd::Zero(m);
    z = Eigen::VectorXd::Zero(m);
    s = Eigen::VectorXd::Zero(m);
    w_tilde = Eigen::VectorXd::Zero(n); // basically the local copy of relevant global parameters G(w, i, j) -> {w_l}
    lamb = Eigen::VectorXd::Zero(m);
    y = Eigen::VectorXd::Zero(n);

    // For most of these vars, need a way to store the previous value for updates
    // x_prev = Eigen::VectorXd::Zero(n);
    //nu_prev = Eigen::VectorXd::Zero(m);
    //z_prev = Eigen::VectorXd::Zero(m);
    s_prev = Eigen::VectorXd::Zero(m);
    w_tilde_prev = Eigen::VectorXd::Zero(n);
    lamb_prev = Eigen::VectorXd::Zero(m);
    y_prev = Eigen::VectorXd::Zero(n);

    // Precompute the lhs for linear solver
    // std::cout << Q.rows() << " " << Q.cols() << std::endl;
    // std::cout << "n=" << n << " m=" << m << std::endl;

    lhs = Eigen::MatrixXd::Identity(n+m, n+m);
    lhs.topLeftCorner(n,n) = (Q + mu * Eigen::MatrixXd::Identity(n,n)).eval();
    lhs.topRightCorner(n,m) = A.transpose();
    lhs.bottomLeftCorner(m,n) = A;
    lhs.bottomRightCorner(m,m) *= rhoinv;

    // Set up LDLT solver so we don't have to set it up every time
    ldlt_solver.compute(lhs);
    if (ldlt_solver.info() != Eigen::Success) {
            throw std::runtime_error("KKT factorization failed during DQPSolver initialization.");
    }

    // Precompute a correctly sized rhs
    rhs = Eigen::VectorXd::Zero(n+m);
}

void DQPSolver::compute_KKT_rhs() {
    auto expr1 = mu * w_tilde - y;
    auto expr2 = z - rhoinv * lamb;

    // rhs is a column vector
    rhs(Eigen::seqN(0,n)) = expr1.eval();
    rhs(Eigen::seqN(n,m)) = expr2.eval();

    // std::cout << rhs.size() << std::endl << rhs << std::endl; // Correct length n+m!
}

void DQPSolver::solve_KKT() {
    // Start by computing rhs
    compute_KKT_rhs();

    // Solve this system and store in temp sln vector
    Eigen::VectorXd sln = ldlt_solver.solve(rhs);

    // x and nu update
    x = sln(Eigen::seqN(0,n)); // x should be length n
    nu = sln(Eigen::seqN(n,m)); // Nu should be length m

    // Check sizes
    // std::cout << x.size() << std::endl;
    // std::cout << nu.size() << std::endl;    
}

// Compute new rhs, solve KKT, set x and nu, update z, update s
void DQPSolver::update_primals() {
    solve_KKT(); //

    // std::cout << "Acols=n=" << n << " Arows=m=" << m << std::endl;
    // std::cout << z.size() << std::endl;
    // std::cout << s.size() << std::endl;
    // std::cout << nu.size() << std::endl;
    // std::cout << lamb.size() << std::endl;

    // save z
    z_prev = z;
    z = s + rhoinv * (nu - lamb);
    
    // Set the value, then clip using lower and upper bounds
    s_prev = s; // save old s
    s = alpha * z + (1 - alpha) * s + rhoinv * lamb;
    s = s.cwiseMin(upper).cwiseMax(lower);
}

void DQPSolver::update_global(const Eigen::VectorXd &w_new) {
    w_tilde_prev = w_tilde; // save old w_tilde
    w_tilde = w_new;
}

void DQPSolver::update_duals() {
    // lamb = (lamb + rho * (alpha * z + (1 - alpha)*s_prev - s)).eval();
    // y = (y + mu * (alpha * x + (1 - alpha)*w_tilde_prev - w_tilde)).eval();
    lamb = lamb + rho * (alpha * z + (1 - alpha)*s_prev - s);
    y = y + mu * (alpha * x + (1 - alpha)*w_tilde_prev - w_tilde);
}

// should return 0 if not yet terminated, or 1 if terminated
int DQPSolver::check_termination() { 
    return 0; 
} 

// Getters and setters
float DQPSolver::getPrimalResidual() {
    return (A*x - z).norm();
}
float DQPSolver::getDualResidual() {
    return (-rho * A.transpose() * (z - z_prev)).norm();
}

// ************************************************************************************
// High-Level DQP Controller
DistributedOptimizer::DistributedOptimizer(const int &nSolvers, const Eigen::MatrixXd &Q, const Eigen::MatrixXd &A, const Eigen::VectorXd &lower, const Eigen::VectorXd &upper, const float alpha, const float rho, const float mu) :  m_nSolvers(nSolvers), Q(Q), A(A), lower(lower), upper(upper), alpha(alpha), rho(rho), mu(mu) {
    // Initialize vector of pointers to un-setup DQPSolvers
    m_solvers.resize(m_nSolvers);
    w = Eigen::VectorXd::Zero(A.cols()); // Init global vector of length n
}

DistributedOptimizer::~DistributedOptimizer()
{

}

// Set up N solvers = (# segments - 2), so chunk_size = 3 * 12 in our case for Q, unsure on A since it should be rectangular...
void DistributedOptimizer::setup_solvers(int chunk_size, int chunk_stride) {
    // create empty vector for LocalView structs with correct chunk_size and start index
    std::vector<LocalView> lv;

    // ADD VALIDATION!!! Check the last index to see if larger than (size of w - 1)
    int tmp = ((m_nSolvers-1) * chunk_stride + (chunk_size-1));
    if (tmp > (w.size()-1)) {
        std::cout << "[ERROR] Max Index = " << tmp << std::endl;
        throw std::runtime_error("[ERROR] The chunk size and chunk stride supplied, combined with the number of local solvers, is larger than the size of the global parameter vector 'w'.");
    }

    // For each solver, initialize with the appropriate block of a given matrix (block diagonal)
    for (int i=0; i<m_nSolvers; ++i) {
        m_solvers[i] = DQPSolver();

        //Initialize with the specific block along diagonal relevant to this solver (lower and upper are already fixed as chunk_size for simplicity)
        // REMEMBER: THESE ARE COLUMN MAJOR!!!!!!
        auto qi = Q.block(i*chunk_stride, i*chunk_stride, chunk_size, chunk_size);
        auto ai = A.block(0, i*chunk_stride, A.rows(), chunk_size); // select vertical m x ni matrices from A
        m_solvers[i].init(qi, ai, lower, upper, alpha, rho, mu);

	// Initialize primal to the correct view of w
	m_solvers[i].w_tilde = w(Eigen::seqN(i*chunk_stride, chunk_size));
        m_solvers[i].x = w(Eigen::seqN(i*chunk_stride, chunk_size));
        m_solvers[i].z = w(Eigen::seqN(i*chunk_stride, chunk_size));
        m_solvers[i].s = w(Eigen::seqN(i*chunk_stride, chunk_size));

        // Set local view up
        lv.emplace_back(i*chunk_stride, chunk_size);
    }
    // Finally pass vector of local views to global to local map function
    Gmap = map_global_to_local_indices(Q.rows(), lv);
}

// update in parallel all local x, nu, z, and s
void DistributedOptimizer::parallel_update_primals() {
    tbb::parallel_for(tbb::blocked_range<int>(0, m_nSolvers), [&](const tbb::blocked_range<int>& r) {
        for (int i = r.begin(); i < r.end(); ++i) {
            // Update lamb and y for each solver
            m_solvers[i].update_primals();
        }
    });
}

// full global update with dual (x, y, and w)
void DistributedOptimizer::init_update_globals() {
    // For each element l of w
    // Extract values {x_i} and {y_i} from each solver j for all i and all j

    float numer = 0.0;
    float denom = 0.0; // okay because this will never be zero. Every w_l has at least one local var
    std::pair<int, int> p;
    for (int l=0; l<w.size(); ++l) {
        for (auto p : Gmap[l]) {
            // DEBUG FOR THE LOVE OF THE GAME
            // std::cout << "GMAP TRAVERSE: " << p.first << " " << p.second << std::endl;

            numer += mu * m_solvers[p.first].x(p.second) + m_solvers[p.first].y(p.second);
            denom += mu;
        }

        w[l] = alpha * numer / denom + (1 - alpha) * w[l];

        numer = 0.0;
        denom = 0.0;
    }
    
    // Pass ONLY VIEW OF w vector into each local solver!!!!
    // for (int s=0; s<m_nSolvers; ++s) {
    //     m_solvers[s].update_global(w);
    // }
}

// simplified global update (x and w only)
void DistributedOptimizer::update_globals() {
    // For each element l of w
    // Extract values {x_i} from each solver j for all i and all j

    float numer = 0.0;
    float denom = 0.0; // okay because this will never be zero. Every w_l has at least one local var
    std::pair<int, int> p;
    for (int l=0; l<w.size(); ++l) {
        for (auto p : Gmap[l]) {
            // DEBUG FOR THE LOVE OF THE GAME
            // std::cout << "GMAP TRAVERSE: " << p.first << p.second << std::endl;

            // This sums mu * solver_j.x_i for all relevant j and i in global to local (l -> {j,i}) mapping
            numer += mu * m_solvers[p.first].x(p.second);
            denom += mu;
        }

        // Slight possibility that denom is actually zero because of incorrect mapping. Should NOT update w_l in that case
        if (std::fabs(denom) > std::numeric_limits<float>::epsilon()) {
            w[l] = alpha * numer / denom + (1 - alpha) * w[l];
        }

        numer = 0.0;
        denom = 0.0;
    }

    // Pass ONLY VIEW OF w vector into each local solver!!!!
    // for (int s=0; s<m_nSolvers; ++s) {
    //     m_solvers[s].update_global(w);
    // }
}

// update in parallel all local y
void DistributedOptimizer::parallel_update_duals() {
    tbb::parallel_for(tbb::blocked_range<int>(0, m_nSolvers), [&](const tbb::blocked_range<int>& r) {
        for (int i = r.begin(); i < r.end(); ++i) {
            // Update lamb and y for each solver
            m_solvers[i].update_duals();
        }
    });
}

// Setters and getters

// Base residual calculation on OSQP?
// This should return norm of x-primal residual
Eigen::VectorXd DistributedOptimizer::getPrimalResiduals() {
    Eigen::VectorXd tmp(m_nSolvers);
    for (int i=0; i<m_nSolvers; ++i) {
        tmp[i] = m_solvers[i].getPrimalResidual();
    }
    return tmp;
} 

// This should return norm of y-dual residual
Eigen::VectorXd DistributedOptimizer::getDualResiduals() {
    Eigen::VectorXd tmp(m_nSolvers);
    for (int i=0; i<m_nSolvers; ++i) {
        tmp[i] = m_solvers[i].getDualResidual();
    }
    return tmp;
}

int main(int argc, char* argv[]) {
    std::cout << "This is a basic example showing the syntax for this DQP C-ADMM solver: " << std::endl;

    // Simple test with basic quadratic objective function (just make a gaussian PSD for Q) and constraints which are just an Identity matrix with some noise added?
    Eigen::MatrixXd rand = Eigen::MatrixXd::Random(36,36);
    Eigen::MatrixXd Q = (10*rand.transpose() * 10*rand).eval();
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(36,36);
    A(25,25) = 10.;
    A(15,25) = 0.1432432;
    A(5,10) = 5.;
    A(27,18) = 2.;
    A(29,13) = 0.5;
    Eigen::VectorXd u = (Eigen::VectorXd::Random(36) * 3).cwiseAbs();
    Eigen::VectorXd l = -u;

    float alpha = 1.5; 
    float rho = 0.3;
    float mu = 0.3;

    // Print all these out?
    // std::cout << Q << std::endl;
    // std::cout << A << std::endl;
    // std::cout << u << std::endl;
    // std::cout << l << std::endl;

    // Construct a Distributed Optimizer and then run it
    DistributedOptimizer dqpopt = DistributedOptimizer(std::atoi(argv[1]), Q, A, l, u, alpha, rho, mu);

    // Read the command line args (REQUIRED NOW!) and pass to solver setup
    dqpopt.setup_solvers(std::atoi(argv[2]), std::atoi(argv[3]));

    // Init
    dqpopt.parallel_update_primals();
    dqpopt.init_update_globals();
    dqpopt.parallel_update_duals();

    std::cout << "Iteration 0 : " << dqpopt.w << std::endl;

   int max_iter = 10;
   
   // Loop C-ADMM
   for (int i=0; i<max_iter; ++i) {
       dqpopt.parallel_update_primals();
       dqpopt.update_globals();
       dqpopt.parallel_update_duals();
   
       std::cout << "Iteration " << i+1 << " : " << dqpopt.w << std::endl;
   }

}
