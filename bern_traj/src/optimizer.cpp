#include "bern_traj/optimizer.hpp"
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

    for (int j = 0; j < local_views.size(); ++j) {
        const auto& view = local_views[j];
        for (int i = 0; i < view.length; ++i) {
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
    x_prev = Eigen::VectorXd::Zero(n);
    nu_prev = Eigen::VectorXd::Zero(m);
    z_prev = Eigen::VectorXd::Zero(m);
    s_prev = Eigen::VectorXd::Zero(m);
    w_tilde_prev = Eigen::VectorXd::Zero(n);
    lamb_prev = Eigen::VectorXd::Zero(m);
    y_prev = Eigen::VectorXd::Zero(n);

    // Precompute the lhs for linear solver
    lhs = Eigen::MatrixXd::Identity(n+m, n+m);
    lhs.topLeftCorner(n,n) = Q + mu * Eigen::MatrixXd::Identity(n,n);
    lhs.topRightCorner(n,m) = A.transpose();
    lhs.bottomLeftCorner(m,n) = A;
    lhs.bottomRightCorner(m,m) *= rhoinv;

    // Set up LDLT solver so we don't have to set it up every time
    ldlt_solver.compute(lhs);

    // Precompute a correctly sized rhs
    rhs = Eigen::VectorXd::Zero(n+m);
}

void DQPSolver::compute_KKT_rhs() {
    rhs(Eigen::seq(0,n)) = mu * w_tilde - y; // you tried calling a vector method on a matrix error here.... unsure why
    rhs(Eigen::seq(n,m)) = z - rhoinv * lamb;
}

void DQPSolver::solve_KKT() {
    // Start by computing rhs
    compute_KKT_rhs();

    // Solve this system and store in temp sln vector
    Eigen::VectorXd sln = ldlt_solver.solve(rhs);

    // x and nu update
    x = sln(Eigen::seq(0,n));
    nu = sln(Eigen::seq(n,m));
}

// Compute new rhs, solve KKT, set x and nu, update z, update s
void DQPSolver::update_primals() {
    solve_KKT(); //
    z = s + rhoinv * (nu - lamb); // Need to initialize prev values to zero I think?
    
    // Set the value, then clip using lower and upper bounds
    s_prev = s; // save old s
    s = alpha * z + (1 - alpha) * s + rhoinv * lamb;
    s = s.array().cwiseMin(upper).cwiseMax(lower); // going to need to figure this one out
}

void DQPSolver::update_global(const Eigen::VectorXd &w_new) {
    w_tilde_prev = w_tilde; // save old w_tilde
    w_tilde = w_new;
}

void DQPSolver::update_duals() {
    lamb = lamb + rho * (alpha * z + (1 - alpha)*s_prev - s);
    y = y + mu * (alpha * x + (1 - alpha)*w_tilde_prev - w_tilde);
}

// should return 0 if not yet terminated, or 1 if terminated
int DQPSolver::check_termination() { 
    return 0; 
} 

// Getters and setters
float DQPSolver::getPrimalResidual() {
    return 0.0;
}
float DQPSolver::getDualResidual() {
    return 0.0;
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

    // For each solver, initialize with the appropriate block of a given matrix (block diagonal)
    for (int i=0; i<m_nSolvers; ++i) {
        m_solvers[i] = DQPSolver();

        //Initialize with the specific block along diagonal relevant to this solver (lower and upper are already fixed as chunk_size for simplicity)
        // RH: TODO - FIX THE A BLOCK SETUP, SINCE IT SHOULD BE N rows by M cols!!!
        m_solvers[i].init(Q.block(i*chunk_stride, i*chunk_stride, chunk_size, chunk_size), A.block(i*chunk_stride, i*chunk_stride, chunk_size, chunk_size), lower, upper, alpha, rho, mu);

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
            numer += mu * m_solvers[p.first].x(p.second) + m_solvers[p.first].y(p.second);
            denom += mu;
        }

        w[l] = alpha * numer / denom + (1 - alpha) * w[l];

        numer = 0.0;
        denom = 0.0;
    }
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
            numer += mu * m_solvers[p.first].x(p.second);
            denom += mu;
        }

        w[l] = alpha * numer / denom + (1 - alpha) * w[l];

        numer = 0.0;
        denom = 0.0;
    }
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
        tmp[i] = m_solvers[i].getPrimalResidual();
    }
    return tmp;
}

