#include "bern_traj/bernstein_trajectory.hpp"
#include "osqp/osqp.h"
#include <Eigen/Sparse>

BernsteinTrajectory::BernsteinTrajectory(TrajectoryParams &bernstein_params_)
{
    isStarted = false;
    isCompleted = false;

    minVel = bernstein_params_.minVel;
    maxVel = bernstein_params_.maxVel;

    minAcc = bernstein_params_.minAcc;
    maxAcc = bernstein_params_.maxAcc;

    controlPtCount = bernstein_params_.controlPtCount;
    minDerivative = bernstein_params_.minDerivative;
    trajDimension = bernstein_params_.trajDimension; // (x, y, z)

    magicFabianConstant = bernstein_params_.magicFabianConstant;
    timeFactor = bernstein_params_.timeFactor;

    segIdx = 0;
    replan = false;

    // waypointCount = bernstein_params_.waypoints.size();
}

BernsteinTrajectory::~BernsteinTrajectory()
{

}

bool BernsteinTrajectory::initialize(rclcpp::Node::SharedPtr node_ptr, const std::vector<Waypoint> *goal_wp, 
                                    const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
    // timeFactor = node_ptr->declare_parameter("time_factor", 1.0);
    // magicFabianConstant = node_ptr->declare_parameter("magic_fabian_constant", 6.0);

    // node_ptr->get_parameter("time_factor", timeFactor);
    // node_ptr->get_parameter("magic_fabian_constant", magicFabianConstant);

    waypointCount = goal_wp->size();
    
    if(waypointCount < 2)
    {
        RCLCPP_ERROR(node_ptr->get_logger(), "BernsteinTrajectory: Not enough waypoints");
        return false;
    }

    segmentCount = waypointCount - 1;
    bernCoeffComb = Eigen::VectorXd::Zero(segmentCount * trajDimension * controlPtCount);
    
    bool sucess = solveOptimizedTraj(node_ptr, goal_wp);
    if(!sucess)
    {
        RCLCPP_ERROR(node_ptr->get_logger(), "BernsteinTrajectory: Failed to solve optimized trajectory");
        return false;
    }

    return true;
}

bool BernsteinTrajectory::generateTrajectory(const Eigen::Vector3f &xi, const Eigen::Vector3f &xf,
                                const Eigen::Vector3f &vi, const Eigen::Vector3f &vf,
                                const Eigen::Vector3f &ai, const Eigen::Vector3f &af, 
                                const float &yaw_i, const float &yaw_f, 
                                const float &yaw_dot_i, const float &yaw_dot_f,
                                float dt)
{
    return true;
}

uav_msgs::msg::PositionCmd::SharedPtr BernsteinTrajectory::update(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
    auto position_cmd = std::make_shared<uav_msgs::msg::PositionCmd>();
    
    return position_cmd;
}

bool BernsteinTrajectory::deactivate()
{
    return true;
}

Eigen::Vector3d BernsteinTrajectory::getRefPosition(double &time_)
{
    return Eigen::Vector3d::Zero();
}

Eigen::Vector3d BernsteinTrajectory::getRefVelocity(double &time_)
{
    return Eigen::Vector3d::Zero();
}

Eigen::Vector3d BernsteinTrajectory::getRefAcceleration(double &time_)
{
    return Eigen::Vector3d::Zero();
}

Eigen::Vector3d BernsteinTrajectory::getRefJerk(double &time_)
{
    return Eigen::Vector3d::Zero();
}

// Bernstein functions
bool BernsteinTrajectory::solveOptimizedTraj(rclcpp::Node::SharedPtr node_ptr, const std::vector<Waypoint> *goal_wp)
{
    // calculate segment time
    generateSegmentTime(goal_wp);
    
    // generating objective function
    Eigen::MatrixXd Q = generateObjectiveFunction();
    Eigen::MatrixXd Q_comb = Eigen::MatrixXd::Zero(bernCoeffComb.size(), bernCoeffComb.size());

    // generate eq and ineq constraints for each dimension and combine into one constraint matrix
    QPEqConstraints eq_constraints_comb;
    QPIneqConstraints ineq_constraints_comb;
    
    int num_constraint_comb = controlPtCount * trajDimension * segmentCount;

    eq_constraints_comb.A = Eigen::MatrixXd::Zero(num_constraint_comb, bernCoeffComb.size());
    eq_constraints_comb.b = Eigen::VectorXd::Zero(num_constraint_comb);

    int num_ineq_constraint = 0;
    int deriv_order = 2; // applying inequality only on vel and acc
   
    for(int i=0; i<segmentCount; i++)
    {
        for(int j=1; j<=deriv_order; j++)
        {
            num_ineq_constraint += controlPtCount - j;
        }
    }

    int num_ineq_constraint_comb = num_ineq_constraint * trajDimension; 

    ineq_constraints_comb.C = Eigen::MatrixXd::Zero(num_ineq_constraint_comb, bernCoeffComb.size());
    ineq_constraints_comb.d = Eigen::VectorXd::Zero(num_ineq_constraint_comb);
    ineq_constraints_comb.f = Eigen::VectorXd::Zero(num_ineq_constraint_comb);

    for(int dim=0; dim<trajDimension; dim++)
    {
        // for each dimension, generate eq and ineq constraints
        QPEqConstraints eq_constraints = generateEqConstraint(dim, goal_wp);
        QPIneqConstraints ineq_constraints = generateIneqConstraint(dim, goal_wp);

        // combine eq and ineq constraints into one big constraint matrix
        eq_constraints_comb.A.block(dim*controlPtCount*segmentCount, dim*controlPtCount*segmentCount, 
                        controlPtCount*segmentCount, controlPtCount*segmentCount) = eq_constraints.A;

        eq_constraints_comb.b.block(dim*controlPtCount*segmentCount, 0, 
                        controlPtCount*segmentCount, 1) = eq_constraints.b;

        ineq_constraints_comb.C.block(dim*num_ineq_constraint, dim*controlPtCount*segmentCount, 
                        num_ineq_constraint, controlPtCount*segmentCount) = ineq_constraints.C;

        ineq_constraints_comb.d.block(dim*num_ineq_constraint, 0,
                        num_ineq_constraint, 1) = ineq_constraints.d;

        ineq_constraints_comb.f.block(dim*num_ineq_constraint, 0,
                        num_ineq_constraint, 1) = ineq_constraints.f;

        // combined objective function
        Q_comb.block(dim*controlPtCount*segmentCount, dim*controlPtCount*segmentCount, 
                        controlPtCount*segmentCount, controlPtCount*segmentCount) = Q;
    }

    // if any additional constraints are needed, add them here

    // solving combined OSQP problem
    bool success = combOSQPSolver(Q_comb, eq_constraints_comb, ineq_constraints_comb, node_ptr);
    if(success)
    {
        for(int dim=0; dim<trajDimension; dim++)
        {
            bernCoeff.col(dim) = bernCoeffComb.block(dim*controlPtCount*segmentCount, 0, 
                                                     controlPtCount*segmentCount, 1);
        }
    }
    else
    {
        return false;
    }

    // solving individual OSQP problem threading

    return success;
}

void BernsteinTrajectory::generateSegmentTime(const std::vector<Waypoint> *goal_wp)
{
    double a_max = maxAcc[0];
    double v_max = maxVel[0];
    trajectory_duration = 0.0;

    for(int i=0; i<segmentCount; i++)
    {
        Waypoint wp0 = (*goal_wp)[i];
        Waypoint wp1 = (*goal_wp)[i+1];

        double dist = (wp1.position - wp0.position).norm();

        double time = timeFactor * dist / v_max * 2 * (1.0 + magicFabianConstant * v_max / a_max * exp(-dist / v_max * 2)); 

        if (time < 1e-6) time = 0.1;

        segmentTime.push_back(time);
        trajectory_duration += time;
    }
}

Eigen::MatrixXd BernsteinTrajectory::generateObjectiveFunction()
{
    /*
    It generated the objective function Q for single dimension and for all segments combined
    In current situation, we have 3 dimensions (x, y, z) and Q is same for all dimensions.
    
    Q_comb = [Q, 0, 0; 
              0, Q, 0;   
              0, 0, Q] 
    
    Output: Q=diag(Q1, Q2, ..., QM) where M = segCnt
            Q \in R^{nxn} n = Pi x segCnt
    */
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(controlPtCount * segmentCount, controlPtCount * segmentCount); 

    for(int i=0; i<segmentCount; i++)
    {
        // generating Q for each segment and then combining them
        Eigen::MatrixXd q = generateQMatrix(segmentTime[i]);
        Q.block(i*controlPtCount, i*controlPtCount, controlPtCount, controlPtCount) = q;
    }

    return Q;
}

Eigen::MatrixXd BernsteinTrajectory::generateQMatrix(double &time_)
{
    /*
        Q = Dm^T * q * Dm
        Here we have controlPtCount = n+1
        q = 1/(2*(n-m)-1) * [ncr(n-m, i) * ncr(n-m, j) / ncr(2*(n-m), i+j)]
            where factor = 1/(2*(n-m)-1)
        
        q \in R^{n-m+1 x n-m+1}
        Dm \in R^{n-m+1 x n+1}
    */
    
    Eigen::MatrixXd q = Eigen::MatrixXd::Zero(controlPtCount-minDerivative, controlPtCount-minDerivative); 
    Eigen::MatrixXd Dm = generateDerivativeMatrix(minDerivative, time_);
    
    double factor = 1.0/(2*(controlPtCount-minDerivative)); 
    int bigN = 2*(controlPtCount-minDerivative);
    int smallN = controlPtCount-minDerivative;

    for(int i=0; i<controlPtCount-minDerivative; i++)
    {
        for(int j=0; j<controlPtCount-minDerivative; j++)
        {
            // q(i, j) = (nCr(smallN, i) * nCr(smallN, j)) / (nCr(bigN, (i + j)));
            q(i, j) = 1 / (nCr(bigN, (i + j)) / (nCr(smallN, i) * nCr(smallN, j)));
        }
    }
    
    return factor * Dm * q * Dm.transpose(); // already Dm is in transpose   
}

Eigen::MatrixXd BernsteinTrajectory::generateDerivativeMatrix(int &minDerivative_, double &time_)
{
    /*
        Dm = [s^m, 0, 0, ..., 0;
              0, s^m, 0, ..., 0;
              0, 0, s^m, ..., 0;
              ...
              0, 0, 0, ..., s^m]
        Dm \in R^{n-m+1 x n+1}

        conv_filt stores the convolution result of s*m
    */

    
    if (minDerivative_ == 0) 
    {
        return Eigen::MatrixXd::Identity(controlPtCount, controlPtCount);
    }
    
    Eigen::MatrixXd Dm = Eigen::MatrixXd::Zero(controlPtCount-minDerivative_, controlPtCount); //Dm transpose
    Eigen::VectorXd conv_filt = Eigen::VectorXd::Zero(controlPtCount); //s^[m]=[-1,1]

    conv_filt.head(minDerivative_ + 1) = getBernsteinBasis(time_, minDerivative_);
    // conv_filt = s*s*s*...*s (m times convolution, m = minDerivative)

    for(int i=0; i<controlPtCount-minDerivative_; i++)
    {
        int q = controlPtCount;
        if(i + minDerivative_ + 1 < controlPtCount)
        {
            q = i + minDerivative_;
        }
        for(int j=i; j<q; j++)
        {
            Dm(j,i) = conv_filt(j-i);
        }
    }
    
    return Dm;
    // return Dm.transpose();
}

Eigen::VectorXd BernsteinTrajectory::getBernsteinBasis(double &time_, int &minDerivative_)
{
    /*
        Generates the convolution kernel s*m
        factor = (n!)/ [(n-m)! * (time)^m]
    */

    Eigen::VectorXd row; // s for convolution

    // m=0 and s(0) = 1
    if (minDerivative_ == 0) 
    {
        row = Eigen::VectorXd::Zero(1);
        row(0) = 1;
        return row;
    }

    // m=1 and s(1) = [-1, 1]
    row = Eigen::VectorXd::Zero(minDerivative_ + 1);
    row(0) = -1;
    row(1) = 1;

    double factor = controlPtCount / time_;

    for(int i=1; i<minDerivative_; i++)
    {
        Eigen::VectorXd diff = row.tail(minDerivative_) - row.head(minDerivative_);
        row.tail(minDerivative_) = diff;
        factor = factor * (controlPtCount - i) / time_;
    }

    if ((minDerivative_ > 1) && (minDerivative_ % 2 == 0)) 
    {
        factor = factor * -1;
    }

    return factor * row;
}

double BernsteinTrajectory::nCr(int n, int r)
{
    /*
        nCr = n! / (r! * (n-r)!)
        p holds the value of n*(n-1)*(n-2)...,
        k holds the value of r*(r-1)*(r-2)...,
    */
    double p = 1.0;
    
    // C(n, r) == C(n, n-r)
    if (n - r < r) r = n - r; // choosing the smaller value

    for (int i = 1; i <= r; i++)
    {
        double numer = (n - r + i);
        double denom = i;
        p = p * numer / denom;
    }
    
    return p;
}


// constraints generation
QPEqConstraints BernsteinTrajectory::generateEqConstraint(int &dimension_, const std::vector<Waypoint> *goal_wp)
{
    /*
        Ax = b
        Genenerate equality constraints for each dimension A \in R(m*n)x(m*n)
        Why? 1st traj endpoint constraints cover 0th and nth bernstein coeff and 
        2nd segment intersection constraints cover (1st to n-1th) bernstein coeff
        thus num_eq_constraint = controlPtCount * segmentCount
        It applies at specific time t_i=0 and t_i=T
    */
    int num_eq_constraint = controlPtCount * segmentCount; // why? ↑
    Eigen::VectorXd bernBasis;

    QPEqConstraints eq_constraints;
    eq_constraints.A = Eigen::MatrixXd::Zero(num_eq_constraint, controlPtCount*segmentCount);

    // (1) trajectory endpoint constraints (start and end) for position, velocity, acceleration
    
    int constraintIndex = 0;
    for(int i=0; i<4; i++) // 4 = pos, vel, acc, jerk
    {
        // start point
        bernBasis = getBernsteinBasis(segmentTime[0], i);
        eq_constraints.A.block(constraintIndex, 0, 1, i+1) = bernBasis.transpose(); 
        eq_constraints.b(constraintIndex) = (*goal_wp)[0].getConstraint(i, dimension_);
        constraintIndex += 1; 

        // end point
        bernBasis = getBernsteinBasis(segmentTime[segmentCount-1], i);
        eq_constraints.A.block(constraintIndex, controlPtCount*segmentCount-1-i, 1, i+1) = bernBasis.transpose();
        eq_constraints.b(constraintIndex) = (*goal_wp)[waypointCount-1].getConstraint(i, dimension_);
        constraintIndex += 1;
    }


    // (2) segment intersection continuity constraints for position, velocity, acceleration 

    for(int i=1; i<(waypointCount-1); i++)
    {
        // set end position = wp i+1 (only need from 1 to n-1 wp. start and end are already set above)
        int j = 0;
        bernBasis = getBernsteinBasis(segmentTime[i-1], j);
        eq_constraints.A.block(constraintIndex, (i*controlPtCount)-j-1, 1, j+1) = bernBasis.transpose();
        eq_constraints.b(constraintIndex) = (*goal_wp)[i].getConstraint(j, dimension_);
        constraintIndex += 1;
        
        // enforce continuity of position, velocity, acceleration, jerk
        for(int j=0; j<4; j++) // 3 = pos, vel, acc, jerk
        {
            // previous segment
            bernBasis = getBernsteinBasis(segmentTime[i-1], j);
            eq_constraints.A.block(constraintIndex, (i*controlPtCount)-j-1, 1, j+1) = bernBasis.transpose();

            // next segment
            bernBasis = getBernsteinBasis(segmentTime[i], j);
            eq_constraints.A.block(constraintIndex, i*controlPtCount, 1, j+1) = -1.0 * bernBasis.transpose();
        
            eq_constraints.b(constraintIndex) = 0.0; 
            constraintIndex += 1;
        }
    }

    // additional constraints like constant z etc can be added here as eq_constraints
    return eq_constraints;
}

QPIneqConstraints BernsteinTrajectory::generateIneqConstraint(int &dimension_, const std::vector<Waypoint> *goal_wp)
{
    /*
        d <= Cx <= f
        Generate ineq constraints for each dimension C \in R(num_ineq_constraint x m*n)
        need to know the total number of ineq constraints for given dimension
        For each WP i.e segment, we can provide ineq constraints over position, vel, acc, and jerk
        It applies for whole time duration of segment (t)
    */

    int num_ineq_constraint = 0;
    int deriv_order = 2; // inequality on vel and acc
    QPIneqConstraints ineq_constraints;

    for(int i=0; i<segmentCount; i++)
    {
        for(int j=1; j<=deriv_order; j++)
        {
            num_ineq_constraint += controlPtCount - j;
        }
    }

    if(num_ineq_constraint == 0)
    {
        ineq_constraints.C = Eigen::MatrixXd::Zero(1, controlPtCount*segmentCount);
        ineq_constraints.d = Eigen::VectorXd::Zero(1);
        ineq_constraints.f = Eigen::VectorXd::Zero(1);

        ineq_constraints.d(0) = -0.1; // inserting non zero values for dense matrix conversion
        ineq_constraints.f(0) = 0.1;
        return ineq_constraints;
    }

    ineq_constraints.C = Eigen::MatrixXd::Zero(num_ineq_constraint, controlPtCount*segmentCount);
    ineq_constraints.d = Eigen::VectorXd::Zero(num_ineq_constraint);
    ineq_constraints.f = Eigen::VectorXd::Zero(num_ineq_constraint);
    int constraintIndex = 0;

    // (1) generating inequality for each segment on vel and acc
    for(int i=0; i<segmentCount; i++)
    {
        for(int j=1; j<=deriv_order; j++)
        {
            ineq_constraints.C.block(constraintIndex, i*controlPtCount, controlPtCount-j, controlPtCount) = 
                            generateDerivativeMatrix(j, segmentTime[i]);
            
            float constraint_value; // = (*goal_wp)[i+1].getConstraint(j, dimension_);
            if(j == 1)
                constraint_value = maxVel[dimension_];
            else if(j == 2)
                constraint_value = maxAcc[dimension_];

            ineq_constraints.d.block(constraintIndex, 0, controlPtCount-j, 1) = 
                            Eigen::VectorXd::Constant(controlPtCount-j, -1.0 * constraint_value);
            ineq_constraints.f.block(constraintIndex, 0, controlPtCount-j, 1) =
                            Eigen::VectorXd::Constant(controlPtCount-j, constraint_value);

            constraintIndex += controlPtCount - j;
        }
    }

    // (2) additional constraints like constant z etc can be added here as ineq_constraints
    return ineq_constraints;
}


// solving OSQP problem
bool BernsteinTrajectory::threadOSQPSolver(Eigen::MatrixXd &Q, QPEqConstraints &eq_constraints, 
                                            QPIneqConstraints &ineq_constraints, rclcpp::Node::SharedPtr node_ptr)
{
    return true;
}

bool BernsteinTrajectory::combOSQPSolver(Eigen::MatrixXd &Q_comb, QPEqConstraints &eq_constraints_comb, 
                                        QPIneqConstraints &ineq_constraints_comb, rclcpp::Node::SharedPtr node_ptr)
{
    /*
        Standad QP problem for OSQP solver:
        min 1/2 x^T * P * x + q^T * x
        st l <= Ax <= u
        where P is a positive semidefinite matrix
        A is a matrix of constraints (both eq and ineq combined)

        CSC format for OSQP:
        p_x : values of non-zero elements in P
        p_i : row index of non-zero elements in P
        p_p : column pointer of non-zero elements in P

        warm start if we have some initial guess for the solution like previous solution
        useful in case of replanning or updating the trajectory
    */
    
    // combine all eq and ineq constraints
    Eigen::MatrixXd combConstraints = Eigen::MatrixXd::Zero(eq_constraints_comb.A.rows() + ineq_constraints_comb.C.rows(), 
                                                            bernCoeffComb.size());
    combConstraints.block(0, 0, eq_constraints_comb.A.rows(), bernCoeffComb.size()) = eq_constraints_comb.A;
    combConstraints.block(eq_constraints_comb.A.rows(), 0, ineq_constraints_comb.C.rows(), bernCoeffComb.size()) = ineq_constraints_comb.C;

    OSQPInt m = combConstraints.rows(); 
    OSQPFloat l[m]; // lower bound of constraints
    OSQPFloat u[m]; // upper bound of constraints

    for(int i=0; i<eq_constraints_comb.A.rows(); i++) // eq constraints
    {
        l[i] = eq_constraints_comb.b(i);
        u[i] = eq_constraints_comb.b(i);
    }

    for(int i=eq_constraints_comb.A.rows(); i<ineq_constraints_comb.C.rows()+eq_constraints_comb.A.rows(); i++) // ineq constraints
    {
        l[i] = ineq_constraints_comb.d(i-eq_constraints_comb.A.rows());
        u[i] = ineq_constraints_comb.f(i-eq_constraints_comb.A.rows());
    }
    
    // linear cost term q^T * x         in our case there is no linear term so set it to zero
    OSQPFloat q[bernCoeffComb.size()];
    for(int i=0; i<bernCoeffComb.size(); i++)
    {
        q[i] = 0.0;
    }
    
    // convert Q_comb, combCconstraint to sparse matrix 
    Eigen::SparseMatrix<double, Eigen::ColMajor> P_sparse = Eigen::MatrixXd(Q_comb.triangularView<Eigen::Upper>()).sparseView(1e-10);
    Eigen::SparseMatrix<double, Eigen::ColMajor> A_sparse = combConstraints.sparseView(1e-10);

    // convert objective matrix P_sparse into CSC (compressed sparse column) format for OSQP
    OSQPInt P_nnz = P_sparse.nonZeros(); // number of non-zero elements in P_sparse
    OSQPFloat P_x[P_nnz]; // values of non-zero elements in P_sparse
    OSQPInt P_i[P_nnz]; // row index of non-zero elements in P_sparse
    OSQPInt P_p[P_sparse.outerSize() + 1]; // column pointer of non-zero elements in P_sparse

    P_p[0] = 0.0;
    int count = 0;

    for(int k=0; k<P_sparse.outerSize(); ++k)
    {
        for (Eigen::SparseMatrix<double, Eigen::ColMajor>::InnerIterator it(P_sparse, k); it; ++it) 
        {
            P_x[count] = (OSQPFloat)it.value();
            P_i[count] = (OSQPInt)it.index();
            count += 1;
        }
        P_p[k + 1] = count;
    }

    // convert constraint matrix A_sparse into CSC (compressed sparse column) format for OSQP
    OSQPInt A_nnz = A_sparse.nonZeros(); 
    OSQPFloat A_x[A_nnz];
    OSQPInt A_i[A_nnz];
    OSQPInt A_p[A_sparse.outerSize() + 1];

    A_p[0] = 0.0;
    count = 0;

    for(int k=0; k<A_sparse.outerSize(); ++k)
    {
        for (Eigen::SparseMatrix<double, Eigen::ColMajor>::InnerIterator it(A_sparse, k); it; ++it) 
        {
            A_x[count] = (OSQPFloat)it.value();
            A_i[count] = (OSQPInt)it.index();
            count += 1;
        }
        A_p[k + 1] = count;
    }

    // define OSQP object for Solver, settings, matrices (P=Q)
    OSQPInt exitflag = 0;
    OSQPInt n = bernCoeffComb.size();

    OSQPSolver *solver;
    OSQPSettings *settings;  
    OSQPCscMatrix *primal_sol = (OSQPCscMatrix *)malloc(sizeof(OSQPCscMatrix));
    OSQPCscMatrix *dual_sol = (OSQPCscMatrix *)malloc(sizeof(OSQPCscMatrix));

    csc_set_data(primal_sol, n, n, P_nnz, P_x, P_i, P_p);
    csc_set_data(dual_sol, m, n, A_nnz, A_x, A_i, A_p);

    // Define solver settings as default
    settings = (OSQPSettings *)malloc(sizeof(OSQPSettings));
    if(settings)
    {
        osqp_set_default_settings(settings);
        settings->alpha = 1.0;  // Change alpha parameter
        settings->verbose = false;
        // settings->adaptive_rho=false;
    }

    // Setup workspace
    exitflag = osqp_setup(&solver, primal_sol, q, dual_sol, l, u, m, n, settings);
    
    auto start = std::chrono::high_resolution_clock::now();

    exitflag = osqp_solve(solver);

    if(exitflag != 0)
    {
        RCLCPP_ERROR(node_ptr->get_logger(), "Combined OSQP solver failed with exitflag: %d", exitflag);
        return false;
    }
    else if(solver->info->status_val == OSQP_SOLVED)
    {
        RCLCPP_INFO(node_ptr->get_logger(), "Combined OSQP solver solved successfully");
        OSQPFloat *x = solver->solution->x;
        for(int i=0; i<n; i++)
        {
            bernCoeffComb(i) = x[i];
            osqp_cleanup(solver);
            return true;
        }
    }
    else if(solver->info->status_val == OSQP_PRIMAL_INFEASIBLE) 
    {
        RCLCPP_ERROR(node_ptr->get_logger(), "Combined OSQP problem is primal infeasible");
        return false;
    }
    else if(solver->info->status_val == OSQP_DUAL_INFEASIBLE) 
    {
        RCLCPP_ERROR(node_ptr->get_logger(), "Combined OSQP problem is dual infeasible");
        return false;
    }
    else
    {
        RCLCPP_ERROR(node_ptr->get_logger(), "Combined OSQP solver failed with status: %d", solver->info->status_val);
        return false;
    }
}



Eigen::MatrixXd BernsteinTrajectory::getBernCoefficients()
{
    return Eigen::MatrixXd();
}


double BernsteinTrajectory::getBernstein(int n, int r, double time)
{
    return 0.0;
}

Eigen::VectorXd BernsteinTrajectory::getConvolutionVector()
{
    return Eigen::VectorXd();
}


double BernsteinTrajectory::getSegmentTime(int segIdx_)
{
    return 0.0;
}

void BernsteinTrajectory::calculateTrajectory()
{

}

Eigen::Vector3d BernsteinTrajectory::calculatePosition(double &time_, int &segmentIndex)
{
    return Eigen::Vector3d::Zero();
}

Eigen::Vector3d BernsteinTrajectory::calculateVelocity(double &time_, int &segmentIndex)
{
    return Eigen::Vector3d::Zero();
}

Eigen::Vector3d BernsteinTrajectory::calculateAcceleration(double &time_, int &segmentIndex)
{
    return Eigen::Vector3d::Zero();
}

Eigen::Vector3d BernsteinTrajectory::calculateJerk(double &time_, int &segmentIndex)
{
    return Eigen::Vector3d::Zero();
}

Eigen::VectorXd BernsteinTrajectory::getBezierBasis(double &time_, int &order)
{
    return Eigen::VectorXd();
}



