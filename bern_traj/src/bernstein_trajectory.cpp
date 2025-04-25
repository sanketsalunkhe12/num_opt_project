#include "bern_traj/bernstein_trajectory.hpp"
#include <Eigen/Sparse>
#include <sstream>
#include <fstream>

BernsteinTrajectory::BernsteinTrajectory(TrajectoryParams &bernstein_params_)
{
    // std::cout << "BernsteinTrajectory: Constructor" << std::endl;
    
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

    obstacleDist = bernstein_params_.obstacleDistance;
    consensusDist = bernstein_params_.consensusDistance;

    segIdx = 0;
    isSQPreplan = false;

    // waypointCount = bernstein_params_.waypoints.size();
}

BernsteinTrajectory::~BernsteinTrajectory()
{

}

bool BernsteinTrajectory::initialize(// rclcpp::Node::SharedPtr node_ptr, 
                                    const std::vector<Waypoint> *goal_wp, 
                                    const std::vector<Obstacle> *obstacles,
                                    const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
    // RCLCPP_INFO(node_ptr->get_logger(), "BernsteinTrajectory: Initializing trajectory");
    // std::cout << "BernsteinTrajectory: Initializing trajectory" << std::endl;
    
    // timeFactor = node_ptr->declare_parameter("time_factor", 1.0);
    // magicFabianConstant = node_ptr->declare_parameter("magic_fabian_constant", 6.0);

    // node_ptr->get_parameter("time_factor", timeFactor);
    // node_ptr->get_parameter("magic_fabian_constant", magicFabianConstant);

    waypointCount = goal_wp->size();
    
    if(waypointCount < 2)
    {
        // RCLCPP_ERROR(node_ptr->get_logger(), "BernsteinTrajectory: Not enough waypoints");
        return false;
    }

    segmentCount = waypointCount - 1;
    bernCoeffComb = Eigen::VectorXd::Zero(segmentCount * trajDimension * controlPtCount);
    bernCoeff = Eigen::MatrixXd::Zero(segmentCount * controlPtCount, trajDimension);
    primalSol = new OSQPFloat[segmentCount * trajDimension * controlPtCount];
    dualSol = new OSQPFloat[segmentCount * trajDimension * controlPtCount];

    // inital guess for primal solution
    for(int i=0; i<waypointCount-1; i++)
    {
        // interpolate between waypoints and use as initial guess primal solution
        Waypoint wp0 = (*goal_wp)[i];
        Waypoint wp1 = (*goal_wp)[i+1];

        for(int j=0; j<controlPtCount; j++)
        {
            float temp_ = static_cast<float>(j) / static_cast<float>(controlPtCount-1);
            Eigen::Vector3f p_ = wp0.position + (wp1.position - wp0.position) * temp_;

            primalSol[(controlPtCount*segmentCount*0) + (i*controlPtCount) + j] = p_(0);
            primalSol[(controlPtCount*segmentCount*1) + (i*controlPtCount) + j] = p_(1);
            primalSol[(controlPtCount*segmentCount*2) + (i*controlPtCount) + j] = p_(2);

            bernCoeffComb[(controlPtCount*segmentCount*0) + (i*controlPtCount) + j] = p_(0);
            bernCoeffComb[(controlPtCount*segmentCount*1) + (i*controlPtCount) + j] = p_(1);
            bernCoeffComb[(controlPtCount*segmentCount*2) + (i*controlPtCount) + j] = p_(2);
            
            // std::cout << "primalsol[" << (controlPtCount*segmentCount*0) + (i*controlPtCount) + j << "]: " << primalSol[(controlPtCount*waypointCount*0) + (i*controlPtCount) + j] << std::endl;
            // std::cout << "primalsol[" << (controlPtCount*segmentCount*1) + (i*controlPtCount) + j << "]: " << primalSol[(controlPtCount*waypointCount*1) + (i*controlPtCount) + j] << std::endl;
            // std::cout << "primalsol[" << (controlPtCount*segmentCount*2) + (i*controlPtCount) + j << "]: " << primalSol[(controlPtCount*waypointCount*2) + (i*controlPtCount) + j] << std::endl;
        }
    }

    for(int dim=0; dim<trajDimension; dim++)
    {
        bernCoeff.col(dim) = bernCoeffComb.block(dim*controlPtCount*segmentCount, 0, 
                                                    controlPtCount*segmentCount, 1);
    }

    bool sucess;
    for(int sqp=0; sqp<10 ; sqp++)
    {
        std::cout << "BernsteinTrajectory: SQP iteration: " << sqp << std::endl;
        
        // print primal and dual solution
        std::cout << "BernsteinTrajectory: primal solution: " << std::endl;
        for(int i=0; i<segmentCount*controlPtCount; i++)
        {
            std::cout << primalSol[i] << std::endl;
        }

        std::cout <<  "BernsteinTrajectory: dual solution: " << std::endl;
        for(int i=0; i<segmentCount*controlPtCount; i++)
        {
            std::cout << dualSol[i] << std::endl;
        }
        
        sucess = solveOptimizedTraj(//node_ptr, 
            goal_wp, obstacles);
    }
    
    // bool sucess = solveOptimizedTraj(//node_ptr, 
    //                             goal_wp, obstacles);
    if(!sucess)
    {
        // RCLCPP_ERROR(node_ptr->get_logger(), "BernsteinTrajectory: Failed to solve optimized trajectory");
        return false;
    }
    else
    {
        
        // print all coeffiecients by dimension
        // for(int dim=0; dim<trajDimension; dim++)
        // {
        //     std::cout << "BernsteinTrajectory: Coefficients for dimension " << dim << ": \n";
        //     // std::cout << bernCoeff.col(dim) << std::endl;
        // }
        // calculateTrajectory();
        return true;
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


// TODO: along with odometry also take obstacles updated position and consensus from other agents
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
bool BernsteinTrajectory::solveOptimizedTraj(//rclcpp::Node::SharedPtr node_ptr, 
    const std::vector<Waypoint> *goal_wp, const std::vector<Obstacle> *obstacles)
{
    // RCLCPP_INFO(node_ptr->get_logger(), "BernsteinTrajectory: Solving optimized trajectory");
    
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
    int num_obstacle_constraint = 0;
    int deriv_order = 2; // applying inequality only on vel and acc
   
    for(int i=0; i<segmentCount; i++)
    {
        for(int j=1; j<=deriv_order; j++)
        {
            num_ineq_constraint += controlPtCount - j;
        }

        if(true)// isObstacle)
            num_obstacle_constraint += sampe_size;// controlPtCount;
        
    }



    // int num_ineq_constraint_comb = (num_ineq_constraint + num_obstacle_constraint) * trajDimension; 
    int num_ineq_constraint_comb = (num_ineq_constraint  * trajDimension) + num_obstacle_constraint; 

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

        // std::cout << "BernsteinTrajectory: Q_comb matrix: \n" << Q_comb << std::endl;

        // SQP solver when we have nonlinear constraints which are not convex
        // lets try all dimensiosn combined 
        // if(true)//isObstacle)
        // {
            
        //     QPIneqConstraints obstacle_constraints = generateObstacleConstraint(dim, obstacles);

        //     ineq_constraints_comb.C.block(num_ineq_constraint*trajDimension + dim*num_obstacle_constraint, 
        //             dim*controlPtCount*segmentCount, num_obstacle_constraint, controlPtCount*segmentCount) =
        //             obstacle_constraints.C;

        //     ineq_constraints_comb.d.block(num_ineq_constraint*trajDimension + dim*num_obstacle_constraint, 0,
        //             num_obstacle_constraint, 1) = obstacle_constraints.d;

        //     ineq_constraints_comb.f.block(num_ineq_constraint*trajDimension + dim*num_obstacle_constraint, 0,
        //             num_obstacle_constraint, 1) = obstacle_constraints.f;
            
        // }

        // if(isConsensus)
        // {
        //     // generating consensus constraints
        //     QPIneqConstraints consensus_constraints = generateConsensusConstraint();

        //     // combine with comb constraints
        // }
    }

    if(true)//isObstacle)
    {
        QPIneqConstraints obstacle_constraints = generateCombObstacleConstraint(obstacles);

        ineq_constraints_comb.C.block(num_ineq_constraint*trajDimension + 0, 0, 
                num_obstacle_constraint, bernCoeffComb.size()) = obstacle_constraints.C;

        ineq_constraints_comb.d.block(num_ineq_constraint*trajDimension + 0, 0, 
                num_obstacle_constraint, 1) = obstacle_constraints.d;

        ineq_constraints_comb.f.block(num_ineq_constraint*trajDimension + 0, 0, 
                num_obstacle_constraint, 1) = obstacle_constraints.f;
                
    }

    
    // std::cout << "BernsteinTrajectory: Q_comb matrix: \n" << Q_comb << std::endl;
    // std::cout << "BernsteinTrajectory: eq constraints matrix: \n" << eq_constraints_comb.A << std::endl;
    // std::cout << "BernsteinTrajectory: eq constraints vector: \n" << eq_constraints_comb.b << std::endl;

    std::cout << "BernsteinTrajectory: ineq constraints matrix: \n" << ineq_constraints_comb.C << std::endl;
    std::cout << "BernsteinTrajectory: ineq constraints vector: \n" << ineq_constraints_comb.d << std::endl;
    std::cout << "BernsteinTrajectory: ineq constraints vector: \n" << ineq_constraints_comb.f << std::endl;

    // if any additional constraints are needed, add them here
    
    
    // solving combined OSQP problem
    bool success = combOSQPSolver(Q_comb, eq_constraints_comb, ineq_constraints_comb);//, node_ptr);
    if(success)
    {
        // std::cout << "BernsteinTrajectory: Solved combined OSQP problem" << std::endl;
        bernCoeff = Eigen::MatrixXd::Zero(segmentCount * controlPtCount, trajDimension);
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
    // std::cout << "BernsteinTrajectory: Generating segment time" << std::endl;
    
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

        // std::cout << "BernsteinTrajectory: Segment " << i << " time: " << time << std::endl;
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
    // std::cout << "BernsteinTrajectory: Generating objective function" << std::endl;
   
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(controlPtCount * segmentCount, controlPtCount * segmentCount); 

    for(int i=0; i<segmentCount; i++)
    {
        // generating Q for each segment and then combining them
        Eigen::MatrixXd q = generateQMatrix(segmentTime[i]);
        Q.block(i*controlPtCount, i*controlPtCount, controlPtCount, controlPtCount) = q;
    }

    // std::cout << "BernsteinTrajectory: Objective matrix: \n" << Q << std::endl;

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
    // std::cout << "BernsteinTrajectory: Generating Q matrix" << std::endl;
    
    Eigen::MatrixXd q = Eigen::MatrixXd::Zero(controlPtCount-minDerivative, controlPtCount-minDerivative); 
    Eigen::MatrixXd Dm = generateDerivativeMatrix(minDerivative, time_).transpose(); 
    
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
    
    // Eigen::MatrixXd temp = factor * Dm.transpose() * q * Dm;
    // std::cout << "BernsteinTrajectory: Q matrix: \n" << temp << std::endl;

    return factor * Dm.transpose() * q * Dm;   
}

// TODO: something needs to check here
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
        Note: Output Dm is in transpose form
    */
    // std::cout << "BernsteinTrajectory: Generating derivative matrix" << std::endl;
    
    if (minDerivative_ == 0) 
    {
        return Eigen::MatrixXd::Identity(controlPtCount, controlPtCount);
    }
    
    Eigen::MatrixXd Dm = Eigen::MatrixXd::Zero(controlPtCount, controlPtCount-minDerivative_); //Dm transpose
    Eigen::VectorXd conv_filt = Eigen::VectorXd::Zero(controlPtCount); //s^[m]=[-1,1]

    conv_filt.head(minDerivative_ + 1) = getBernsteinBasis(time_, minDerivative_);
    // conv_filt = s*s*s*...*s (m times convolution, m = minDerivative)
    
    for(int i=0; i<controlPtCount-minDerivative_; i++)
    {
        int q = controlPtCount;
        if(i + minDerivative_ + 1 < controlPtCount)
        {
            q = i + minDerivative_ + 1; 
// IMP      // This +1 is not in original code but without this Dm is not taking last element in all cases 
        }
        for(int j=i; j<q; j++)
        {
            Dm(j,i) = conv_filt(j-i);
        }
    }

    // std::cout << "BernsteinTrajectory: Dm matrix: \n" << Dm << std::endl;
    
    return Dm; // it is already in transpose 
}

Eigen::VectorXd BernsteinTrajectory::getBernsteinBasis(double &time_, int &minDerivative_)
{
    /*
        Generates the convolution kernel s*m
        factor = (n!)/ [(n-m)! * (time)^m]
    */
    // std::cout << "BernsteinTrajectory: Generating Bernstein basis" << std::endl;
    Eigen::VectorXd row; // s for convolution

    // m=0 and s(0) = 1
    if (minDerivative_ == 0) 
    {
        row = Eigen::VectorXd::Zero(1);
        row(0) = 1;
        // std::cout << "BernsteinTrajectory 0 minder: Bernstein basis: \n" << row << std::endl;
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

    Eigen::VectorXd temp = factor * row;
    // std::cout << "BernsteinTrajectory: Bernstein basis: \n" << temp << std::endl;

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
    // std::cout << "BernsteinTrajectory: Generating eq constraints" << std::endl;

    int num_eq_constraint = controlPtCount * segmentCount; // why? ↑
    Eigen::VectorXd bernBasis;

    QPEqConstraints eq_constraints;
    eq_constraints.A = Eigen::MatrixXd::Zero(num_eq_constraint, controlPtCount*segmentCount);
    eq_constraints.b = Eigen::VectorXd::Zero(num_eq_constraint);

    // (1) trajectory endpoint constraints (start and end) for position, velocity, acceleration
    
    int constraintIndex = 0;
    for(int i=0; i<4; i++) // 4 = pos, vel, acc, jerk
    {
        // start point
        bernBasis = getBernsteinBasis(segmentTime[0], i);
        // std::cout << "BernsteinTrajectory: bernBasis: \n" << bernBasis << std::endl;
        eq_constraints.A.block(constraintIndex, 0, 1, i+1) = bernBasis.transpose(); 
        // std::cout << "BernsteinTrajectory: eq_constraints.A: \n" << eq_constraints.A << std::endl;
        eq_constraints.b(constraintIndex) = (*goal_wp)[0].getConstraint(i, dimension_);
        // std::cout << "BernsteinTrajectory: eq_constraints.b: \n" << eq_constraints.b << std::endl;
        constraintIndex += 1; 

        // end point
        bernBasis = getBernsteinBasis(segmentTime[segmentCount-1], i);
        eq_constraints.A.block(constraintIndex, controlPtCount*segmentCount-1-i, 1, i+1) = bernBasis.transpose();
        eq_constraints.b(constraintIndex) = (*goal_wp)[waypointCount-1].getConstraint(i, dimension_);
        constraintIndex += 1;
    }

    // std::cout << "BernsteinTrajectory 1: eq constraints matrix: \n" << eq_constraints.A << std::endl;


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

    // std::cout << "BernsteinTrajectory: eq constraints matrix: \n" << eq_constraints.A << std::endl;
    // std::cout << "BernsteinTrajectory: eq constraintse vector: \n" << eq_constraints.b << std::endl;

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
    // std::cout << "BernsteinTrajectory: Generating ineq constraints" << std::endl;
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
                            generateDerivativeMatrix(j, segmentTime[i]).transpose();
            
            // std::cout << "BernsteinTrajectorye : ineq constraints matrix: \n" << ineq_constraints.C << std::endl;

            float constraint_value; // = (*goal_wp)[i+1].getConstraint(j, dimension_);
            if(j == 1)
                constraint_value = maxVel[dimension_];
            else if(j == 2)
                constraint_value = maxAcc[dimension_];

            ineq_constraints.d.block(constraintIndex, 0, controlPtCount-j, 1) = 
                            Eigen::VectorXd::Constant(controlPtCount-j, -1.0 * constraint_value);
            ineq_constraints.f.block(constraintIndex, 0, controlPtCount-j, 1) =
                            Eigen::VectorXd::Constant(controlPtCount-j, constraint_value);

            // std::cout << "BernsteinTrajectory: ineq constraints vector: \n" << ineq_constraints.d << std::endl;
            // std::cout << "BernsteinTrajectory: ineq constraints vector: \n" << ineq_constraints.f << std::endl;

            constraintIndex += controlPtCount - j;
        }
    }

    // std::cout << "BernsteinTrajectory: ineq constraints matrix: \n" << ineq_constraints.C << std::endl;
    // std::cout << "BernsteinTrajectory: ineq constraints vector: \n" << ineq_constraints.d << std::endl;
    // std::cout << "BernsteinTrajectory: ineq constraints vector: \n" << ineq_constraints.f << std::endl;

    // (2) additional constraints like constant z etc can be added here as ineq_constraints
    return ineq_constraints;
}

QPIneqConstraints BernsteinTrajectory::generateObstacleConstraint(int &dimension_, const std::vector<Obstacle> *obstacles)
{
    
    /*
        generating obstacle constraints which are linearized along at xk    
        [\delta B] = [B_0^n(\tau)  
                      B_2^n(\tau)
                      ...
                      B_n^n(\tau)] \in R^{(1)x(n+1)}

        Here we need to apply the obstacle constraints on the entire trajectory at all time steps 
        Lets assume sample size = controlPtCount
    */

    // int sampe_size = 20;

    QPIneqConstraints ineq_constraints;
    
    std::cout  << "BernsteinTrajectory: Generating obstacle constraints dimension" << dimension_ << std::endl;
    std::cout << "BernsteinTrajectory: obstacle distance: " << obstacleDist << std::endl;


    // ineq_constraints.C = Eigen::MatrixXd::Zero(controlPtCount*segmentCount, controlPtCount*segmentCount);
    // ineq_constraints.d = Eigen::VectorXd::Zero(controlPtCount*segmentCount);
    // ineq_constraints.f = Eigen::VectorXd::Zero(controlPtCount*segmentCount);


    ineq_constraints.C = Eigen::MatrixXd::Zero(sampe_size*segmentCount, controlPtCount*segmentCount);
    ineq_constraints.d = Eigen::VectorXd::Zero(sampe_size*segmentCount);
    ineq_constraints.f = Eigen::VectorXd::Zero(sampe_size*segmentCount);
    
    // Dm derivative matrix Dont need this just identity matrix    
    // q matrix
    // Eigen::MatrixXd q_ = Eigen::MatrixXd::Zero(controlPtCount, controlPtCount); 
        
    // int bigN = 2*(controlPtCount);
    // int smallN = controlPtCount;

    // for(int i=0; i<controlPtCount; i++)
    // {
    //     for(int j=0; j<controlPtCount; j++)
    //     {
    //         q_(i, j) = 1 / (nCr(bigN, (i + j)) / (nCr(smallN, i) * nCr(smallN, j)));
    //     }
    // }
     
    // for(int i=0; i<segmentCount; i++)
    // {
    //     ineq_constraints.C.block(i*controlPtCount, i*controlPtCount, controlPtCount, controlPtCount) = q_;
    // }

    // with discrtized time t = 1/12
    // for(int sample=0; sample<controlPtCount; sample++)
    for(int sample=0; sample<sampe_size; sample++)
    {
        // double time_ = (double)sample / (double)(controlPtCount-1);
        double time_ = (double)sample / (double)(sampe_size-1);
        Eigen::VectorXd bernBasis = getBezierBasis(time_, controlPtCount);

        for(int seg_=0; seg_<segmentCount; seg_++)
        
        {
            // ineq_constraints.C.block(sample + (seg_*controlPtCount), seg_*controlPtCount, 1, controlPtCount) = 
            //                                 bernBasis.transpose();

            ineq_constraints.C.block(sample + (seg_*sampe_size), seg_*controlPtCount, 1, controlPtCount) = 
                                            bernBasis.transpose();

            // upper bound constraints f
            double position = calculatePosition(time_, seg_)[dimension_];
            double obstacle_position = (*obstacles)[0].position[dimension_];
            double obst_position = (position - obstacle_position);

            std::cout << "BernsteinTrajectory: obstacle position: " << obst_position << std::endl;

            // double denom = 2.0 * (position - obstacle_position);
            double denom = 2.0 * (obst_position);
            if (std::abs(denom) < 1e-6)
                denom = (denom >= 0.0 ? 1e-6 : -1e-6);  // prevent instability
            
            // double constant = ((obstacleDist*obstacleDist) - (position - obstacle_position)*(position - obstacle_position)) / 
            //                     (2 * (position - obstacle_position));

            double constant = ((obstacleDist*obstacleDist) - (obst_position)*(obst_position)) / 
                                (denom);



            Eigen::VectorXd bernCoeffPk = bernCoeff.block(seg_*controlPtCount, dimension_, controlPtCount, 1);
      
            double b_Pk = bernBasis.dot(bernCoeffPk);
            ineq_constraints.f(sample + (seg_*sampe_size)) = b_Pk + constant;

            // lower bound constraints d
            ineq_constraints.d(sample + (seg_*sampe_size)) = -OSQP_INFTY;
        }
    }

    // std::cout << "BernsteinTrajectory: obstacle constraints matrix: \n" << ineq_constraints.C << std::endl;

    // std::cout << "BernsteinTrajectory: obstacle constraints vector: \n" << ineq_constraints.f << std::endl;
    
    
    // // assign -inifinity to the obstacle lower constraints d
    // for(int i=0; i<controlPtCount*segmentCount; i++)
    // {
    //     ineq_constraints.d(i) = -OSQP_INFTY;
    // }

    // // convert obstacle into bernstein
    // Eigen::VectorXd obs_bern = Eigen::VectorXd::Ones(controlPtCount*segmentCount) * (*obstacles)[0].position[dimension_];
    
    // // obstacle upper constraints f [\delta B][Pk] - ( g(Pk) / (2*(Bi(t) - Obs(t)) )
    // Eigen::VectorXd b_pk = ineq_constraints.C * bernCoeffComb.block(dimension_, 0, controlPtCount*segmentCount, 1);
    // Eigen::VectorXd g_pk = (Eigen::VectorXd::Ones(controlPtCount*segmentCount) * obstacle_dist).array() - 
    //                         (b_pk - obs_bern).array().square();

    // ineq_constraints.f = b_pk - g_pk; 
    // // TODO: not complete and sure about formulation

    // std::cout << "Done BernsteinTrajectory: obstacle constraints matrix" << std::endl;

    return ineq_constraints;
}


QPIneqConstraints BernsteinTrajectory::generateCombObstacleConstraint(const std::vector<Obstacle> *obstacles)
{
    QPIneqConstraints comb_obst_constraints;

    std::cout  << "BernsteinTrajectory: Generating comb obstacle constraints dimension" << std::endl;
    std::cout << "BernsteinTrajectory: obstacle distance: " << obstacleDist << std::endl;

    comb_obst_constraints.C = Eigen::MatrixXd::Zero(sampe_size*segmentCount, controlPtCount*segmentCount*trajDimension);
    comb_obst_constraints.d = Eigen::VectorXd::Zero(sampe_size*segmentCount);
    comb_obst_constraints.f = Eigen::VectorXd::Zero(sampe_size*segmentCount);

    for(int sample=0; sample<sampe_size; sample++)
    {
        double time_ = (double)sample / (double)(sampe_size-1);
        Eigen::VectorXd bernBasis = getBezierBasis(time_, controlPtCount);

        for(int seg_=0; seg_<segmentCount; seg_++)
        {
            comb_obst_constraints.C.block(sample + (seg_*sampe_size), seg_*controlPtCount, 1, controlPtCount) = 
                                            bernBasis.transpose(); // for x
            comb_obst_constraints.C.block(sample + (seg_*sampe_size), seg_*controlPtCount + controlPtCount*segmentCount, 1, controlPtCount) = 
                                            bernBasis.transpose(); // for y
            comb_obst_constraints.C.block(sample + (seg_*sampe_size), seg_*controlPtCount + 2*controlPtCount*segmentCount, 1, controlPtCount) =
                                            bernBasis.transpose(); // for z

            Eigen::Vector3d position_vec = calculatePosition(time_, seg_);
            Eigen::Vector3d obstacle_vec = (*obstacles)[0].position;
            double obst_position = (position_vec - obstacle_vec).norm(); 
            
            double denom = 2.0 * (obst_position);
            if (std::abs(denom) < 1e-6)
                denom = (denom >= 0.0 ? 1e-6 : -1e-6);

            double constant = ((obstacleDist*obstacleDist) - (obst_position*obst_position)) / 
                                (denom);
            
            
            Eigen::VectorXd bernBasisComb = Eigen::VectorXd::Zero(controlPtCount*segmentCount*trajDimension);
            for(int dim=0; dim<trajDimension; dim++)
            {
                bernBasisComb.block(dim*controlPtCount*segmentCount, 0, controlPtCount*segmentCount, 1) = 
                    bernBasis;
            }

            double b_Pk = bernBasisComb.dot(bernCoeffComb);
            
            comb_obst_constraints.f(sample + (seg_*sampe_size)) = constant; // + b_Pk + ;
                        
            comb_obst_constraints.d(sample + (seg_*sampe_size)) = -OSQP_INFTY;
        }
    }

    // std::cout << "BernsteinTrajectory: obstacle constraints matrix: \n" << comb_obst_constraints.C << std::endl;

    // std::cout << "BernsteinTrajectory: obstacle constraints vector: \n" << comb_obst_constraints.d << std::endl;
    // std::cout << "BernsteinTrajectory: obstacle constraints vector: \n" << comb_obst_constraints.f << std::endl;
    
    return comb_obst_constraints;
}


QPIneqConstraints BernsteinTrajectory::generateConsensusConstraint()
{
    QPIneqConstraints ineq_constraints;

    return ineq_constraints;
}

// solving OSQP problem
bool BernsteinTrajectory::threadOSQPSolver(Eigen::MatrixXd &Q, QPEqConstraints &eq_constraints, 
                                            QPIneqConstraints &ineq_constraints)//, rclcpp::Node::SharedPtr node_ptr)
{
    return true;
}

bool BernsteinTrajectory::combOSQPSolver(Eigen::MatrixXd &Q_comb, QPEqConstraints &eq_constraints_comb, 
                                        QPIneqConstraints &ineq_constraints_comb)//, rclcpp::Node::SharedPtr node_ptr)
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
    
    // std::cout << "BernsteinTrajectory: Solving combined OSQP problem" << std::endl;

    // combine all eq and ineq constraints
    Eigen::MatrixXd combConstraints = Eigen::MatrixXd::Zero(eq_constraints_comb.A.rows() + ineq_constraints_comb.C.rows(), 
                                                            bernCoeffComb.size());
    combConstraints.block(0, 0, eq_constraints_comb.A.rows(), bernCoeffComb.size()) = eq_constraints_comb.A;
    combConstraints.block(eq_constraints_comb.A.rows(), 0, ineq_constraints_comb.C.rows(), bernCoeffComb.size()) = ineq_constraints_comb.C;

    // std::cout << "BernsteinTrajectory: Combined constraints matrix: \n" << combConstraints << std::endl;
    
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
    // std::cout << "BernsteinTrajectory: Combined constraints lower bound size : " << l.size() << std::endl;
    // std::cout << "BernsteinTrajectory: Combined constraints lower bound size : " << u.size() << std::endl;

    
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
    
    if(isSQPreplan)
        osqp_warm_start(solver, primalSol, dualSol);
    else
        osqp_warm_start(solver, primalSol, nullptr);
    
    auto start = std::chrono::high_resolution_clock::now();

    isSQPreplan = true;

    exitflag = osqp_solve(solver);

    if(exitflag != 0)
    {
        // RCLCPP_ERROR(node_ptr->get_logger(), "Combined OSQP solver failed with exitflag: %d", exitflag);
        std::cout << "BernsteinTrajectory: Combined OSQP solver failed with exitflag: " << exitflag << std::endl;
        return false;
    }
    else if(solver->info->status_val == OSQP_SOLVED)
    {
        // RCLCPP_INFO(node_ptr->get_logger(), "Combined OSQP solver solved successfully");
        std::cout << "BernsteinTrajectory: Combined OSQP solver solved successfully" << std::endl;

        primalSol = solver->solution->x; // primal solution
        dualSol = solver->solution->y; // dual solution

        for(int i=0; i<n; i++)
        {
            bernCoeffComb(i) = primalSol[i]; 
        }
        // osqp_cleanup(solver);

        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        std::cout << "Elapsed wall time: " << elapsed.count() << " seconds\n";

        // no SQP replanning no obstacle
        // 0.000594863 seconds without primal and dual warm start
        // 0.00028736 seconds with primal start

        return true;
    }
    else if(solver->info->status_val == OSQP_PRIMAL_INFEASIBLE || 
            solver->info->status_val == OSQP_DUAL_INFEASIBLE)
    {
        std::cout << "BernsteinTrajectory: Combined OSQP problem is primal dual infeasible" << std::endl;
        
        primalSol = solver->solution->x; // primal solution
        dualSol = solver->solution->y; // dual solution

        for(int i=0; i<n; i++)
            bernCoeffComb(i) = primalSol[i]; 


        return false;
    }
    else
    {
        // RCLCPP_ERROR(node_ptr->get_logger(), "Combined OSQP solver failed with status: %d", solver->info->status_val);
        std::cout << "BernsteinTrajectory: Combined OSQP solver failed with status: " << solver->info->status_val << std::endl;
        
        primalSol = solver->solution->x; // primal solution
        dualSol = solver->solution->y; // dual solution

        for(int i=0; i<n; i++)
            bernCoeffComb(i) = primalSol[i]; 

        return false;
    }
}



double BernsteinTrajectory::getBernstein(int n, int r, double time)
{
    return 0.0;
}

Eigen::VectorXd BernsteinTrajectory::getConvolutionVector()
{
    // feels no need this function
    return Eigen::VectorXd();
}

double BernsteinTrajectory::getSegmentTime(int segIdx_)
{
    return 0.0;
}

// post optimization

TrajectoryState BernsteinTrajectory::calculateTrajectory()
{
    // calculate reference trajectory after opimization for visualization 
    // std::cout << "BernsteinTrajectory: Calculating trajectory" << std::endl;

    TrajectoryState trajState;
    
    for(int i=0; i<segmentCount; i++)
    {
        for(double time_=0.0; time_<= 1.0; time_+=0.01)
        {
            Eigen::Vector3d pos = calculatePosition(time_, i);
            Eigen::Vector3d vel = calculateVelocity(time_, i);
            Eigen::Vector3d acc = calculateAcceleration(time_, i);
            Eigen::Vector3d jerk = calculateJerk(time_, i);

            trajState.position.push_back(pos);
            trajState.velocity.push_back(vel);
            trajState.acceleration.push_back(acc);
            trajState.jerk.push_back(jerk);
        }
    }

    return trajState;
}

// TODO: factor values are not as per formula in original code
// verify this.
// TOTO: Also combine all pos, vel, acc, jerk in one function
Eigen::Vector3d BernsteinTrajectory::calculatePosition(double &time_, int &segmentIndex)
{
    /*
        Input: time_ in [0, 1]
        Compute Bernstein polynomial position 
        p(t) = sum_{i=0}^{n-1} P_i * B_{n,i}(tau)
        where P_i is the control point and B_{n,i}(tau) is the Bernstein basis
    */

    int order = controlPtCount - 0; // B_{i}^{n-m}
    Eigen::Vector3d traj_position;

    for(int dim=0; dim<trajDimension; dim++)
    {
        Eigen::VectorXd x_bern_coeff = bernCoeff.block(segmentIndex*controlPtCount, dim, controlPtCount, 1);
        Eigen::VectorXd bern_basis = getBezierBasis(time_, order);

        double position = bern_basis.dot(x_bern_coeff);
        traj_position(dim) = position;
    }
    
    return traj_position;
}

Eigen::Vector3d BernsteinTrajectory::calculateVelocity(double &time_, int &segmentIndex)
{
    /*
        Compute Bernstein polynomial velocity 
        v(t) = n/T * sum_{i=0}^{n-m-1} (P_i+1 - P_i) * B_{n-m,i}(tau)
    */

    int order = controlPtCount - 1; 
    Eigen::Vector3d traj_velocity;

    for(int dim=0; dim<trajDimension; dim++)
    {
        Eigen::VectorXd x_bern_coeff = bernCoeff.block(segmentIndex*controlPtCount, dim, controlPtCount, 1);
        Eigen::VectorXd vel_bern_coeff = x_bern_coeff.tail(controlPtCount-1) - x_bern_coeff.head(controlPtCount-1); // (P_i+1 - P_i)
        
        Eigen::VectorXd bern_basis = getBezierBasis(time_, order);

        double factor = ((controlPtCount - 1) / segmentTime[segmentIndex])*1.0; // n/T

        double velocity = factor * bern_basis.dot(vel_bern_coeff);
        traj_velocity(dim) = velocity;
    }
    
    return traj_velocity;
}

Eigen::Vector3d BernsteinTrajectory::calculateAcceleration(double &time_, int &segmentIndex)
{
    /*
        Compute Bernstein polynomial acceleration 
        a(t) = n*(n-1)/T^2 * sum_{i=0}^{n-m-1} (P_i+2 - 2*P_i+1 + P_i) * B_{n-m,i}(tau)
    */
    
    int order = controlPtCount - 2;
    Eigen::Vector3d traj_acceleration;

    for(int dim=0; dim<trajDimension; dim++)
    {
        Eigen::VectorXd x_bern_coeff = bernCoeff.block(segmentIndex*controlPtCount, dim, controlPtCount, 1);
        Eigen::VectorXd acc_bern_coeff = x_bern_coeff.segment(2, controlPtCount-2) - 2.0*x_bern_coeff.segment(1, controlPtCount-2) + x_bern_coeff.segment(0, controlPtCount-2);
        // (P_i+2 - 2*P_i+1 + P_i)
        
        Eigen::VectorXd bern_basis = getBezierBasis(time_, order);

        double factor = ((controlPtCount-1) * (controlPtCount-2)) / (segmentTime[segmentIndex] * segmentTime[segmentIndex]); // n*(n-1)/T^2

        double acceleration = factor * bern_basis.dot(acc_bern_coeff);
        traj_acceleration(dim) = acceleration;
    }
    
    return traj_acceleration;
}

Eigen::Vector3d BernsteinTrajectory::calculateJerk(double &time_, int &segmentIndex)
{
    /*
        Compute Bernstein polynomial jerk 
        j(t) = n*(n-1)*(n-2)/T^3 * sum_{i=0}^{n-m-1} (P_i+3 - 3*P_i+2 + 3*P_i+1 - P_i) * B_{n-m,i}(tau)
    */

    int order = controlPtCount - 3;
    Eigen::Vector3d traj_jerk;

    for(int dim=0; dim<trajDimension; dim++)
    {
        Eigen::VectorXd x_bern_coeff = bernCoeff.block(segmentIndex*controlPtCount, dim, controlPtCount, 1);
        Eigen::VectorXd jerk_bern_coeff = x_bern_coeff.segment(3, controlPtCount-3) - 3.0*x_bern_coeff.segment(2, controlPtCount-3) 
                            + 3.0*x_bern_coeff.segment(1, controlPtCount-3) - x_bern_coeff.segment(0, controlPtCount-3);
        // (P_i+3 - 3*P_i+2 + 3*P_i+1 - P_i)
        
        Eigen::VectorXd bern_basis = getBezierBasis(time_, order);

        double factor = ((controlPtCount-1) * (controlPtCount-2) * (controlPtCount-3)) / (segmentTime[segmentIndex] * segmentTime[segmentIndex] * segmentTime[segmentIndex]); 
        // n*(n-1)*(n-2)/T^3

        double jerk = factor * bern_basis.dot(jerk_bern_coeff);
        traj_jerk(dim) = jerk;
    }

    return traj_jerk;
}

Eigen::VectorXd BernsteinTrajectory::getBezierBasis(double &time_, int &order)
{
    /*
        Generates the Bernstein basis for given order and time
        B^{n-m}_{i}(tau) = nCr(n-m,i) * (1-tau)^(n-m-i) * tau^i
        where n-m = order, i = index of control point, tau = time
    */
    
    Eigen::VectorXd bern_basis = Eigen::VectorXd::Zero(order);

    for(int i=0; i<order; i++)
    {
        bern_basis(i) = nCr(order-1,i) * pow((1-time_), (order-1-i)) * pow(time_,i);
    }
    
    return bern_basis;
}

Eigen::MatrixXd BernsteinTrajectory::getTrajCoefficients()
{
    return bernCoeff;
}


