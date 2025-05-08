#ifndef BERNSTEIN_TRAJECTORY_HPP
#define BERNSTEIN_TRAJECTORY_HPP

#include "trajectory.h"
#include "osqp/osqp.h"
#include "optimizer.hpp" // RH: Include our DQP implementation

struct QPEqConstraints
{
    Eigen::MatrixXd A;
    Eigen::VectorXd b;
    //Ax = b 
};

struct QPIneqConstraints
{
    Eigen::MatrixXd C;
    Eigen::VectorXd d, f;
    //d <= Cx <= f
};


class BernsteinTrajectory : public Trajectory
{
    public:
        BernsteinTrajectory(TrajectoryParams &bernstein_params_);
        BernsteinTrajectory(TrajectoryParams &bernstein_params_, int solver_Type); // Adding overloaded constructor which initializes member var for solver type switch variable
        ~BernsteinTrajectory();

        Eigen::Vector3d getRefPosition(double &time_);
        Eigen::Vector3d getRefVelocity(double &time_);
        Eigen::Vector3d getRefAcceleration(double &time_);
        Eigen::Vector3d getRefJerk(double &time_);

        Eigen::MatrixXd getTrajCoefficients();

        bool initialize(//rclcpp::Node::SharedPtr node_ptr, 
                        const std::vector<Waypoint> *goal_wp, 
                        const std::vector<Obstacle> *obstacles,
                        const nav_msgs::msg::Odometry::ConstSharedPtr msg);
        bool generateTrajectory(const Eigen::Vector3f &xi, const Eigen::Vector3f &xf,
                                const Eigen::Vector3f &vi, const Eigen::Vector3f &vf,
                                const Eigen::Vector3f &ai, const Eigen::Vector3f &af, 
                                const float &yaw_i, const float &yaw_f, 
                                const float &yaw_dot_i, const float &yaw_dot_f,
                                float dt);
        bool deactivate();
        uav_msgs::msg::PositionCmd::SharedPtr update(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

        double getObjectiveValue();

    private:
        Eigen::Vector3d start, end;
        double startTime, endTime;
        double duration;
        Eigen::Vector3d a, b, c, d;

        /*
        Bernstein Trajectory Parameters

            waypointCount: number of waypoints
            segmentCount: number of segments between waypoints
            coeffCount: total number of coefficients for all segments P_i,n
            controlPtCount: number of control points per segment (n+1)(P_0, P_1, ..., P_n)
            minDerivative: minimum derivative order (min jerk or snap of the trajectory) (m)
            trajDimension: dimension of the trajectory (x, y, z)
            sampleSize: discritized time for obstacle avoidance linearization

            bernCoeffComb: combined bernstein coefficients for all segments and all dimensions
                [P_seg1_x, ..., P_segN_x, P_seg1_y,  ..., P_segN_y, P_seg1_z,  ..., P_segN_z]
            bernCoeff: bernstein coefficients for each segment and each dimension
                [P_seg1_x, P_seg1_y, P_seg1_z;
                 P_seg2_x, P_seg2_y, P_seg2_z;
                 P_seg3_x, P_seg3_y, P_seg3_z;
                    ...,      ...,     ...;] 
        */

        int waypointCount{0}, segmentCount, obstacleCount{0}, 
            controlPtCount, minDerivative, trajDimension, segIdx,
            timeFactor, magicFabianConstant,
            sampleSize{12};
        bool isSQPreplan;
        bool isObstacle, isConsensus;
        double obstacleDist, consensusDist;
        Eigen::VectorXd bernCoeffComb;
        OSQPFloat *primalSol, *dualSol;
        Eigen::MatrixXd bernCoeff;
        std::vector<double> segmentTime;
        int solverType;

        
        // Bernstein functions
        bool solveOptimizedTraj(//rclcpp::Node::SharedPtr node_ptr, 
            const std::vector<Waypoint> *goal_wp, const std::vector<Obstacle> *obstacles);
        
        void generateSegmentTime(const std::vector<Waypoint> *goal_wp);

        Eigen::MatrixXd generateObjectiveFunction();
        Eigen::MatrixXd generateQMatrix(double &time_);
        Eigen::MatrixXd generateDerivativeMatrix(int &minDerivative_, double &time_);
        Eigen::VectorXd getBernsteinBasis(double &time_, int &minDerivative_);
        double nCr(int n, int r);
        
        // constraint generation
        QPEqConstraints generateEqConstraint(int &dimension_, const std::vector<Waypoint> *goal_wp);
        QPIneqConstraints generateIneqConstraint(int &dimension_, const std::vector<Waypoint> *goal_wp);
        QPIneqConstraints generateObstacleConstraint(int &dimension_,const std::vector<Obstacle> *obstacles);
        
        QPIneqConstraints generateCombObstacleConstraint(const std::vector<Obstacle> *obstacles);

        
        QPIneqConstraints generateConsensusConstraint();

        // solving OSQP problem
        bool threadOSQPSolver(Eigen::MatrixXd &Q, QPEqConstraints &eq_constraints, QPIneqConstraints &ineq_constraints);//, 
                                // rclcpp::Node::SharedPtr node_ptr);

	// RH: Adding dedicated solver for DQP solving
	bool combDQPSolver(Eigen::MatrixXd &Q_comb, QPEqConstraints &eq_constraints_comb, 
                            QPIneqConstraints &ineq_constraints_comb);

        bool combOSQPSolver(Eigen::MatrixXd &Q_comb, QPEqConstraints &eq_constraints_comb, 
                            QPIneqConstraints &ineq_constraints_comb);//, rclcpp::Node::SharedPtr node_ptr);

        // post optimization
        std::vector<Eigen::Vector3d> refPosition, refVelocity, refAcceleration, refJerk;

        TrajectoryState calculateTrajectory();

        Eigen::Vector3d calculatePosition(double &time_, int &segmentIndex);
        Eigen::Vector3d calculateVelocity(double &time_, int &segmentIndex);
        Eigen::Vector3d calculateAcceleration(double &time_, int &segmentIndex);
        Eigen::Vector3d calculateJerk(double &time_, int &segmentIndex);

        Eigen::VectorXd getBezierBasis(double &time_, int &order);

                
        double getBernstein(int n, int r, double time);
        
        Eigen::VectorXd getConvolutionVector();
        double getSegmentTime(int segIdx_);

};


#endif // BERNSTEIN_TRAJECTORY_HPP
