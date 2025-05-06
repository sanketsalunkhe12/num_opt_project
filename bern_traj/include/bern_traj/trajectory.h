#ifndef TRAJECTORY_HPP
#define TRAJECTORY_HPP

#include "eigen3/Eigen/Dense"
#include "bern_traj/waypoint.h"
#include "bern_traj/obstacle.h"
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "uav_msgs/msg/position_cmd.hpp"


enum TrajectoryType
{
    LINE,
    CIRCLE,
    LISSAJOUS,
    HOVER,
    BERNSTEIN
};

struct TrajectoryParams
{
    // std::vector<Waypoint> waypoints = {}; // no need passing in initialize function
    TrajectoryType trajectoryType;

    std::vector<float> minVel, minAcc, minJerk = {0.0, 0.0, 0.0}; // inertial frame
    std::vector<float> maxVel, maxAcc, maxJerk = {0.0, 0.0, 0.0}; // inertial frame
    float max_yaw_rate{0.0}, max_yaw_acc{0.0}, min_yaw_rate{0.0}, min_yaw_acc{0.0}; // inertial frame

    // berstein related params
    int controlPtCount{12}, minDerivative{3}, trajDimension{3};
    float magicFabianConstant{6.0}, timeFactor{1.0};
    double obstacleDistance{0.0}, consensusDistance{0.0};
};

struct TrajectoryState
{
    std::vector<Eigen::Vector3d> position;
    std::vector<Eigen::Vector3d> velocity;
    std::vector<Eigen::Vector3d> acceleration;
    std::vector<Eigen::Vector3d> jerk;
};


class Trajectory
{
    public:
        // change double to float
        virtual Eigen::Vector3d getRefPosition(double &time_) = 0;
        virtual Eigen::Vector3d getRefVelocity(double &time_) = 0;
        virtual Eigen::Vector3d getRefAcceleration(double &time_) = 0;
        virtual Eigen::Vector3d getRefJerk(double &time_) = 0;

        virtual Eigen::MatrixXd getTrajCoefficients() = 0;

        virtual TrajectoryState calculateTrajectory() = 0; 
        
        // virtual Eigen::Vector<Eigen::Vector3d> getTrajectoryPosition() = 0;
        // virtual Eigen::Vector<Eigen::Vector3d> getTrajectoryVelocity() = 0;
        // virtual Eigen::Vector<Eigen::Vector3d> getTrajectoryAcceleration() = 0;
        // virtual Eigen::Vector<Eigen::Vector3d> getTrajectoryJerk() = 0;

        virtual bool initialize(//rclcpp::Node::SharedPtr node_ptr, 
                                const std::vector<Waypoint> *goal_wp,
                                const std::vector<Obstacle> *obstacles, 
                                const nav_msgs::msg::Odometry::ConstSharedPtr msg) = 0;
        virtual bool generateTrajectory(const Eigen::Vector3f &xi, const Eigen::Vector3f &xf,
                                        const Eigen::Vector3f &vi, const Eigen::Vector3f &vf,
                                        const Eigen::Vector3f &ai, const Eigen::Vector3f &af, 
                                        const float &yaw_i, const float &yaw_f, 
                                        const float &yaw_dot_i, const float &yaw_dot_f,
                                        float dt) = 0;
        virtual bool deactivate() = 0;
        virtual uav_msgs::msg::PositionCmd::SharedPtr update(const nav_msgs::msg::Odometry::ConstSharedPtr msg) = 0;

        bool isStarted, isCompleted, isActive; // isComplete = goal_reached
        double startTime, endTime;
        TrajectoryType trajectoryType;

        // kinematics constraints
        std::vector<float> minVel, minAcc, minJerk; //also yaw constraints
        std::vector<float> maxVel, maxAcc, maxJerk;
        float max_yaw_rate, max_yaw_acc, min_yaw_rate, min_yaw_acc;

        float total_trajectory_length, current_trajectory_length;
        float trajectory_duration;
};

#endif // TRAJECTORY_H