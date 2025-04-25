#ifndef TRAJECTORY_MANAGER_HPP
#define TRAJECTORY_MANAGER_HPP

#include "bern_traj/bernstein_trajectory.hpp"
#include "bern_traj/admm.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter.hpp"

#include "uav_msgs/msg/position_cmd.hpp"
#include "uav_msgs/msg/consensus_traj.hpp"

class TrajectoryManager : public rclcpp::Node, public std::enable_shared_from_this<TrajectoryManager>
{
    public:
        TrajectoryManager(const rclcpp::NodeOptions &options);
        ~TrajectoryManager();

        void initializeTrajectory();
    private:
        std::vector<Waypoint> waypoints;
        std::vector<Obstacle> obstacles;
        std::string trajectoryType;
        std::string robotName;

        TrajectoryParams bernsteinParams;
        
        std::shared_ptr<Trajectory> currentTrajectory = std::make_shared<BernsteinTrajectory>(bernsteinParams); // should be a null trajectory
     
        
        // publisher
        rclcpp::Publisher<uav_msgs::msg::PositionCmd>::SharedPtr positionCmdPub;
        rclcpp::Publisher<uav_msgs::msg::ConsensusTraj>::SharedPtr consensusTrajPub;

        // // subscriber
        // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub;
};

#endif // TRAJECTORY_MANAGER_HPP


