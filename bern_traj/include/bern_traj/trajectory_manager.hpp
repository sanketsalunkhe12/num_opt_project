#ifndef TRAJECTORY_MANAGER_HPP
#define TRAJECTORY_MANAGER_HPP

#include "bern_traj/bernstein_trajectory.hpp"

#include "rclcpp/rclcpp.hpp"

class TrajectoryManager : public rclcpp::Node, public std::enable_shared_from_this<TrajectoryManager>
{
    public:
        TrajectoryManager(const rclcpp::NodeOptions &options);
        ~TrajectoryManager();

        void initializeTrajectory();
    private:
        std::vector<Waypoint> waypoints;
        std::string trajectoryType;

        TrajectoryParams bernsteinParams;
        
        std::shared_ptr<Trajectory> currentTrajectory = std::make_shared<BernsteinTrajectory>(bernsteinParams); // should be a null trajectory
        
};

#endif // TRAJECTORY_MANAGER_HPP


