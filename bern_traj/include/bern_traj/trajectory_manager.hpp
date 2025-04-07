#ifndef TRAJECTORY_MANAGER_HPP
#define TRAJECTORY_MANAGER_HPP


// #include "trajectory_manager/trajectory/line_trajectory.hpp"
// #include "trajectory_manager/trajectory/circle_trajectory.hpp"
#include "bern_traj/bernstein_trajectory.hpp"

#include "rclcpp/rclcpp.hpp"

class TrajectoryManager : public rclcpp::Node
{
    public:
        TrajectoryManager(const rclcpp::NodeOptions &options);
        ~TrajectoryManager();

        // void setWaypoints(std::vector<Waypoint> &waypoints_);
        // void setTrajectoryType(std::string &trajectoryType_);

        // void initialize();
        // void activate();
        // void deactivate();

        // void update();

    private:
        std::vector<Waypoint> waypoints;
        std::string trajectoryType;

        // TrajectoryParams lineParams;
        // TrajectoryParams circleParams;
        // TrajectoryParams hoverParams;
        TrajectoryParams bernsteinParams;
        
        // Trajectory *trajectory;
        std::shared_ptr<Trajectory> currentTrajectory = std::make_shared<BernsteinTrajectory>(bernsteinParams); // should be a null trajectory

};

#endif // TRAJECTORY_MANAGER_HPP


