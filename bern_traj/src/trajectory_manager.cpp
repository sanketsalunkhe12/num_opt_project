#include "bern_traj/trajectory_manager.hpp"

TrajectoryManager::TrajectoryManager(const rclcpp::NodeOptions &options)
    : Node("trajectory_manager", options)
{
    // ros2 parameters

    // publishers
    // desired position command
    // diff flat control command

    // subscribers
    // odometery

    Waypoint wp0;
    wp0.position = {0.0, 0.0, 5.0};
    // wp0.velocity = {0.0, 0.0, 0.0};
    // wp0.acceleration = {0.0, 0.0, 0.0};
    // wp0.jerk = {0.0, 0.0, 0.0};

    Waypoint wp1;
    wp1.position = {5.0, 5.0, 5.0};
    // wp1.velocity = {2.0, 2.0, 0.0};
    // wp1.acceleration = {2.0, 2.0, 0.0};
    // wp1.jerk = {0.0, 0.0, 0.0};

    Waypoint wp2;
    wp2.position = {10.0, 0.0, 5.0};
    // wp2.velocity = {0.0, -2.0, 0.0};
    // wp2.acceleration = {0.0, -2.0, 0.0};
    // wp2.jerk = {0.0, 0.0, 0.0};
    
    // bernsteinParams.waypoints.push_back(wp0);
    // bernsteinParams.waypoints.push_back(wp1);
    // bernsteinParams.waypoints.push_back(wp2);

    std::vector<Waypoint> waypoints;
    waypoints.push_back(wp0);
    waypoints.push_back(wp1);
    waypoints.push_back(wp2);
    
    bernsteinParams.trajectoryType = BERNSTEIN;

    bernsteinParams.minVel = {-5.0, -5.0, -5.0};
    bernsteinParams.maxVel = {5.0, 5.0, 5.0};
    bernsteinParams.minAcc = {-2.0, -2.0, -2.0};
    bernsteinParams.maxAcc = {2.0, 2.0, 2.0};
    
    bernsteinParams.controlPtCount = 12;
    bernsteinParams.minDerivative = 3;
    bernsteinParams.trajDimension = 3;

    currentTrajectory = std::make_shared<BernsteinTrajectory>(bernsteinParams);

    currentTrajectory->initialize(this->shared_from_this(), &waypoints, nullptr);
}

TrajectoryManager::~TrajectoryManager()
{
    if(currentTrajectory != nullptr)
    {
        // delete currentTrajectory;
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("trajectory_manager");
    auto trajectory_manager = std::make_shared<TrajectoryManager>(node->get_node_options());

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}