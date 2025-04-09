#include "bern_traj/trajectory_manager.hpp"

TrajectoryManager::TrajectoryManager(const rclcpp::NodeOptions &options)
                : Node("trajectory_manager", options)
{
    RCLCPP_INFO(this->get_logger(), "Trajectory Manager Node started");

    bernsteinParams.trajectoryType = BERNSTEIN;

    bernsteinParams.minVel = {-5.0, -5.0, -5.0};
    bernsteinParams.maxVel = {5.0, 5.0, 5.0};
    bernsteinParams.minAcc = {-2.0, -2.0, -2.0};
    bernsteinParams.maxAcc = {2.0, 2.0, 2.0};

    bernsteinParams.controlPtCount = 12;
    bernsteinParams.minDerivative = 3;
    bernsteinParams.trajDimension = 3;

    currentTrajectory = std::make_shared<BernsteinTrajectory>(bernsteinParams);
}

void TrajectoryManager::initializeTrajectory()
{
    std::vector<Waypoint> waypoints{
        {Eigen::Vector3f(0.0, 0.0, 5.0)},
        {Eigen::Vector3f(5.0, 5.0, 5.0)},
        {Eigen::Vector3f(10.0, 0.0, 5.0)}
    };

    // Call initialize after the object is fully managed by a shared_ptr
    currentTrajectory->initialize(&waypoints, nullptr);
}

TrajectoryManager::~TrajectoryManager()
{
    if (currentTrajectory != nullptr)
    {
        // Cleanup if necessary
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Create the TrajectoryManager node as a shared_ptr
    auto trajectory_manager = std::make_shared<TrajectoryManager>(rclcpp::NodeOptions());

    // Initialize the trajectory after the object is fully constructed
    trajectory_manager->initializeTrajectory();

    rclcpp::spin(trajectory_manager);
    rclcpp::shutdown();

    return 0;
}