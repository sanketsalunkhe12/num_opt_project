#include "bern_traj/trajectory_manager.hpp"
#include <sstream>
#include <fstream>

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

    // parameters
    this->declare_parameter<std::string>("robot_name", "robot_0");
    this->get_parameter("robot_name", robotName);

    RCLCPP_INFO(this->get_logger(), "Robot name: %s", robotName.c_str());


    std::vector<double> wp_raw;
    this->declare_parameter("waypoints", std::vector<double>{});
    this->get_parameter("waypoints", wp_raw);

    for(size_t i = 0; i < wp_raw.size(); i += 3) 
    {
        waypoints.push_back({Eigen::Vector3f(wp_raw[i], wp_raw[i + 1], wp_raw[i + 2])});
    }

    // publisher
    consensusTrajPub = this->create_publisher<uav_msgs::msg::ConsensusTraj>("consensus_traj", 10);
    positionCmdPub = this->create_publisher<uav_msgs::msg::PositionCmd>("position_cmd", 10);
}

void TrajectoryManager::initializeTrajectory()
{
    // RCLCPP_INFO(this->get_logger(), "Loaded %ld waypoints", waypoints.size());
    // for(int i=0; i < waypoints.size(); ++i)
    // {
    //     RCLCPP_INFO(this->get_logger(), "Waypoint %d: %f, %f, %f", i, waypoints[i].position.x(), waypoints[i].position.y(), waypoints[i].position.z());
    // }

    std::vector<Obstacle> obstacles{
        {Eigen::Vector3f(2.0, 2.0, 5.0)}
        // {Eigen::Vector3f(7.0, 2.0, 5.0)}
    };

    // Call initialize after the object is fully managed by a shared_ptr
    if(currentTrajectory->initialize(&waypoints, &obstacles, nullptr))
    {
        RCLCPP_INFO(this->get_logger(), "Trajectory initialized successfully");

        // publish trajectory
        TrajectoryState trajState = currentTrajectory->calculateTrajectory();

        // if(vizTrajectory)
        // {
        //     // publishRefTrajectory();
        // }

        // save data to txt file
        std::ofstream
        filep("/home/sanket/Projects/docker_ws/ros2_humble_ws/data/position_data_"+robotName+".txt");
        std::ofstream
        filev("/home/sanket/Projects/docker_ws/ros2_humble_ws/data/velocity_data_"+robotName+".txt");
        std::ofstream
        filea("/home/sanket/Projects/docker_ws/ros2_humble_ws/data/acceleration_data_"+robotName+".txt");
        std::ofstream
        filej("/home/sanket/Projects/docker_ws/ros2_humble_ws/data/jerk_data_"+robotName+".txt");

        for(int i=0; i<trajState.position.size(); i++)
        {
            filep << trajState.position[i].transpose() << std::endl;
            filev << trajState.velocity[i].transpose() << std::endl;
            filea << trajState.acceleration[i].transpose() << std::endl;
            filej << trajState.jerk[i].transpose() << std::endl;
        }

        filep.close();
        filev.close();
        filea.close();
        filej.close();

        // publish trajectory coefficients
        Eigen::MatrixXd coeffs = currentTrajectory->getTrajCoefficients();
        uav_msgs::msg::ConsensusTraj consensus_traj_msg;

        consensus_traj_msg.header.stamp = this->now();
        consensus_traj_msg.header.frame_id = "map";

        consensus_traj_msg.waypoints.resize(waypoints.size());
        for (size_t i = 0; i < waypoints.size(); ++i)
        {
            consensus_traj_msg.waypoints[i].x = waypoints[i].position.x();
            consensus_traj_msg.waypoints[i].y = waypoints[i].position.y();
            consensus_traj_msg.waypoints[i].z = waypoints[i].position.z();
        }
        
        consensus_traj_msg.bern_coeffs.resize(coeffs.rows());
        for (int i = 0; i < coeffs.rows(); ++i)
        {
            consensus_traj_msg.bern_coeffs[i].x = coeffs(i, 0);
            consensus_traj_msg.bern_coeffs[i].y = coeffs(i, 1);
            consensus_traj_msg.bern_coeffs[i].z = coeffs(i, 2);
        }

        consensus_traj_msg.velocity_min.x = bernsteinParams.minVel[0];
        consensus_traj_msg.velocity_min.y = bernsteinParams.minVel[1];
        consensus_traj_msg.velocity_min.z = bernsteinParams.minVel[2];

        consensus_traj_msg.velocity_max.x = bernsteinParams.maxVel[0];
        consensus_traj_msg.velocity_max.y = bernsteinParams.maxVel[1];
        consensus_traj_msg.velocity_max.z = bernsteinParams.maxVel[2];

        consensus_traj_msg.acceleration_min.x = bernsteinParams.minAcc[0];
        consensus_traj_msg.acceleration_min.y = bernsteinParams.minAcc[1];
        consensus_traj_msg.acceleration_min.z = bernsteinParams.minAcc[2];

        consensus_traj_msg.acceleration_max.x = bernsteinParams.maxAcc[0];
        consensus_traj_msg.acceleration_max.y = bernsteinParams.maxAcc[1];
        consensus_traj_msg.acceleration_max.z = bernsteinParams.maxAcc[2];

        consensusTrajPub->publish(consensus_traj_msg);
        RCLCPP_INFO(this->get_logger(), "Trajectory coefficients published");

    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize trajectory");
    }


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

    // rclcpp::spin(trajectory_manager);
    rclcpp::shutdown();

    return 0;
}