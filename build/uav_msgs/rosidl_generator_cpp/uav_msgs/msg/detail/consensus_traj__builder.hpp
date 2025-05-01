// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from uav_msgs:msg/ConsensusTraj.idl
// generated code does not contain a copyright notice

#ifndef UAV_MSGS__MSG__DETAIL__CONSENSUS_TRAJ__BUILDER_HPP_
#define UAV_MSGS__MSG__DETAIL__CONSENSUS_TRAJ__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "uav_msgs/msg/detail/consensus_traj__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace uav_msgs
{

namespace msg
{

namespace builder
{

class Init_ConsensusTraj_acceleration_max
{
public:
  explicit Init_ConsensusTraj_acceleration_max(::uav_msgs::msg::ConsensusTraj & msg)
  : msg_(msg)
  {}
  ::uav_msgs::msg::ConsensusTraj acceleration_max(::uav_msgs::msg::ConsensusTraj::_acceleration_max_type arg)
  {
    msg_.acceleration_max = std::move(arg);
    return std::move(msg_);
  }

private:
  ::uav_msgs::msg::ConsensusTraj msg_;
};

class Init_ConsensusTraj_acceleration_min
{
public:
  explicit Init_ConsensusTraj_acceleration_min(::uav_msgs::msg::ConsensusTraj & msg)
  : msg_(msg)
  {}
  Init_ConsensusTraj_acceleration_max acceleration_min(::uav_msgs::msg::ConsensusTraj::_acceleration_min_type arg)
  {
    msg_.acceleration_min = std::move(arg);
    return Init_ConsensusTraj_acceleration_max(msg_);
  }

private:
  ::uav_msgs::msg::ConsensusTraj msg_;
};

class Init_ConsensusTraj_velocity_max
{
public:
  explicit Init_ConsensusTraj_velocity_max(::uav_msgs::msg::ConsensusTraj & msg)
  : msg_(msg)
  {}
  Init_ConsensusTraj_acceleration_min velocity_max(::uav_msgs::msg::ConsensusTraj::_velocity_max_type arg)
  {
    msg_.velocity_max = std::move(arg);
    return Init_ConsensusTraj_acceleration_min(msg_);
  }

private:
  ::uav_msgs::msg::ConsensusTraj msg_;
};

class Init_ConsensusTraj_velocity_min
{
public:
  explicit Init_ConsensusTraj_velocity_min(::uav_msgs::msg::ConsensusTraj & msg)
  : msg_(msg)
  {}
  Init_ConsensusTraj_velocity_max velocity_min(::uav_msgs::msg::ConsensusTraj::_velocity_min_type arg)
  {
    msg_.velocity_min = std::move(arg);
    return Init_ConsensusTraj_velocity_max(msg_);
  }

private:
  ::uav_msgs::msg::ConsensusTraj msg_;
};

class Init_ConsensusTraj_bern_coeffs
{
public:
  explicit Init_ConsensusTraj_bern_coeffs(::uav_msgs::msg::ConsensusTraj & msg)
  : msg_(msg)
  {}
  Init_ConsensusTraj_velocity_min bern_coeffs(::uav_msgs::msg::ConsensusTraj::_bern_coeffs_type arg)
  {
    msg_.bern_coeffs = std::move(arg);
    return Init_ConsensusTraj_velocity_min(msg_);
  }

private:
  ::uav_msgs::msg::ConsensusTraj msg_;
};

class Init_ConsensusTraj_waypoints
{
public:
  explicit Init_ConsensusTraj_waypoints(::uav_msgs::msg::ConsensusTraj & msg)
  : msg_(msg)
  {}
  Init_ConsensusTraj_bern_coeffs waypoints(::uav_msgs::msg::ConsensusTraj::_waypoints_type arg)
  {
    msg_.waypoints = std::move(arg);
    return Init_ConsensusTraj_bern_coeffs(msg_);
  }

private:
  ::uav_msgs::msg::ConsensusTraj msg_;
};

class Init_ConsensusTraj_robot_name
{
public:
  explicit Init_ConsensusTraj_robot_name(::uav_msgs::msg::ConsensusTraj & msg)
  : msg_(msg)
  {}
  Init_ConsensusTraj_waypoints robot_name(::uav_msgs::msg::ConsensusTraj::_robot_name_type arg)
  {
    msg_.robot_name = std::move(arg);
    return Init_ConsensusTraj_waypoints(msg_);
  }

private:
  ::uav_msgs::msg::ConsensusTraj msg_;
};

class Init_ConsensusTraj_header
{
public:
  Init_ConsensusTraj_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ConsensusTraj_robot_name header(::uav_msgs::msg::ConsensusTraj::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ConsensusTraj_robot_name(msg_);
  }

private:
  ::uav_msgs::msg::ConsensusTraj msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::uav_msgs::msg::ConsensusTraj>()
{
  return uav_msgs::msg::builder::Init_ConsensusTraj_header();
}

}  // namespace uav_msgs

#endif  // UAV_MSGS__MSG__DETAIL__CONSENSUS_TRAJ__BUILDER_HPP_
