// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from uav_msgs:msg/PositionCmd.idl
// generated code does not contain a copyright notice

#ifndef UAV_MSGS__MSG__DETAIL__POSITION_CMD__BUILDER_HPP_
#define UAV_MSGS__MSG__DETAIL__POSITION_CMD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "uav_msgs/msg/detail/position_cmd__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace uav_msgs
{

namespace msg
{

namespace builder
{

class Init_PositionCmd_yaw_dot
{
public:
  explicit Init_PositionCmd_yaw_dot(::uav_msgs::msg::PositionCmd & msg)
  : msg_(msg)
  {}
  ::uav_msgs::msg::PositionCmd yaw_dot(::uav_msgs::msg::PositionCmd::_yaw_dot_type arg)
  {
    msg_.yaw_dot = std::move(arg);
    return std::move(msg_);
  }

private:
  ::uav_msgs::msg::PositionCmd msg_;
};

class Init_PositionCmd_yaw
{
public:
  explicit Init_PositionCmd_yaw(::uav_msgs::msg::PositionCmd & msg)
  : msg_(msg)
  {}
  Init_PositionCmd_yaw_dot yaw(::uav_msgs::msg::PositionCmd::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_PositionCmd_yaw_dot(msg_);
  }

private:
  ::uav_msgs::msg::PositionCmd msg_;
};

class Init_PositionCmd_jerk
{
public:
  explicit Init_PositionCmd_jerk(::uav_msgs::msg::PositionCmd & msg)
  : msg_(msg)
  {}
  Init_PositionCmd_yaw jerk(::uav_msgs::msg::PositionCmd::_jerk_type arg)
  {
    msg_.jerk = std::move(arg);
    return Init_PositionCmd_yaw(msg_);
  }

private:
  ::uav_msgs::msg::PositionCmd msg_;
};

class Init_PositionCmd_acceleration
{
public:
  explicit Init_PositionCmd_acceleration(::uav_msgs::msg::PositionCmd & msg)
  : msg_(msg)
  {}
  Init_PositionCmd_jerk acceleration(::uav_msgs::msg::PositionCmd::_acceleration_type arg)
  {
    msg_.acceleration = std::move(arg);
    return Init_PositionCmd_jerk(msg_);
  }

private:
  ::uav_msgs::msg::PositionCmd msg_;
};

class Init_PositionCmd_velocity
{
public:
  explicit Init_PositionCmd_velocity(::uav_msgs::msg::PositionCmd & msg)
  : msg_(msg)
  {}
  Init_PositionCmd_acceleration velocity(::uav_msgs::msg::PositionCmd::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_PositionCmd_acceleration(msg_);
  }

private:
  ::uav_msgs::msg::PositionCmd msg_;
};

class Init_PositionCmd_position
{
public:
  explicit Init_PositionCmd_position(::uav_msgs::msg::PositionCmd & msg)
  : msg_(msg)
  {}
  Init_PositionCmd_velocity position(::uav_msgs::msg::PositionCmd::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_PositionCmd_velocity(msg_);
  }

private:
  ::uav_msgs::msg::PositionCmd msg_;
};

class Init_PositionCmd_header
{
public:
  Init_PositionCmd_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PositionCmd_position header(::uav_msgs::msg::PositionCmd::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_PositionCmd_position(msg_);
  }

private:
  ::uav_msgs::msg::PositionCmd msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::uav_msgs::msg::PositionCmd>()
{
  return uav_msgs::msg::builder::Init_PositionCmd_header();
}

}  // namespace uav_msgs

#endif  // UAV_MSGS__MSG__DETAIL__POSITION_CMD__BUILDER_HPP_
