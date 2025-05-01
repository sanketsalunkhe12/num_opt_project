// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from uav_msgs:msg/ConsensusTraj.idl
// generated code does not contain a copyright notice

#ifndef UAV_MSGS__MSG__DETAIL__CONSENSUS_TRAJ__TRAITS_HPP_
#define UAV_MSGS__MSG__DETAIL__CONSENSUS_TRAJ__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "uav_msgs/msg/detail/consensus_traj__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'robot_name'
#include "std_msgs/msg/detail/string__traits.hpp"
// Member 'waypoints'
// Member 'bern_coeffs'
#include "geometry_msgs/msg/detail/point__traits.hpp"
// Member 'velocity_min'
// Member 'velocity_max'
// Member 'acceleration_min'
// Member 'acceleration_max'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"

namespace uav_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const ConsensusTraj & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: robot_name
  {
    out << "robot_name: ";
    to_flow_style_yaml(msg.robot_name, out);
    out << ", ";
  }

  // member: waypoints
  {
    if (msg.waypoints.size() == 0) {
      out << "waypoints: []";
    } else {
      out << "waypoints: [";
      size_t pending_items = msg.waypoints.size();
      for (auto item : msg.waypoints) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: bern_coeffs
  {
    if (msg.bern_coeffs.size() == 0) {
      out << "bern_coeffs: []";
    } else {
      out << "bern_coeffs: [";
      size_t pending_items = msg.bern_coeffs.size();
      for (auto item : msg.bern_coeffs) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: velocity_min
  {
    out << "velocity_min: ";
    to_flow_style_yaml(msg.velocity_min, out);
    out << ", ";
  }

  // member: velocity_max
  {
    out << "velocity_max: ";
    to_flow_style_yaml(msg.velocity_max, out);
    out << ", ";
  }

  // member: acceleration_min
  {
    out << "acceleration_min: ";
    to_flow_style_yaml(msg.acceleration_min, out);
    out << ", ";
  }

  // member: acceleration_max
  {
    out << "acceleration_max: ";
    to_flow_style_yaml(msg.acceleration_max, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ConsensusTraj & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: robot_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robot_name:\n";
    to_block_style_yaml(msg.robot_name, out, indentation + 2);
  }

  // member: waypoints
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.waypoints.size() == 0) {
      out << "waypoints: []\n";
    } else {
      out << "waypoints:\n";
      for (auto item : msg.waypoints) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: bern_coeffs
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.bern_coeffs.size() == 0) {
      out << "bern_coeffs: []\n";
    } else {
      out << "bern_coeffs:\n";
      for (auto item : msg.bern_coeffs) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: velocity_min
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "velocity_min:\n";
    to_block_style_yaml(msg.velocity_min, out, indentation + 2);
  }

  // member: velocity_max
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "velocity_max:\n";
    to_block_style_yaml(msg.velocity_max, out, indentation + 2);
  }

  // member: acceleration_min
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "acceleration_min:\n";
    to_block_style_yaml(msg.acceleration_min, out, indentation + 2);
  }

  // member: acceleration_max
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "acceleration_max:\n";
    to_block_style_yaml(msg.acceleration_max, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ConsensusTraj & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace uav_msgs

namespace rosidl_generator_traits
{

[[deprecated("use uav_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const uav_msgs::msg::ConsensusTraj & msg,
  std::ostream & out, size_t indentation = 0)
{
  uav_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use uav_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const uav_msgs::msg::ConsensusTraj & msg)
{
  return uav_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<uav_msgs::msg::ConsensusTraj>()
{
  return "uav_msgs::msg::ConsensusTraj";
}

template<>
inline const char * name<uav_msgs::msg::ConsensusTraj>()
{
  return "uav_msgs/msg/ConsensusTraj";
}

template<>
struct has_fixed_size<uav_msgs::msg::ConsensusTraj>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<uav_msgs::msg::ConsensusTraj>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<uav_msgs::msg::ConsensusTraj>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UAV_MSGS__MSG__DETAIL__CONSENSUS_TRAJ__TRAITS_HPP_
