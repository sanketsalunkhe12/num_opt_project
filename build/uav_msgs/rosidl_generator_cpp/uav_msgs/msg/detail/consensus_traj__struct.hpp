// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from uav_msgs:msg/ConsensusTraj.idl
// generated code does not contain a copyright notice

#ifndef UAV_MSGS__MSG__DETAIL__CONSENSUS_TRAJ__STRUCT_HPP_
#define UAV_MSGS__MSG__DETAIL__CONSENSUS_TRAJ__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'robot_name'
#include "std_msgs/msg/detail/string__struct.hpp"
// Member 'waypoints'
// Member 'bern_coeffs'
#include "geometry_msgs/msg/detail/point__struct.hpp"
// Member 'velocity_min'
// Member 'velocity_max'
// Member 'acceleration_min'
// Member 'acceleration_max'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__uav_msgs__msg__ConsensusTraj __attribute__((deprecated))
#else
# define DEPRECATED__uav_msgs__msg__ConsensusTraj __declspec(deprecated)
#endif

namespace uav_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ConsensusTraj_
{
  using Type = ConsensusTraj_<ContainerAllocator>;

  explicit ConsensusTraj_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    robot_name(_init),
    velocity_min(_init),
    velocity_max(_init),
    acceleration_min(_init),
    acceleration_max(_init)
  {
    (void)_init;
  }

  explicit ConsensusTraj_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    robot_name(_alloc, _init),
    velocity_min(_alloc, _init),
    velocity_max(_alloc, _init),
    acceleration_min(_alloc, _init),
    acceleration_max(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _robot_name_type =
    std_msgs::msg::String_<ContainerAllocator>;
  _robot_name_type robot_name;
  using _waypoints_type =
    std::vector<geometry_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Point_<ContainerAllocator>>>;
  _waypoints_type waypoints;
  using _bern_coeffs_type =
    std::vector<geometry_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Point_<ContainerAllocator>>>;
  _bern_coeffs_type bern_coeffs;
  using _velocity_min_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _velocity_min_type velocity_min;
  using _velocity_max_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _velocity_max_type velocity_max;
  using _acceleration_min_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _acceleration_min_type acceleration_min;
  using _acceleration_max_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _acceleration_max_type acceleration_max;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__robot_name(
    const std_msgs::msg::String_<ContainerAllocator> & _arg)
  {
    this->robot_name = _arg;
    return *this;
  }
  Type & set__waypoints(
    const std::vector<geometry_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Point_<ContainerAllocator>>> & _arg)
  {
    this->waypoints = _arg;
    return *this;
  }
  Type & set__bern_coeffs(
    const std::vector<geometry_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Point_<ContainerAllocator>>> & _arg)
  {
    this->bern_coeffs = _arg;
    return *this;
  }
  Type & set__velocity_min(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->velocity_min = _arg;
    return *this;
  }
  Type & set__velocity_max(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->velocity_max = _arg;
    return *this;
  }
  Type & set__acceleration_min(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->acceleration_min = _arg;
    return *this;
  }
  Type & set__acceleration_max(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->acceleration_max = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    uav_msgs::msg::ConsensusTraj_<ContainerAllocator> *;
  using ConstRawPtr =
    const uav_msgs::msg::ConsensusTraj_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<uav_msgs::msg::ConsensusTraj_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<uav_msgs::msg::ConsensusTraj_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      uav_msgs::msg::ConsensusTraj_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<uav_msgs::msg::ConsensusTraj_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      uav_msgs::msg::ConsensusTraj_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<uav_msgs::msg::ConsensusTraj_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<uav_msgs::msg::ConsensusTraj_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<uav_msgs::msg::ConsensusTraj_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__uav_msgs__msg__ConsensusTraj
    std::shared_ptr<uav_msgs::msg::ConsensusTraj_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__uav_msgs__msg__ConsensusTraj
    std::shared_ptr<uav_msgs::msg::ConsensusTraj_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ConsensusTraj_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->robot_name != other.robot_name) {
      return false;
    }
    if (this->waypoints != other.waypoints) {
      return false;
    }
    if (this->bern_coeffs != other.bern_coeffs) {
      return false;
    }
    if (this->velocity_min != other.velocity_min) {
      return false;
    }
    if (this->velocity_max != other.velocity_max) {
      return false;
    }
    if (this->acceleration_min != other.acceleration_min) {
      return false;
    }
    if (this->acceleration_max != other.acceleration_max) {
      return false;
    }
    return true;
  }
  bool operator!=(const ConsensusTraj_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ConsensusTraj_

// alias to use template instance with default allocator
using ConsensusTraj =
  uav_msgs::msg::ConsensusTraj_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace uav_msgs

#endif  // UAV_MSGS__MSG__DETAIL__CONSENSUS_TRAJ__STRUCT_HPP_
