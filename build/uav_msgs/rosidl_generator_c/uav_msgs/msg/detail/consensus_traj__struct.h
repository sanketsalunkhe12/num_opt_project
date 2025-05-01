// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from uav_msgs:msg/ConsensusTraj.idl
// generated code does not contain a copyright notice

#ifndef UAV_MSGS__MSG__DETAIL__CONSENSUS_TRAJ__STRUCT_H_
#define UAV_MSGS__MSG__DETAIL__CONSENSUS_TRAJ__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'robot_name'
#include "std_msgs/msg/detail/string__struct.h"
// Member 'waypoints'
// Member 'bern_coeffs'
#include "geometry_msgs/msg/detail/point__struct.h"
// Member 'velocity_min'
// Member 'velocity_max'
// Member 'acceleration_min'
// Member 'acceleration_max'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/ConsensusTraj in the package uav_msgs.
typedef struct uav_msgs__msg__ConsensusTraj
{
  std_msgs__msg__Header header;
  std_msgs__msg__String robot_name;
  geometry_msgs__msg__Point__Sequence waypoints;
  geometry_msgs__msg__Point__Sequence bern_coeffs;
  geometry_msgs__msg__Vector3 velocity_min;
  geometry_msgs__msg__Vector3 velocity_max;
  geometry_msgs__msg__Vector3 acceleration_min;
  geometry_msgs__msg__Vector3 acceleration_max;
} uav_msgs__msg__ConsensusTraj;

// Struct for a sequence of uav_msgs__msg__ConsensusTraj.
typedef struct uav_msgs__msg__ConsensusTraj__Sequence
{
  uav_msgs__msg__ConsensusTraj * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} uav_msgs__msg__ConsensusTraj__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UAV_MSGS__MSG__DETAIL__CONSENSUS_TRAJ__STRUCT_H_
