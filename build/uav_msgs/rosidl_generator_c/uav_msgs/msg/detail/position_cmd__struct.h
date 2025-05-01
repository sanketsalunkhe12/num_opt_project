// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from uav_msgs:msg/PositionCmd.idl
// generated code does not contain a copyright notice

#ifndef UAV_MSGS__MSG__DETAIL__POSITION_CMD__STRUCT_H_
#define UAV_MSGS__MSG__DETAIL__POSITION_CMD__STRUCT_H_

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
// Member 'position'
#include "geometry_msgs/msg/detail/point__struct.h"
// Member 'velocity'
// Member 'acceleration'
// Member 'jerk'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/PositionCmd in the package uav_msgs.
typedef struct uav_msgs__msg__PositionCmd
{
  std_msgs__msg__Header header;
  geometry_msgs__msg__Point position;
  geometry_msgs__msg__Vector3 velocity;
  geometry_msgs__msg__Vector3 acceleration;
  geometry_msgs__msg__Vector3 jerk;
  double yaw;
  double yaw_dot;
} uav_msgs__msg__PositionCmd;

// Struct for a sequence of uav_msgs__msg__PositionCmd.
typedef struct uav_msgs__msg__PositionCmd__Sequence
{
  uav_msgs__msg__PositionCmd * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} uav_msgs__msg__PositionCmd__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UAV_MSGS__MSG__DETAIL__POSITION_CMD__STRUCT_H_
