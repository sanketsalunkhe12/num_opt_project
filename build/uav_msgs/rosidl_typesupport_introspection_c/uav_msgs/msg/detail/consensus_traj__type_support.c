// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from uav_msgs:msg/ConsensusTraj.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "uav_msgs/msg/detail/consensus_traj__rosidl_typesupport_introspection_c.h"
#include "uav_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "uav_msgs/msg/detail/consensus_traj__functions.h"
#include "uav_msgs/msg/detail/consensus_traj__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `robot_name`
#include "std_msgs/msg/string.h"
// Member `robot_name`
#include "std_msgs/msg/detail/string__rosidl_typesupport_introspection_c.h"
// Member `waypoints`
// Member `bern_coeffs`
#include "geometry_msgs/msg/point.h"
// Member `waypoints`
// Member `bern_coeffs`
#include "geometry_msgs/msg/detail/point__rosidl_typesupport_introspection_c.h"
// Member `velocity_min`
// Member `velocity_max`
// Member `acceleration_min`
// Member `acceleration_max`
#include "geometry_msgs/msg/vector3.h"
// Member `velocity_min`
// Member `velocity_max`
// Member `acceleration_min`
// Member `acceleration_max`
#include "geometry_msgs/msg/detail/vector3__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void uav_msgs__msg__ConsensusTraj__rosidl_typesupport_introspection_c__ConsensusTraj_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  uav_msgs__msg__ConsensusTraj__init(message_memory);
}

void uav_msgs__msg__ConsensusTraj__rosidl_typesupport_introspection_c__ConsensusTraj_fini_function(void * message_memory)
{
  uav_msgs__msg__ConsensusTraj__fini(message_memory);
}

size_t uav_msgs__msg__ConsensusTraj__rosidl_typesupport_introspection_c__size_function__ConsensusTraj__waypoints(
  const void * untyped_member)
{
  const geometry_msgs__msg__Point__Sequence * member =
    (const geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return member->size;
}

const void * uav_msgs__msg__ConsensusTraj__rosidl_typesupport_introspection_c__get_const_function__ConsensusTraj__waypoints(
  const void * untyped_member, size_t index)
{
  const geometry_msgs__msg__Point__Sequence * member =
    (const geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

void * uav_msgs__msg__ConsensusTraj__rosidl_typesupport_introspection_c__get_function__ConsensusTraj__waypoints(
  void * untyped_member, size_t index)
{
  geometry_msgs__msg__Point__Sequence * member =
    (geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

void uav_msgs__msg__ConsensusTraj__rosidl_typesupport_introspection_c__fetch_function__ConsensusTraj__waypoints(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const geometry_msgs__msg__Point * item =
    ((const geometry_msgs__msg__Point *)
    uav_msgs__msg__ConsensusTraj__rosidl_typesupport_introspection_c__get_const_function__ConsensusTraj__waypoints(untyped_member, index));
  geometry_msgs__msg__Point * value =
    (geometry_msgs__msg__Point *)(untyped_value);
  *value = *item;
}

void uav_msgs__msg__ConsensusTraj__rosidl_typesupport_introspection_c__assign_function__ConsensusTraj__waypoints(
  void * untyped_member, size_t index, const void * untyped_value)
{
  geometry_msgs__msg__Point * item =
    ((geometry_msgs__msg__Point *)
    uav_msgs__msg__ConsensusTraj__rosidl_typesupport_introspection_c__get_function__ConsensusTraj__waypoints(untyped_member, index));
  const geometry_msgs__msg__Point * value =
    (const geometry_msgs__msg__Point *)(untyped_value);
  *item = *value;
}

bool uav_msgs__msg__ConsensusTraj__rosidl_typesupport_introspection_c__resize_function__ConsensusTraj__waypoints(
  void * untyped_member, size_t size)
{
  geometry_msgs__msg__Point__Sequence * member =
    (geometry_msgs__msg__Point__Sequence *)(untyped_member);
  geometry_msgs__msg__Point__Sequence__fini(member);
  return geometry_msgs__msg__Point__Sequence__init(member, size);
}

size_t uav_msgs__msg__ConsensusTraj__rosidl_typesupport_introspection_c__size_function__ConsensusTraj__bern_coeffs(
  const void * untyped_member)
{
  const geometry_msgs__msg__Point__Sequence * member =
    (const geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return member->size;
}

const void * uav_msgs__msg__ConsensusTraj__rosidl_typesupport_introspection_c__get_const_function__ConsensusTraj__bern_coeffs(
  const void * untyped_member, size_t index)
{
  const geometry_msgs__msg__Point__Sequence * member =
    (const geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

void * uav_msgs__msg__ConsensusTraj__rosidl_typesupport_introspection_c__get_function__ConsensusTraj__bern_coeffs(
  void * untyped_member, size_t index)
{
  geometry_msgs__msg__Point__Sequence * member =
    (geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

void uav_msgs__msg__ConsensusTraj__rosidl_typesupport_introspection_c__fetch_function__ConsensusTraj__bern_coeffs(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const geometry_msgs__msg__Point * item =
    ((const geometry_msgs__msg__Point *)
    uav_msgs__msg__ConsensusTraj__rosidl_typesupport_introspection_c__get_const_function__ConsensusTraj__bern_coeffs(untyped_member, index));
  geometry_msgs__msg__Point * value =
    (geometry_msgs__msg__Point *)(untyped_value);
  *value = *item;
}

void uav_msgs__msg__ConsensusTraj__rosidl_typesupport_introspection_c__assign_function__ConsensusTraj__bern_coeffs(
  void * untyped_member, size_t index, const void * untyped_value)
{
  geometry_msgs__msg__Point * item =
    ((geometry_msgs__msg__Point *)
    uav_msgs__msg__ConsensusTraj__rosidl_typesupport_introspection_c__get_function__ConsensusTraj__bern_coeffs(untyped_member, index));
  const geometry_msgs__msg__Point * value =
    (const geometry_msgs__msg__Point *)(untyped_value);
  *item = *value;
}

bool uav_msgs__msg__ConsensusTraj__rosidl_typesupport_introspection_c__resize_function__ConsensusTraj__bern_coeffs(
  void * untyped_member, size_t size)
{
  geometry_msgs__msg__Point__Sequence * member =
    (geometry_msgs__msg__Point__Sequence *)(untyped_member);
  geometry_msgs__msg__Point__Sequence__fini(member);
  return geometry_msgs__msg__Point__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember uav_msgs__msg__ConsensusTraj__rosidl_typesupport_introspection_c__ConsensusTraj_message_member_array[8] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(uav_msgs__msg__ConsensusTraj, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "robot_name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(uav_msgs__msg__ConsensusTraj, robot_name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "waypoints",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(uav_msgs__msg__ConsensusTraj, waypoints),  // bytes offset in struct
    NULL,  // default value
    uav_msgs__msg__ConsensusTraj__rosidl_typesupport_introspection_c__size_function__ConsensusTraj__waypoints,  // size() function pointer
    uav_msgs__msg__ConsensusTraj__rosidl_typesupport_introspection_c__get_const_function__ConsensusTraj__waypoints,  // get_const(index) function pointer
    uav_msgs__msg__ConsensusTraj__rosidl_typesupport_introspection_c__get_function__ConsensusTraj__waypoints,  // get(index) function pointer
    uav_msgs__msg__ConsensusTraj__rosidl_typesupport_introspection_c__fetch_function__ConsensusTraj__waypoints,  // fetch(index, &value) function pointer
    uav_msgs__msg__ConsensusTraj__rosidl_typesupport_introspection_c__assign_function__ConsensusTraj__waypoints,  // assign(index, value) function pointer
    uav_msgs__msg__ConsensusTraj__rosidl_typesupport_introspection_c__resize_function__ConsensusTraj__waypoints  // resize(index) function pointer
  },
  {
    "bern_coeffs",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(uav_msgs__msg__ConsensusTraj, bern_coeffs),  // bytes offset in struct
    NULL,  // default value
    uav_msgs__msg__ConsensusTraj__rosidl_typesupport_introspection_c__size_function__ConsensusTraj__bern_coeffs,  // size() function pointer
    uav_msgs__msg__ConsensusTraj__rosidl_typesupport_introspection_c__get_const_function__ConsensusTraj__bern_coeffs,  // get_const(index) function pointer
    uav_msgs__msg__ConsensusTraj__rosidl_typesupport_introspection_c__get_function__ConsensusTraj__bern_coeffs,  // get(index) function pointer
    uav_msgs__msg__ConsensusTraj__rosidl_typesupport_introspection_c__fetch_function__ConsensusTraj__bern_coeffs,  // fetch(index, &value) function pointer
    uav_msgs__msg__ConsensusTraj__rosidl_typesupport_introspection_c__assign_function__ConsensusTraj__bern_coeffs,  // assign(index, value) function pointer
    uav_msgs__msg__ConsensusTraj__rosidl_typesupport_introspection_c__resize_function__ConsensusTraj__bern_coeffs  // resize(index) function pointer
  },
  {
    "velocity_min",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(uav_msgs__msg__ConsensusTraj, velocity_min),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "velocity_max",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(uav_msgs__msg__ConsensusTraj, velocity_max),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "acceleration_min",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(uav_msgs__msg__ConsensusTraj, acceleration_min),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "acceleration_max",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(uav_msgs__msg__ConsensusTraj, acceleration_max),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers uav_msgs__msg__ConsensusTraj__rosidl_typesupport_introspection_c__ConsensusTraj_message_members = {
  "uav_msgs__msg",  // message namespace
  "ConsensusTraj",  // message name
  8,  // number of fields
  sizeof(uav_msgs__msg__ConsensusTraj),
  uav_msgs__msg__ConsensusTraj__rosidl_typesupport_introspection_c__ConsensusTraj_message_member_array,  // message members
  uav_msgs__msg__ConsensusTraj__rosidl_typesupport_introspection_c__ConsensusTraj_init_function,  // function to initialize message memory (memory has to be allocated)
  uav_msgs__msg__ConsensusTraj__rosidl_typesupport_introspection_c__ConsensusTraj_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t uav_msgs__msg__ConsensusTraj__rosidl_typesupport_introspection_c__ConsensusTraj_message_type_support_handle = {
  0,
  &uav_msgs__msg__ConsensusTraj__rosidl_typesupport_introspection_c__ConsensusTraj_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_uav_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, uav_msgs, msg, ConsensusTraj)() {
  uav_msgs__msg__ConsensusTraj__rosidl_typesupport_introspection_c__ConsensusTraj_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  uav_msgs__msg__ConsensusTraj__rosidl_typesupport_introspection_c__ConsensusTraj_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, String)();
  uav_msgs__msg__ConsensusTraj__rosidl_typesupport_introspection_c__ConsensusTraj_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point)();
  uav_msgs__msg__ConsensusTraj__rosidl_typesupport_introspection_c__ConsensusTraj_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point)();
  uav_msgs__msg__ConsensusTraj__rosidl_typesupport_introspection_c__ConsensusTraj_message_member_array[4].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Vector3)();
  uav_msgs__msg__ConsensusTraj__rosidl_typesupport_introspection_c__ConsensusTraj_message_member_array[5].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Vector3)();
  uav_msgs__msg__ConsensusTraj__rosidl_typesupport_introspection_c__ConsensusTraj_message_member_array[6].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Vector3)();
  uav_msgs__msg__ConsensusTraj__rosidl_typesupport_introspection_c__ConsensusTraj_message_member_array[7].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Vector3)();
  if (!uav_msgs__msg__ConsensusTraj__rosidl_typesupport_introspection_c__ConsensusTraj_message_type_support_handle.typesupport_identifier) {
    uav_msgs__msg__ConsensusTraj__rosidl_typesupport_introspection_c__ConsensusTraj_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &uav_msgs__msg__ConsensusTraj__rosidl_typesupport_introspection_c__ConsensusTraj_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
