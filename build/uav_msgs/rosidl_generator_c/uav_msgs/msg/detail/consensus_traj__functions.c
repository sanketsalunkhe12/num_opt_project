// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from uav_msgs:msg/ConsensusTraj.idl
// generated code does not contain a copyright notice
#include "uav_msgs/msg/detail/consensus_traj__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `robot_name`
#include "std_msgs/msg/detail/string__functions.h"
// Member `waypoints`
// Member `bern_coeffs`
#include "geometry_msgs/msg/detail/point__functions.h"
// Member `velocity_min`
// Member `velocity_max`
// Member `acceleration_min`
// Member `acceleration_max`
#include "geometry_msgs/msg/detail/vector3__functions.h"

bool
uav_msgs__msg__ConsensusTraj__init(uav_msgs__msg__ConsensusTraj * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    uav_msgs__msg__ConsensusTraj__fini(msg);
    return false;
  }
  // robot_name
  if (!std_msgs__msg__String__init(&msg->robot_name)) {
    uav_msgs__msg__ConsensusTraj__fini(msg);
    return false;
  }
  // waypoints
  if (!geometry_msgs__msg__Point__Sequence__init(&msg->waypoints, 0)) {
    uav_msgs__msg__ConsensusTraj__fini(msg);
    return false;
  }
  // bern_coeffs
  if (!geometry_msgs__msg__Point__Sequence__init(&msg->bern_coeffs, 0)) {
    uav_msgs__msg__ConsensusTraj__fini(msg);
    return false;
  }
  // velocity_min
  if (!geometry_msgs__msg__Vector3__init(&msg->velocity_min)) {
    uav_msgs__msg__ConsensusTraj__fini(msg);
    return false;
  }
  // velocity_max
  if (!geometry_msgs__msg__Vector3__init(&msg->velocity_max)) {
    uav_msgs__msg__ConsensusTraj__fini(msg);
    return false;
  }
  // acceleration_min
  if (!geometry_msgs__msg__Vector3__init(&msg->acceleration_min)) {
    uav_msgs__msg__ConsensusTraj__fini(msg);
    return false;
  }
  // acceleration_max
  if (!geometry_msgs__msg__Vector3__init(&msg->acceleration_max)) {
    uav_msgs__msg__ConsensusTraj__fini(msg);
    return false;
  }
  return true;
}

void
uav_msgs__msg__ConsensusTraj__fini(uav_msgs__msg__ConsensusTraj * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // robot_name
  std_msgs__msg__String__fini(&msg->robot_name);
  // waypoints
  geometry_msgs__msg__Point__Sequence__fini(&msg->waypoints);
  // bern_coeffs
  geometry_msgs__msg__Point__Sequence__fini(&msg->bern_coeffs);
  // velocity_min
  geometry_msgs__msg__Vector3__fini(&msg->velocity_min);
  // velocity_max
  geometry_msgs__msg__Vector3__fini(&msg->velocity_max);
  // acceleration_min
  geometry_msgs__msg__Vector3__fini(&msg->acceleration_min);
  // acceleration_max
  geometry_msgs__msg__Vector3__fini(&msg->acceleration_max);
}

bool
uav_msgs__msg__ConsensusTraj__are_equal(const uav_msgs__msg__ConsensusTraj * lhs, const uav_msgs__msg__ConsensusTraj * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // robot_name
  if (!std_msgs__msg__String__are_equal(
      &(lhs->robot_name), &(rhs->robot_name)))
  {
    return false;
  }
  // waypoints
  if (!geometry_msgs__msg__Point__Sequence__are_equal(
      &(lhs->waypoints), &(rhs->waypoints)))
  {
    return false;
  }
  // bern_coeffs
  if (!geometry_msgs__msg__Point__Sequence__are_equal(
      &(lhs->bern_coeffs), &(rhs->bern_coeffs)))
  {
    return false;
  }
  // velocity_min
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->velocity_min), &(rhs->velocity_min)))
  {
    return false;
  }
  // velocity_max
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->velocity_max), &(rhs->velocity_max)))
  {
    return false;
  }
  // acceleration_min
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->acceleration_min), &(rhs->acceleration_min)))
  {
    return false;
  }
  // acceleration_max
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->acceleration_max), &(rhs->acceleration_max)))
  {
    return false;
  }
  return true;
}

bool
uav_msgs__msg__ConsensusTraj__copy(
  const uav_msgs__msg__ConsensusTraj * input,
  uav_msgs__msg__ConsensusTraj * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // robot_name
  if (!std_msgs__msg__String__copy(
      &(input->robot_name), &(output->robot_name)))
  {
    return false;
  }
  // waypoints
  if (!geometry_msgs__msg__Point__Sequence__copy(
      &(input->waypoints), &(output->waypoints)))
  {
    return false;
  }
  // bern_coeffs
  if (!geometry_msgs__msg__Point__Sequence__copy(
      &(input->bern_coeffs), &(output->bern_coeffs)))
  {
    return false;
  }
  // velocity_min
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->velocity_min), &(output->velocity_min)))
  {
    return false;
  }
  // velocity_max
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->velocity_max), &(output->velocity_max)))
  {
    return false;
  }
  // acceleration_min
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->acceleration_min), &(output->acceleration_min)))
  {
    return false;
  }
  // acceleration_max
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->acceleration_max), &(output->acceleration_max)))
  {
    return false;
  }
  return true;
}

uav_msgs__msg__ConsensusTraj *
uav_msgs__msg__ConsensusTraj__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uav_msgs__msg__ConsensusTraj * msg = (uav_msgs__msg__ConsensusTraj *)allocator.allocate(sizeof(uav_msgs__msg__ConsensusTraj), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(uav_msgs__msg__ConsensusTraj));
  bool success = uav_msgs__msg__ConsensusTraj__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
uav_msgs__msg__ConsensusTraj__destroy(uav_msgs__msg__ConsensusTraj * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    uav_msgs__msg__ConsensusTraj__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
uav_msgs__msg__ConsensusTraj__Sequence__init(uav_msgs__msg__ConsensusTraj__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uav_msgs__msg__ConsensusTraj * data = NULL;

  if (size) {
    data = (uav_msgs__msg__ConsensusTraj *)allocator.zero_allocate(size, sizeof(uav_msgs__msg__ConsensusTraj), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = uav_msgs__msg__ConsensusTraj__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        uav_msgs__msg__ConsensusTraj__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
uav_msgs__msg__ConsensusTraj__Sequence__fini(uav_msgs__msg__ConsensusTraj__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      uav_msgs__msg__ConsensusTraj__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

uav_msgs__msg__ConsensusTraj__Sequence *
uav_msgs__msg__ConsensusTraj__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uav_msgs__msg__ConsensusTraj__Sequence * array = (uav_msgs__msg__ConsensusTraj__Sequence *)allocator.allocate(sizeof(uav_msgs__msg__ConsensusTraj__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = uav_msgs__msg__ConsensusTraj__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
uav_msgs__msg__ConsensusTraj__Sequence__destroy(uav_msgs__msg__ConsensusTraj__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    uav_msgs__msg__ConsensusTraj__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
uav_msgs__msg__ConsensusTraj__Sequence__are_equal(const uav_msgs__msg__ConsensusTraj__Sequence * lhs, const uav_msgs__msg__ConsensusTraj__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!uav_msgs__msg__ConsensusTraj__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
uav_msgs__msg__ConsensusTraj__Sequence__copy(
  const uav_msgs__msg__ConsensusTraj__Sequence * input,
  uav_msgs__msg__ConsensusTraj__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(uav_msgs__msg__ConsensusTraj);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    uav_msgs__msg__ConsensusTraj * data =
      (uav_msgs__msg__ConsensusTraj *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!uav_msgs__msg__ConsensusTraj__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          uav_msgs__msg__ConsensusTraj__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!uav_msgs__msg__ConsensusTraj__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
