// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from robot_state_message:srv/ReturnRobotState.idl
// generated code does not contain a copyright notice
#include "robot_state_message/srv/detail/return_robot_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `joint_pos`
// Member `gripper_pos`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
robot_state_message__srv__ReturnRobotState_Request__init(robot_state_message__srv__ReturnRobotState_Request * msg)
{
  if (!msg) {
    return false;
  }
  // joint_pos
  if (!rosidl_runtime_c__float__Sequence__init(&msg->joint_pos, 0)) {
    robot_state_message__srv__ReturnRobotState_Request__fini(msg);
    return false;
  }
  // gripper_pos
  if (!rosidl_runtime_c__float__Sequence__init(&msg->gripper_pos, 0)) {
    robot_state_message__srv__ReturnRobotState_Request__fini(msg);
    return false;
  }
  // sm_state
  // time
  return true;
}

void
robot_state_message__srv__ReturnRobotState_Request__fini(robot_state_message__srv__ReturnRobotState_Request * msg)
{
  if (!msg) {
    return;
  }
  // joint_pos
  rosidl_runtime_c__float__Sequence__fini(&msg->joint_pos);
  // gripper_pos
  rosidl_runtime_c__float__Sequence__fini(&msg->gripper_pos);
  // sm_state
  // time
}

bool
robot_state_message__srv__ReturnRobotState_Request__are_equal(const robot_state_message__srv__ReturnRobotState_Request * lhs, const robot_state_message__srv__ReturnRobotState_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // joint_pos
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->joint_pos), &(rhs->joint_pos)))
  {
    return false;
  }
  // gripper_pos
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->gripper_pos), &(rhs->gripper_pos)))
  {
    return false;
  }
  // sm_state
  if (lhs->sm_state != rhs->sm_state) {
    return false;
  }
  // time
  if (lhs->time != rhs->time) {
    return false;
  }
  return true;
}

bool
robot_state_message__srv__ReturnRobotState_Request__copy(
  const robot_state_message__srv__ReturnRobotState_Request * input,
  robot_state_message__srv__ReturnRobotState_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // joint_pos
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->joint_pos), &(output->joint_pos)))
  {
    return false;
  }
  // gripper_pos
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->gripper_pos), &(output->gripper_pos)))
  {
    return false;
  }
  // sm_state
  output->sm_state = input->sm_state;
  // time
  output->time = input->time;
  return true;
}

robot_state_message__srv__ReturnRobotState_Request *
robot_state_message__srv__ReturnRobotState_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_state_message__srv__ReturnRobotState_Request * msg = (robot_state_message__srv__ReturnRobotState_Request *)allocator.allocate(sizeof(robot_state_message__srv__ReturnRobotState_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robot_state_message__srv__ReturnRobotState_Request));
  bool success = robot_state_message__srv__ReturnRobotState_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robot_state_message__srv__ReturnRobotState_Request__destroy(robot_state_message__srv__ReturnRobotState_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robot_state_message__srv__ReturnRobotState_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robot_state_message__srv__ReturnRobotState_Request__Sequence__init(robot_state_message__srv__ReturnRobotState_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_state_message__srv__ReturnRobotState_Request * data = NULL;

  if (size) {
    data = (robot_state_message__srv__ReturnRobotState_Request *)allocator.zero_allocate(size, sizeof(robot_state_message__srv__ReturnRobotState_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robot_state_message__srv__ReturnRobotState_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robot_state_message__srv__ReturnRobotState_Request__fini(&data[i - 1]);
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
robot_state_message__srv__ReturnRobotState_Request__Sequence__fini(robot_state_message__srv__ReturnRobotState_Request__Sequence * array)
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
      robot_state_message__srv__ReturnRobotState_Request__fini(&array->data[i]);
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

robot_state_message__srv__ReturnRobotState_Request__Sequence *
robot_state_message__srv__ReturnRobotState_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_state_message__srv__ReturnRobotState_Request__Sequence * array = (robot_state_message__srv__ReturnRobotState_Request__Sequence *)allocator.allocate(sizeof(robot_state_message__srv__ReturnRobotState_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robot_state_message__srv__ReturnRobotState_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robot_state_message__srv__ReturnRobotState_Request__Sequence__destroy(robot_state_message__srv__ReturnRobotState_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robot_state_message__srv__ReturnRobotState_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robot_state_message__srv__ReturnRobotState_Request__Sequence__are_equal(const robot_state_message__srv__ReturnRobotState_Request__Sequence * lhs, const robot_state_message__srv__ReturnRobotState_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robot_state_message__srv__ReturnRobotState_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robot_state_message__srv__ReturnRobotState_Request__Sequence__copy(
  const robot_state_message__srv__ReturnRobotState_Request__Sequence * input,
  robot_state_message__srv__ReturnRobotState_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robot_state_message__srv__ReturnRobotState_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robot_state_message__srv__ReturnRobotState_Request * data =
      (robot_state_message__srv__ReturnRobotState_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robot_state_message__srv__ReturnRobotState_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robot_state_message__srv__ReturnRobotState_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robot_state_message__srv__ReturnRobotState_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `joint_pos`
// Member `gripper_pos`
// already included above
// #include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
robot_state_message__srv__ReturnRobotState_Response__init(robot_state_message__srv__ReturnRobotState_Response * msg)
{
  if (!msg) {
    return false;
  }
  // joint_pos
  if (!rosidl_runtime_c__float__Sequence__init(&msg->joint_pos, 0)) {
    robot_state_message__srv__ReturnRobotState_Response__fini(msg);
    return false;
  }
  // gripper_pos
  if (!rosidl_runtime_c__float__Sequence__init(&msg->gripper_pos, 0)) {
    robot_state_message__srv__ReturnRobotState_Response__fini(msg);
    return false;
  }
  // sm_state
  // time
  return true;
}

void
robot_state_message__srv__ReturnRobotState_Response__fini(robot_state_message__srv__ReturnRobotState_Response * msg)
{
  if (!msg) {
    return;
  }
  // joint_pos
  rosidl_runtime_c__float__Sequence__fini(&msg->joint_pos);
  // gripper_pos
  rosidl_runtime_c__float__Sequence__fini(&msg->gripper_pos);
  // sm_state
  // time
}

bool
robot_state_message__srv__ReturnRobotState_Response__are_equal(const robot_state_message__srv__ReturnRobotState_Response * lhs, const robot_state_message__srv__ReturnRobotState_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // joint_pos
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->joint_pos), &(rhs->joint_pos)))
  {
    return false;
  }
  // gripper_pos
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->gripper_pos), &(rhs->gripper_pos)))
  {
    return false;
  }
  // sm_state
  if (lhs->sm_state != rhs->sm_state) {
    return false;
  }
  // time
  if (lhs->time != rhs->time) {
    return false;
  }
  return true;
}

bool
robot_state_message__srv__ReturnRobotState_Response__copy(
  const robot_state_message__srv__ReturnRobotState_Response * input,
  robot_state_message__srv__ReturnRobotState_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // joint_pos
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->joint_pos), &(output->joint_pos)))
  {
    return false;
  }
  // gripper_pos
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->gripper_pos), &(output->gripper_pos)))
  {
    return false;
  }
  // sm_state
  output->sm_state = input->sm_state;
  // time
  output->time = input->time;
  return true;
}

robot_state_message__srv__ReturnRobotState_Response *
robot_state_message__srv__ReturnRobotState_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_state_message__srv__ReturnRobotState_Response * msg = (robot_state_message__srv__ReturnRobotState_Response *)allocator.allocate(sizeof(robot_state_message__srv__ReturnRobotState_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robot_state_message__srv__ReturnRobotState_Response));
  bool success = robot_state_message__srv__ReturnRobotState_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robot_state_message__srv__ReturnRobotState_Response__destroy(robot_state_message__srv__ReturnRobotState_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robot_state_message__srv__ReturnRobotState_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robot_state_message__srv__ReturnRobotState_Response__Sequence__init(robot_state_message__srv__ReturnRobotState_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_state_message__srv__ReturnRobotState_Response * data = NULL;

  if (size) {
    data = (robot_state_message__srv__ReturnRobotState_Response *)allocator.zero_allocate(size, sizeof(robot_state_message__srv__ReturnRobotState_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robot_state_message__srv__ReturnRobotState_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robot_state_message__srv__ReturnRobotState_Response__fini(&data[i - 1]);
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
robot_state_message__srv__ReturnRobotState_Response__Sequence__fini(robot_state_message__srv__ReturnRobotState_Response__Sequence * array)
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
      robot_state_message__srv__ReturnRobotState_Response__fini(&array->data[i]);
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

robot_state_message__srv__ReturnRobotState_Response__Sequence *
robot_state_message__srv__ReturnRobotState_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_state_message__srv__ReturnRobotState_Response__Sequence * array = (robot_state_message__srv__ReturnRobotState_Response__Sequence *)allocator.allocate(sizeof(robot_state_message__srv__ReturnRobotState_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robot_state_message__srv__ReturnRobotState_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robot_state_message__srv__ReturnRobotState_Response__Sequence__destroy(robot_state_message__srv__ReturnRobotState_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robot_state_message__srv__ReturnRobotState_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robot_state_message__srv__ReturnRobotState_Response__Sequence__are_equal(const robot_state_message__srv__ReturnRobotState_Response__Sequence * lhs, const robot_state_message__srv__ReturnRobotState_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robot_state_message__srv__ReturnRobotState_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robot_state_message__srv__ReturnRobotState_Response__Sequence__copy(
  const robot_state_message__srv__ReturnRobotState_Response__Sequence * input,
  robot_state_message__srv__ReturnRobotState_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robot_state_message__srv__ReturnRobotState_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robot_state_message__srv__ReturnRobotState_Response * data =
      (robot_state_message__srv__ReturnRobotState_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robot_state_message__srv__ReturnRobotState_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robot_state_message__srv__ReturnRobotState_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robot_state_message__srv__ReturnRobotState_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
