// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from message_server:srv/GetLatestMessage.idl
// generated code does not contain a copyright notice
#include "message_server/srv/detail/get_latest_message__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
message_server__srv__GetLatestMessage_Request__init(message_server__srv__GetLatestMessage_Request * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
message_server__srv__GetLatestMessage_Request__fini(message_server__srv__GetLatestMessage_Request * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
message_server__srv__GetLatestMessage_Request__are_equal(const message_server__srv__GetLatestMessage_Request * lhs, const message_server__srv__GetLatestMessage_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
message_server__srv__GetLatestMessage_Request__copy(
  const message_server__srv__GetLatestMessage_Request * input,
  message_server__srv__GetLatestMessage_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

message_server__srv__GetLatestMessage_Request *
message_server__srv__GetLatestMessage_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  message_server__srv__GetLatestMessage_Request * msg = (message_server__srv__GetLatestMessage_Request *)allocator.allocate(sizeof(message_server__srv__GetLatestMessage_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(message_server__srv__GetLatestMessage_Request));
  bool success = message_server__srv__GetLatestMessage_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
message_server__srv__GetLatestMessage_Request__destroy(message_server__srv__GetLatestMessage_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    message_server__srv__GetLatestMessage_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
message_server__srv__GetLatestMessage_Request__Sequence__init(message_server__srv__GetLatestMessage_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  message_server__srv__GetLatestMessage_Request * data = NULL;

  if (size) {
    data = (message_server__srv__GetLatestMessage_Request *)allocator.zero_allocate(size, sizeof(message_server__srv__GetLatestMessage_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = message_server__srv__GetLatestMessage_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        message_server__srv__GetLatestMessage_Request__fini(&data[i - 1]);
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
message_server__srv__GetLatestMessage_Request__Sequence__fini(message_server__srv__GetLatestMessage_Request__Sequence * array)
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
      message_server__srv__GetLatestMessage_Request__fini(&array->data[i]);
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

message_server__srv__GetLatestMessage_Request__Sequence *
message_server__srv__GetLatestMessage_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  message_server__srv__GetLatestMessage_Request__Sequence * array = (message_server__srv__GetLatestMessage_Request__Sequence *)allocator.allocate(sizeof(message_server__srv__GetLatestMessage_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = message_server__srv__GetLatestMessage_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
message_server__srv__GetLatestMessage_Request__Sequence__destroy(message_server__srv__GetLatestMessage_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    message_server__srv__GetLatestMessage_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
message_server__srv__GetLatestMessage_Request__Sequence__are_equal(const message_server__srv__GetLatestMessage_Request__Sequence * lhs, const message_server__srv__GetLatestMessage_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!message_server__srv__GetLatestMessage_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
message_server__srv__GetLatestMessage_Request__Sequence__copy(
  const message_server__srv__GetLatestMessage_Request__Sequence * input,
  message_server__srv__GetLatestMessage_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(message_server__srv__GetLatestMessage_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    message_server__srv__GetLatestMessage_Request * data =
      (message_server__srv__GetLatestMessage_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!message_server__srv__GetLatestMessage_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          message_server__srv__GetLatestMessage_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!message_server__srv__GetLatestMessage_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `latest_message`
#include "rosidl_runtime_c/string_functions.h"

bool
message_server__srv__GetLatestMessage_Response__init(message_server__srv__GetLatestMessage_Response * msg)
{
  if (!msg) {
    return false;
  }
  // latest_message
  if (!rosidl_runtime_c__String__init(&msg->latest_message)) {
    message_server__srv__GetLatestMessage_Response__fini(msg);
    return false;
  }
  return true;
}

void
message_server__srv__GetLatestMessage_Response__fini(message_server__srv__GetLatestMessage_Response * msg)
{
  if (!msg) {
    return;
  }
  // latest_message
  rosidl_runtime_c__String__fini(&msg->latest_message);
}

bool
message_server__srv__GetLatestMessage_Response__are_equal(const message_server__srv__GetLatestMessage_Response * lhs, const message_server__srv__GetLatestMessage_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // latest_message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->latest_message), &(rhs->latest_message)))
  {
    return false;
  }
  return true;
}

bool
message_server__srv__GetLatestMessage_Response__copy(
  const message_server__srv__GetLatestMessage_Response * input,
  message_server__srv__GetLatestMessage_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // latest_message
  if (!rosidl_runtime_c__String__copy(
      &(input->latest_message), &(output->latest_message)))
  {
    return false;
  }
  return true;
}

message_server__srv__GetLatestMessage_Response *
message_server__srv__GetLatestMessage_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  message_server__srv__GetLatestMessage_Response * msg = (message_server__srv__GetLatestMessage_Response *)allocator.allocate(sizeof(message_server__srv__GetLatestMessage_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(message_server__srv__GetLatestMessage_Response));
  bool success = message_server__srv__GetLatestMessage_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
message_server__srv__GetLatestMessage_Response__destroy(message_server__srv__GetLatestMessage_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    message_server__srv__GetLatestMessage_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
message_server__srv__GetLatestMessage_Response__Sequence__init(message_server__srv__GetLatestMessage_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  message_server__srv__GetLatestMessage_Response * data = NULL;

  if (size) {
    data = (message_server__srv__GetLatestMessage_Response *)allocator.zero_allocate(size, sizeof(message_server__srv__GetLatestMessage_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = message_server__srv__GetLatestMessage_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        message_server__srv__GetLatestMessage_Response__fini(&data[i - 1]);
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
message_server__srv__GetLatestMessage_Response__Sequence__fini(message_server__srv__GetLatestMessage_Response__Sequence * array)
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
      message_server__srv__GetLatestMessage_Response__fini(&array->data[i]);
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

message_server__srv__GetLatestMessage_Response__Sequence *
message_server__srv__GetLatestMessage_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  message_server__srv__GetLatestMessage_Response__Sequence * array = (message_server__srv__GetLatestMessage_Response__Sequence *)allocator.allocate(sizeof(message_server__srv__GetLatestMessage_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = message_server__srv__GetLatestMessage_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
message_server__srv__GetLatestMessage_Response__Sequence__destroy(message_server__srv__GetLatestMessage_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    message_server__srv__GetLatestMessage_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
message_server__srv__GetLatestMessage_Response__Sequence__are_equal(const message_server__srv__GetLatestMessage_Response__Sequence * lhs, const message_server__srv__GetLatestMessage_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!message_server__srv__GetLatestMessage_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
message_server__srv__GetLatestMessage_Response__Sequence__copy(
  const message_server__srv__GetLatestMessage_Response__Sequence * input,
  message_server__srv__GetLatestMessage_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(message_server__srv__GetLatestMessage_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    message_server__srv__GetLatestMessage_Response * data =
      (message_server__srv__GetLatestMessage_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!message_server__srv__GetLatestMessage_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          message_server__srv__GetLatestMessage_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!message_server__srv__GetLatestMessage_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
