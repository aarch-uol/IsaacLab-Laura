// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robot_state_message:srv/ReturnRobotState.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_STATE_MESSAGE__SRV__DETAIL__RETURN_ROBOT_STATE__STRUCT_H_
#define ROBOT_STATE_MESSAGE__SRV__DETAIL__RETURN_ROBOT_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'joint_pos'
// Member 'gripper_pos'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in srv/ReturnRobotState in the package robot_state_message.
typedef struct robot_state_message__srv__ReturnRobotState_Request
{
  rosidl_runtime_c__float__Sequence joint_pos;
  rosidl_runtime_c__float__Sequence gripper_pos;
  int32_t sm_state;
  int32_t time;
} robot_state_message__srv__ReturnRobotState_Request;

// Struct for a sequence of robot_state_message__srv__ReturnRobotState_Request.
typedef struct robot_state_message__srv__ReturnRobotState_Request__Sequence
{
  robot_state_message__srv__ReturnRobotState_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_state_message__srv__ReturnRobotState_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'joint_pos'
// Member 'gripper_pos'
// already included above
// #include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in srv/ReturnRobotState in the package robot_state_message.
typedef struct robot_state_message__srv__ReturnRobotState_Response
{
  rosidl_runtime_c__float__Sequence joint_pos;
  rosidl_runtime_c__float__Sequence gripper_pos;
  int32_t sm_state;
  int32_t time;
} robot_state_message__srv__ReturnRobotState_Response;

// Struct for a sequence of robot_state_message__srv__ReturnRobotState_Response.
typedef struct robot_state_message__srv__ReturnRobotState_Response__Sequence
{
  robot_state_message__srv__ReturnRobotState_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_state_message__srv__ReturnRobotState_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_STATE_MESSAGE__SRV__DETAIL__RETURN_ROBOT_STATE__STRUCT_H_
