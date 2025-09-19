// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from message_server:srv/GetLatestMessage.idl
// generated code does not contain a copyright notice

#ifndef MESSAGE_SERVER__SRV__DETAIL__GET_LATEST_MESSAGE__STRUCT_H_
#define MESSAGE_SERVER__SRV__DETAIL__GET_LATEST_MESSAGE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/GetLatestMessage in the package message_server.
typedef struct message_server__srv__GetLatestMessage_Request
{
  uint8_t structure_needs_at_least_one_member;
} message_server__srv__GetLatestMessage_Request;

// Struct for a sequence of message_server__srv__GetLatestMessage_Request.
typedef struct message_server__srv__GetLatestMessage_Request__Sequence
{
  message_server__srv__GetLatestMessage_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} message_server__srv__GetLatestMessage_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'latest_message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/GetLatestMessage in the package message_server.
typedef struct message_server__srv__GetLatestMessage_Response
{
  rosidl_runtime_c__String latest_message;
} message_server__srv__GetLatestMessage_Response;

// Struct for a sequence of message_server__srv__GetLatestMessage_Response.
typedef struct message_server__srv__GetLatestMessage_Response__Sequence
{
  message_server__srv__GetLatestMessage_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} message_server__srv__GetLatestMessage_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MESSAGE_SERVER__SRV__DETAIL__GET_LATEST_MESSAGE__STRUCT_H_
