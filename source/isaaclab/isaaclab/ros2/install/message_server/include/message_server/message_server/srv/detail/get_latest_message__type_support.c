// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from message_server:srv/GetLatestMessage.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "message_server/srv/detail/get_latest_message__rosidl_typesupport_introspection_c.h"
#include "message_server/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "message_server/srv/detail/get_latest_message__functions.h"
#include "message_server/srv/detail/get_latest_message__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void message_server__srv__GetLatestMessage_Request__rosidl_typesupport_introspection_c__GetLatestMessage_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  message_server__srv__GetLatestMessage_Request__init(message_memory);
}

void message_server__srv__GetLatestMessage_Request__rosidl_typesupport_introspection_c__GetLatestMessage_Request_fini_function(void * message_memory)
{
  message_server__srv__GetLatestMessage_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember message_server__srv__GetLatestMessage_Request__rosidl_typesupport_introspection_c__GetLatestMessage_Request_message_member_array[1] = {
  {
    "structure_needs_at_least_one_member",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(message_server__srv__GetLatestMessage_Request, structure_needs_at_least_one_member),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers message_server__srv__GetLatestMessage_Request__rosidl_typesupport_introspection_c__GetLatestMessage_Request_message_members = {
  "message_server__srv",  // message namespace
  "GetLatestMessage_Request",  // message name
  1,  // number of fields
  sizeof(message_server__srv__GetLatestMessage_Request),
  message_server__srv__GetLatestMessage_Request__rosidl_typesupport_introspection_c__GetLatestMessage_Request_message_member_array,  // message members
  message_server__srv__GetLatestMessage_Request__rosidl_typesupport_introspection_c__GetLatestMessage_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  message_server__srv__GetLatestMessage_Request__rosidl_typesupport_introspection_c__GetLatestMessage_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t message_server__srv__GetLatestMessage_Request__rosidl_typesupport_introspection_c__GetLatestMessage_Request_message_type_support_handle = {
  0,
  &message_server__srv__GetLatestMessage_Request__rosidl_typesupport_introspection_c__GetLatestMessage_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_message_server
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, message_server, srv, GetLatestMessage_Request)() {
  if (!message_server__srv__GetLatestMessage_Request__rosidl_typesupport_introspection_c__GetLatestMessage_Request_message_type_support_handle.typesupport_identifier) {
    message_server__srv__GetLatestMessage_Request__rosidl_typesupport_introspection_c__GetLatestMessage_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &message_server__srv__GetLatestMessage_Request__rosidl_typesupport_introspection_c__GetLatestMessage_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "message_server/srv/detail/get_latest_message__rosidl_typesupport_introspection_c.h"
// already included above
// #include "message_server/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "message_server/srv/detail/get_latest_message__functions.h"
// already included above
// #include "message_server/srv/detail/get_latest_message__struct.h"


// Include directives for member types
// Member `latest_message`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void message_server__srv__GetLatestMessage_Response__rosidl_typesupport_introspection_c__GetLatestMessage_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  message_server__srv__GetLatestMessage_Response__init(message_memory);
}

void message_server__srv__GetLatestMessage_Response__rosidl_typesupport_introspection_c__GetLatestMessage_Response_fini_function(void * message_memory)
{
  message_server__srv__GetLatestMessage_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember message_server__srv__GetLatestMessage_Response__rosidl_typesupport_introspection_c__GetLatestMessage_Response_message_member_array[1] = {
  {
    "latest_message",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(message_server__srv__GetLatestMessage_Response, latest_message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers message_server__srv__GetLatestMessage_Response__rosidl_typesupport_introspection_c__GetLatestMessage_Response_message_members = {
  "message_server__srv",  // message namespace
  "GetLatestMessage_Response",  // message name
  1,  // number of fields
  sizeof(message_server__srv__GetLatestMessage_Response),
  message_server__srv__GetLatestMessage_Response__rosidl_typesupport_introspection_c__GetLatestMessage_Response_message_member_array,  // message members
  message_server__srv__GetLatestMessage_Response__rosidl_typesupport_introspection_c__GetLatestMessage_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  message_server__srv__GetLatestMessage_Response__rosidl_typesupport_introspection_c__GetLatestMessage_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t message_server__srv__GetLatestMessage_Response__rosidl_typesupport_introspection_c__GetLatestMessage_Response_message_type_support_handle = {
  0,
  &message_server__srv__GetLatestMessage_Response__rosidl_typesupport_introspection_c__GetLatestMessage_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_message_server
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, message_server, srv, GetLatestMessage_Response)() {
  if (!message_server__srv__GetLatestMessage_Response__rosidl_typesupport_introspection_c__GetLatestMessage_Response_message_type_support_handle.typesupport_identifier) {
    message_server__srv__GetLatestMessage_Response__rosidl_typesupport_introspection_c__GetLatestMessage_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &message_server__srv__GetLatestMessage_Response__rosidl_typesupport_introspection_c__GetLatestMessage_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "message_server/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "message_server/srv/detail/get_latest_message__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers message_server__srv__detail__get_latest_message__rosidl_typesupport_introspection_c__GetLatestMessage_service_members = {
  "message_server__srv",  // service namespace
  "GetLatestMessage",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // message_server__srv__detail__get_latest_message__rosidl_typesupport_introspection_c__GetLatestMessage_Request_message_type_support_handle,
  NULL  // response message
  // message_server__srv__detail__get_latest_message__rosidl_typesupport_introspection_c__GetLatestMessage_Response_message_type_support_handle
};

static rosidl_service_type_support_t message_server__srv__detail__get_latest_message__rosidl_typesupport_introspection_c__GetLatestMessage_service_type_support_handle = {
  0,
  &message_server__srv__detail__get_latest_message__rosidl_typesupport_introspection_c__GetLatestMessage_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, message_server, srv, GetLatestMessage_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, message_server, srv, GetLatestMessage_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_message_server
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, message_server, srv, GetLatestMessage)() {
  if (!message_server__srv__detail__get_latest_message__rosidl_typesupport_introspection_c__GetLatestMessage_service_type_support_handle.typesupport_identifier) {
    message_server__srv__detail__get_latest_message__rosidl_typesupport_introspection_c__GetLatestMessage_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)message_server__srv__detail__get_latest_message__rosidl_typesupport_introspection_c__GetLatestMessage_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, message_server, srv, GetLatestMessage_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, message_server, srv, GetLatestMessage_Response)()->data;
  }

  return &message_server__srv__detail__get_latest_message__rosidl_typesupport_introspection_c__GetLatestMessage_service_type_support_handle;
}
