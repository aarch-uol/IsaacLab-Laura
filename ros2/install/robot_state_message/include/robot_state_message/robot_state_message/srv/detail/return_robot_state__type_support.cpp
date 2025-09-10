// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from robot_state_message:srv/ReturnRobotState.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "robot_state_message/srv/detail/return_robot_state__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace robot_state_message
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void ReturnRobotState_Request_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) robot_state_message::srv::ReturnRobotState_Request(_init);
}

void ReturnRobotState_Request_fini_function(void * message_memory)
{
  auto typed_message = static_cast<robot_state_message::srv::ReturnRobotState_Request *>(message_memory);
  typed_message->~ReturnRobotState_Request();
}

size_t size_function__ReturnRobotState_Request__joint_pos(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__ReturnRobotState_Request__joint_pos(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__ReturnRobotState_Request__joint_pos(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void fetch_function__ReturnRobotState_Request__joint_pos(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__ReturnRobotState_Request__joint_pos(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__ReturnRobotState_Request__joint_pos(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__ReturnRobotState_Request__joint_pos(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

void resize_function__ReturnRobotState_Request__joint_pos(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__ReturnRobotState_Request__gripper_pos(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__ReturnRobotState_Request__gripper_pos(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__ReturnRobotState_Request__gripper_pos(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void fetch_function__ReturnRobotState_Request__gripper_pos(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__ReturnRobotState_Request__gripper_pos(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__ReturnRobotState_Request__gripper_pos(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__ReturnRobotState_Request__gripper_pos(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

void resize_function__ReturnRobotState_Request__gripper_pos(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember ReturnRobotState_Request_message_member_array[4] = {
  {
    "joint_pos",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_state_message::srv::ReturnRobotState_Request, joint_pos),  // bytes offset in struct
    nullptr,  // default value
    size_function__ReturnRobotState_Request__joint_pos,  // size() function pointer
    get_const_function__ReturnRobotState_Request__joint_pos,  // get_const(index) function pointer
    get_function__ReturnRobotState_Request__joint_pos,  // get(index) function pointer
    fetch_function__ReturnRobotState_Request__joint_pos,  // fetch(index, &value) function pointer
    assign_function__ReturnRobotState_Request__joint_pos,  // assign(index, value) function pointer
    resize_function__ReturnRobotState_Request__joint_pos  // resize(index) function pointer
  },
  {
    "gripper_pos",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_state_message::srv::ReturnRobotState_Request, gripper_pos),  // bytes offset in struct
    nullptr,  // default value
    size_function__ReturnRobotState_Request__gripper_pos,  // size() function pointer
    get_const_function__ReturnRobotState_Request__gripper_pos,  // get_const(index) function pointer
    get_function__ReturnRobotState_Request__gripper_pos,  // get(index) function pointer
    fetch_function__ReturnRobotState_Request__gripper_pos,  // fetch(index, &value) function pointer
    assign_function__ReturnRobotState_Request__gripper_pos,  // assign(index, value) function pointer
    resize_function__ReturnRobotState_Request__gripper_pos  // resize(index) function pointer
  },
  {
    "sm_state",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_state_message::srv::ReturnRobotState_Request, sm_state),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "time",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_state_message::srv::ReturnRobotState_Request, time),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers ReturnRobotState_Request_message_members = {
  "robot_state_message::srv",  // message namespace
  "ReturnRobotState_Request",  // message name
  4,  // number of fields
  sizeof(robot_state_message::srv::ReturnRobotState_Request),
  ReturnRobotState_Request_message_member_array,  // message members
  ReturnRobotState_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  ReturnRobotState_Request_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t ReturnRobotState_Request_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &ReturnRobotState_Request_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace robot_state_message


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<robot_state_message::srv::ReturnRobotState_Request>()
{
  return &::robot_state_message::srv::rosidl_typesupport_introspection_cpp::ReturnRobotState_Request_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, robot_state_message, srv, ReturnRobotState_Request)() {
  return &::robot_state_message::srv::rosidl_typesupport_introspection_cpp::ReturnRobotState_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "robot_state_message/srv/detail/return_robot_state__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace robot_state_message
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void ReturnRobotState_Response_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) robot_state_message::srv::ReturnRobotState_Response(_init);
}

void ReturnRobotState_Response_fini_function(void * message_memory)
{
  auto typed_message = static_cast<robot_state_message::srv::ReturnRobotState_Response *>(message_memory);
  typed_message->~ReturnRobotState_Response();
}

size_t size_function__ReturnRobotState_Response__joint_pos(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__ReturnRobotState_Response__joint_pos(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__ReturnRobotState_Response__joint_pos(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void fetch_function__ReturnRobotState_Response__joint_pos(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__ReturnRobotState_Response__joint_pos(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__ReturnRobotState_Response__joint_pos(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__ReturnRobotState_Response__joint_pos(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

void resize_function__ReturnRobotState_Response__joint_pos(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__ReturnRobotState_Response__gripper_pos(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__ReturnRobotState_Response__gripper_pos(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__ReturnRobotState_Response__gripper_pos(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void fetch_function__ReturnRobotState_Response__gripper_pos(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__ReturnRobotState_Response__gripper_pos(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__ReturnRobotState_Response__gripper_pos(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__ReturnRobotState_Response__gripper_pos(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

void resize_function__ReturnRobotState_Response__gripper_pos(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember ReturnRobotState_Response_message_member_array[4] = {
  {
    "joint_pos",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_state_message::srv::ReturnRobotState_Response, joint_pos),  // bytes offset in struct
    nullptr,  // default value
    size_function__ReturnRobotState_Response__joint_pos,  // size() function pointer
    get_const_function__ReturnRobotState_Response__joint_pos,  // get_const(index) function pointer
    get_function__ReturnRobotState_Response__joint_pos,  // get(index) function pointer
    fetch_function__ReturnRobotState_Response__joint_pos,  // fetch(index, &value) function pointer
    assign_function__ReturnRobotState_Response__joint_pos,  // assign(index, value) function pointer
    resize_function__ReturnRobotState_Response__joint_pos  // resize(index) function pointer
  },
  {
    "gripper_pos",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_state_message::srv::ReturnRobotState_Response, gripper_pos),  // bytes offset in struct
    nullptr,  // default value
    size_function__ReturnRobotState_Response__gripper_pos,  // size() function pointer
    get_const_function__ReturnRobotState_Response__gripper_pos,  // get_const(index) function pointer
    get_function__ReturnRobotState_Response__gripper_pos,  // get(index) function pointer
    fetch_function__ReturnRobotState_Response__gripper_pos,  // fetch(index, &value) function pointer
    assign_function__ReturnRobotState_Response__gripper_pos,  // assign(index, value) function pointer
    resize_function__ReturnRobotState_Response__gripper_pos  // resize(index) function pointer
  },
  {
    "sm_state",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_state_message::srv::ReturnRobotState_Response, sm_state),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "time",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_state_message::srv::ReturnRobotState_Response, time),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers ReturnRobotState_Response_message_members = {
  "robot_state_message::srv",  // message namespace
  "ReturnRobotState_Response",  // message name
  4,  // number of fields
  sizeof(robot_state_message::srv::ReturnRobotState_Response),
  ReturnRobotState_Response_message_member_array,  // message members
  ReturnRobotState_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  ReturnRobotState_Response_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t ReturnRobotState_Response_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &ReturnRobotState_Response_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace robot_state_message


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<robot_state_message::srv::ReturnRobotState_Response>()
{
  return &::robot_state_message::srv::rosidl_typesupport_introspection_cpp::ReturnRobotState_Response_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, robot_state_message, srv, ReturnRobotState_Response)() {
  return &::robot_state_message::srv::rosidl_typesupport_introspection_cpp::ReturnRobotState_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"
// already included above
// #include "robot_state_message/srv/detail/return_robot_state__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/service_type_support_decl.hpp"

namespace robot_state_message
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

// this is intentionally not const to allow initialization later to prevent an initialization race
static ::rosidl_typesupport_introspection_cpp::ServiceMembers ReturnRobotState_service_members = {
  "robot_state_message::srv",  // service namespace
  "ReturnRobotState",  // service name
  // these two fields are initialized below on the first access
  // see get_service_type_support_handle<robot_state_message::srv::ReturnRobotState>()
  nullptr,  // request message
  nullptr  // response message
};

static const rosidl_service_type_support_t ReturnRobotState_service_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &ReturnRobotState_service_members,
  get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace robot_state_message


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<robot_state_message::srv::ReturnRobotState>()
{
  // get a handle to the value to be returned
  auto service_type_support =
    &::robot_state_message::srv::rosidl_typesupport_introspection_cpp::ReturnRobotState_service_type_support_handle;
  // get a non-const and properly typed version of the data void *
  auto service_members = const_cast<::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
    static_cast<const ::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
      service_type_support->data));
  // make sure that both the request_members_ and the response_members_ are initialized
  // if they are not, initialize them
  if (
    service_members->request_members_ == nullptr ||
    service_members->response_members_ == nullptr)
  {
    // initialize the request_members_ with the static function from the external library
    service_members->request_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::robot_state_message::srv::ReturnRobotState_Request
      >()->data
      );
    // initialize the response_members_ with the static function from the external library
    service_members->response_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::robot_state_message::srv::ReturnRobotState_Response
      >()->data
      );
  }
  // finally return the properly initialized service_type_support handle
  return service_type_support;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, robot_state_message, srv, ReturnRobotState)() {
  return ::rosidl_typesupport_introspection_cpp::get_service_type_support_handle<robot_state_message::srv::ReturnRobotState>();
}

#ifdef __cplusplus
}
#endif
