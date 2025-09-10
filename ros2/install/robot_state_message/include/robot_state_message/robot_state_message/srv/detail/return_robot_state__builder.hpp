// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_state_message:srv/ReturnRobotState.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_STATE_MESSAGE__SRV__DETAIL__RETURN_ROBOT_STATE__BUILDER_HPP_
#define ROBOT_STATE_MESSAGE__SRV__DETAIL__RETURN_ROBOT_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_state_message/srv/detail/return_robot_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_state_message
{

namespace srv
{

namespace builder
{

class Init_ReturnRobotState_Request_time
{
public:
  explicit Init_ReturnRobotState_Request_time(::robot_state_message::srv::ReturnRobotState_Request & msg)
  : msg_(msg)
  {}
  ::robot_state_message::srv::ReturnRobotState_Request time(::robot_state_message::srv::ReturnRobotState_Request::_time_type arg)
  {
    msg_.time = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_state_message::srv::ReturnRobotState_Request msg_;
};

class Init_ReturnRobotState_Request_sm_state
{
public:
  explicit Init_ReturnRobotState_Request_sm_state(::robot_state_message::srv::ReturnRobotState_Request & msg)
  : msg_(msg)
  {}
  Init_ReturnRobotState_Request_time sm_state(::robot_state_message::srv::ReturnRobotState_Request::_sm_state_type arg)
  {
    msg_.sm_state = std::move(arg);
    return Init_ReturnRobotState_Request_time(msg_);
  }

private:
  ::robot_state_message::srv::ReturnRobotState_Request msg_;
};

class Init_ReturnRobotState_Request_gripper_pos
{
public:
  explicit Init_ReturnRobotState_Request_gripper_pos(::robot_state_message::srv::ReturnRobotState_Request & msg)
  : msg_(msg)
  {}
  Init_ReturnRobotState_Request_sm_state gripper_pos(::robot_state_message::srv::ReturnRobotState_Request::_gripper_pos_type arg)
  {
    msg_.gripper_pos = std::move(arg);
    return Init_ReturnRobotState_Request_sm_state(msg_);
  }

private:
  ::robot_state_message::srv::ReturnRobotState_Request msg_;
};

class Init_ReturnRobotState_Request_joint_pos
{
public:
  Init_ReturnRobotState_Request_joint_pos()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ReturnRobotState_Request_gripper_pos joint_pos(::robot_state_message::srv::ReturnRobotState_Request::_joint_pos_type arg)
  {
    msg_.joint_pos = std::move(arg);
    return Init_ReturnRobotState_Request_gripper_pos(msg_);
  }

private:
  ::robot_state_message::srv::ReturnRobotState_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_state_message::srv::ReturnRobotState_Request>()
{
  return robot_state_message::srv::builder::Init_ReturnRobotState_Request_joint_pos();
}

}  // namespace robot_state_message


namespace robot_state_message
{

namespace srv
{

namespace builder
{

class Init_ReturnRobotState_Response_time
{
public:
  explicit Init_ReturnRobotState_Response_time(::robot_state_message::srv::ReturnRobotState_Response & msg)
  : msg_(msg)
  {}
  ::robot_state_message::srv::ReturnRobotState_Response time(::robot_state_message::srv::ReturnRobotState_Response::_time_type arg)
  {
    msg_.time = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_state_message::srv::ReturnRobotState_Response msg_;
};

class Init_ReturnRobotState_Response_sm_state
{
public:
  explicit Init_ReturnRobotState_Response_sm_state(::robot_state_message::srv::ReturnRobotState_Response & msg)
  : msg_(msg)
  {}
  Init_ReturnRobotState_Response_time sm_state(::robot_state_message::srv::ReturnRobotState_Response::_sm_state_type arg)
  {
    msg_.sm_state = std::move(arg);
    return Init_ReturnRobotState_Response_time(msg_);
  }

private:
  ::robot_state_message::srv::ReturnRobotState_Response msg_;
};

class Init_ReturnRobotState_Response_gripper_pos
{
public:
  explicit Init_ReturnRobotState_Response_gripper_pos(::robot_state_message::srv::ReturnRobotState_Response & msg)
  : msg_(msg)
  {}
  Init_ReturnRobotState_Response_sm_state gripper_pos(::robot_state_message::srv::ReturnRobotState_Response::_gripper_pos_type arg)
  {
    msg_.gripper_pos = std::move(arg);
    return Init_ReturnRobotState_Response_sm_state(msg_);
  }

private:
  ::robot_state_message::srv::ReturnRobotState_Response msg_;
};

class Init_ReturnRobotState_Response_joint_pos
{
public:
  Init_ReturnRobotState_Response_joint_pos()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ReturnRobotState_Response_gripper_pos joint_pos(::robot_state_message::srv::ReturnRobotState_Response::_joint_pos_type arg)
  {
    msg_.joint_pos = std::move(arg);
    return Init_ReturnRobotState_Response_gripper_pos(msg_);
  }

private:
  ::robot_state_message::srv::ReturnRobotState_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_state_message::srv::ReturnRobotState_Response>()
{
  return robot_state_message::srv::builder::Init_ReturnRobotState_Response_joint_pos();
}

}  // namespace robot_state_message

#endif  // ROBOT_STATE_MESSAGE__SRV__DETAIL__RETURN_ROBOT_STATE__BUILDER_HPP_
