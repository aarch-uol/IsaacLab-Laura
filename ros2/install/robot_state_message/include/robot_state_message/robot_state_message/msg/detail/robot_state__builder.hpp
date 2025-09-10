// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_state_message:msg/RobotState.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_STATE_MESSAGE__MSG__DETAIL__ROBOT_STATE__BUILDER_HPP_
#define ROBOT_STATE_MESSAGE__MSG__DETAIL__ROBOT_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_state_message/msg/detail/robot_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_state_message
{

namespace msg
{

namespace builder
{

class Init_RobotState_time
{
public:
  explicit Init_RobotState_time(::robot_state_message::msg::RobotState & msg)
  : msg_(msg)
  {}
  ::robot_state_message::msg::RobotState time(::robot_state_message::msg::RobotState::_time_type arg)
  {
    msg_.time = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_state_message::msg::RobotState msg_;
};

class Init_RobotState_sm_state
{
public:
  explicit Init_RobotState_sm_state(::robot_state_message::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_time sm_state(::robot_state_message::msg::RobotState::_sm_state_type arg)
  {
    msg_.sm_state = std::move(arg);
    return Init_RobotState_time(msg_);
  }

private:
  ::robot_state_message::msg::RobotState msg_;
};

class Init_RobotState_gripper_pos
{
public:
  explicit Init_RobotState_gripper_pos(::robot_state_message::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_sm_state gripper_pos(::robot_state_message::msg::RobotState::_gripper_pos_type arg)
  {
    msg_.gripper_pos = std::move(arg);
    return Init_RobotState_sm_state(msg_);
  }

private:
  ::robot_state_message::msg::RobotState msg_;
};

class Init_RobotState_joint_pos
{
public:
  Init_RobotState_joint_pos()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotState_gripper_pos joint_pos(::robot_state_message::msg::RobotState::_joint_pos_type arg)
  {
    msg_.joint_pos = std::move(arg);
    return Init_RobotState_gripper_pos(msg_);
  }

private:
  ::robot_state_message::msg::RobotState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_state_message::msg::RobotState>()
{
  return robot_state_message::msg::builder::Init_RobotState_joint_pos();
}

}  // namespace robot_state_message

#endif  // ROBOT_STATE_MESSAGE__MSG__DETAIL__ROBOT_STATE__BUILDER_HPP_
