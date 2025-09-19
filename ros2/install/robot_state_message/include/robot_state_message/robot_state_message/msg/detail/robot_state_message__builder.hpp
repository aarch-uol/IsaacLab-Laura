// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_state_message:msg/RobotStateMessage.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_STATE_MESSAGE__MSG__DETAIL__ROBOT_STATE_MESSAGE__BUILDER_HPP_
#define ROBOT_STATE_MESSAGE__MSG__DETAIL__ROBOT_STATE_MESSAGE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_state_message/msg/detail/robot_state_message__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_state_message
{

namespace msg
{

namespace builder
{

class Init_RobotStateMessage_time
{
public:
  explicit Init_RobotStateMessage_time(::robot_state_message::msg::RobotStateMessage & msg)
  : msg_(msg)
  {}
  ::robot_state_message::msg::RobotStateMessage time(::robot_state_message::msg::RobotStateMessage::_time_type arg)
  {
    msg_.time = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_state_message::msg::RobotStateMessage msg_;
};

class Init_RobotStateMessage_sm_state
{
public:
  explicit Init_RobotStateMessage_sm_state(::robot_state_message::msg::RobotStateMessage & msg)
  : msg_(msg)
  {}
  Init_RobotStateMessage_time sm_state(::robot_state_message::msg::RobotStateMessage::_sm_state_type arg)
  {
    msg_.sm_state = std::move(arg);
    return Init_RobotStateMessage_time(msg_);
  }

private:
  ::robot_state_message::msg::RobotStateMessage msg_;
};

class Init_RobotStateMessage_gripper_pos
{
public:
  explicit Init_RobotStateMessage_gripper_pos(::robot_state_message::msg::RobotStateMessage & msg)
  : msg_(msg)
  {}
  Init_RobotStateMessage_sm_state gripper_pos(::robot_state_message::msg::RobotStateMessage::_gripper_pos_type arg)
  {
    msg_.gripper_pos = std::move(arg);
    return Init_RobotStateMessage_sm_state(msg_);
  }

private:
  ::robot_state_message::msg::RobotStateMessage msg_;
};

class Init_RobotStateMessage_joint_pos
{
public:
  Init_RobotStateMessage_joint_pos()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotStateMessage_gripper_pos joint_pos(::robot_state_message::msg::RobotStateMessage::_joint_pos_type arg)
  {
    msg_.joint_pos = std::move(arg);
    return Init_RobotStateMessage_gripper_pos(msg_);
  }

private:
  ::robot_state_message::msg::RobotStateMessage msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_state_message::msg::RobotStateMessage>()
{
  return robot_state_message::msg::builder::Init_RobotStateMessage_joint_pos();
}

}  // namespace robot_state_message

#endif  // ROBOT_STATE_MESSAGE__MSG__DETAIL__ROBOT_STATE_MESSAGE__BUILDER_HPP_
