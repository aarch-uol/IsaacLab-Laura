// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robot_state_message:msg/RobotStateMessage.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_STATE_MESSAGE__MSG__DETAIL__ROBOT_STATE_MESSAGE__TRAITS_HPP_
#define ROBOT_STATE_MESSAGE__MSG__DETAIL__ROBOT_STATE_MESSAGE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "robot_state_message/msg/detail/robot_state_message__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace robot_state_message
{

namespace msg
{

inline void to_flow_style_yaml(
  const RobotStateMessage & msg,
  std::ostream & out)
{
  out << "{";
  // member: joint_pos
  {
    if (msg.joint_pos.size() == 0) {
      out << "joint_pos: []";
    } else {
      out << "joint_pos: [";
      size_t pending_items = msg.joint_pos.size();
      for (auto item : msg.joint_pos) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: gripper_pos
  {
    if (msg.gripper_pos.size() == 0) {
      out << "gripper_pos: []";
    } else {
      out << "gripper_pos: [";
      size_t pending_items = msg.gripper_pos.size();
      for (auto item : msg.gripper_pos) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: sm_state
  {
    out << "sm_state: ";
    rosidl_generator_traits::value_to_yaml(msg.sm_state, out);
    out << ", ";
  }

  // member: time
  {
    out << "time: ";
    rosidl_generator_traits::value_to_yaml(msg.time, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RobotStateMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: joint_pos
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.joint_pos.size() == 0) {
      out << "joint_pos: []\n";
    } else {
      out << "joint_pos:\n";
      for (auto item : msg.joint_pos) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: gripper_pos
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.gripper_pos.size() == 0) {
      out << "gripper_pos: []\n";
    } else {
      out << "gripper_pos:\n";
      for (auto item : msg.gripper_pos) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: sm_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sm_state: ";
    rosidl_generator_traits::value_to_yaml(msg.sm_state, out);
    out << "\n";
  }

  // member: time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "time: ";
    rosidl_generator_traits::value_to_yaml(msg.time, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RobotStateMessage & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace robot_state_message

namespace rosidl_generator_traits
{

[[deprecated("use robot_state_message::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robot_state_message::msg::RobotStateMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  robot_state_message::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robot_state_message::msg::to_yaml() instead")]]
inline std::string to_yaml(const robot_state_message::msg::RobotStateMessage & msg)
{
  return robot_state_message::msg::to_yaml(msg);
}

template<>
inline const char * data_type<robot_state_message::msg::RobotStateMessage>()
{
  return "robot_state_message::msg::RobotStateMessage";
}

template<>
inline const char * name<robot_state_message::msg::RobotStateMessage>()
{
  return "robot_state_message/msg/RobotStateMessage";
}

template<>
struct has_fixed_size<robot_state_message::msg::RobotStateMessage>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<robot_state_message::msg::RobotStateMessage>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<robot_state_message::msg::RobotStateMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROBOT_STATE_MESSAGE__MSG__DETAIL__ROBOT_STATE_MESSAGE__TRAITS_HPP_
