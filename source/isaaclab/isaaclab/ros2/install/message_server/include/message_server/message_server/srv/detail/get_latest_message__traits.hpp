// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from message_server:srv/GetLatestMessage.idl
// generated code does not contain a copyright notice

#ifndef MESSAGE_SERVER__SRV__DETAIL__GET_LATEST_MESSAGE__TRAITS_HPP_
#define MESSAGE_SERVER__SRV__DETAIL__GET_LATEST_MESSAGE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "message_server/srv/detail/get_latest_message__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace message_server
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetLatestMessage_Request & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetLatestMessage_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetLatestMessage_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace message_server

namespace rosidl_generator_traits
{

[[deprecated("use message_server::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const message_server::srv::GetLatestMessage_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  message_server::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use message_server::srv::to_yaml() instead")]]
inline std::string to_yaml(const message_server::srv::GetLatestMessage_Request & msg)
{
  return message_server::srv::to_yaml(msg);
}

template<>
inline const char * data_type<message_server::srv::GetLatestMessage_Request>()
{
  return "message_server::srv::GetLatestMessage_Request";
}

template<>
inline const char * name<message_server::srv::GetLatestMessage_Request>()
{
  return "message_server/srv/GetLatestMessage_Request";
}

template<>
struct has_fixed_size<message_server::srv::GetLatestMessage_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<message_server::srv::GetLatestMessage_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<message_server::srv::GetLatestMessage_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace message_server
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetLatestMessage_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: latest_message
  {
    out << "latest_message: ";
    rosidl_generator_traits::value_to_yaml(msg.latest_message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetLatestMessage_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: latest_message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "latest_message: ";
    rosidl_generator_traits::value_to_yaml(msg.latest_message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetLatestMessage_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace message_server

namespace rosidl_generator_traits
{

[[deprecated("use message_server::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const message_server::srv::GetLatestMessage_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  message_server::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use message_server::srv::to_yaml() instead")]]
inline std::string to_yaml(const message_server::srv::GetLatestMessage_Response & msg)
{
  return message_server::srv::to_yaml(msg);
}

template<>
inline const char * data_type<message_server::srv::GetLatestMessage_Response>()
{
  return "message_server::srv::GetLatestMessage_Response";
}

template<>
inline const char * name<message_server::srv::GetLatestMessage_Response>()
{
  return "message_server/srv/GetLatestMessage_Response";
}

template<>
struct has_fixed_size<message_server::srv::GetLatestMessage_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<message_server::srv::GetLatestMessage_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<message_server::srv::GetLatestMessage_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<message_server::srv::GetLatestMessage>()
{
  return "message_server::srv::GetLatestMessage";
}

template<>
inline const char * name<message_server::srv::GetLatestMessage>()
{
  return "message_server/srv/GetLatestMessage";
}

template<>
struct has_fixed_size<message_server::srv::GetLatestMessage>
  : std::integral_constant<
    bool,
    has_fixed_size<message_server::srv::GetLatestMessage_Request>::value &&
    has_fixed_size<message_server::srv::GetLatestMessage_Response>::value
  >
{
};

template<>
struct has_bounded_size<message_server::srv::GetLatestMessage>
  : std::integral_constant<
    bool,
    has_bounded_size<message_server::srv::GetLatestMessage_Request>::value &&
    has_bounded_size<message_server::srv::GetLatestMessage_Response>::value
  >
{
};

template<>
struct is_service<message_server::srv::GetLatestMessage>
  : std::true_type
{
};

template<>
struct is_service_request<message_server::srv::GetLatestMessage_Request>
  : std::true_type
{
};

template<>
struct is_service_response<message_server::srv::GetLatestMessage_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // MESSAGE_SERVER__SRV__DETAIL__GET_LATEST_MESSAGE__TRAITS_HPP_
