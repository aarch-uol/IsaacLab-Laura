// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from message_server:srv/GetLatestMessage.idl
// generated code does not contain a copyright notice

#ifndef MESSAGE_SERVER__SRV__DETAIL__GET_LATEST_MESSAGE__BUILDER_HPP_
#define MESSAGE_SERVER__SRV__DETAIL__GET_LATEST_MESSAGE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "message_server/srv/detail/get_latest_message__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace message_server
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::message_server::srv::GetLatestMessage_Request>()
{
  return ::message_server::srv::GetLatestMessage_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace message_server


namespace message_server
{

namespace srv
{

namespace builder
{

class Init_GetLatestMessage_Response_latest_message
{
public:
  Init_GetLatestMessage_Response_latest_message()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::message_server::srv::GetLatestMessage_Response latest_message(::message_server::srv::GetLatestMessage_Response::_latest_message_type arg)
  {
    msg_.latest_message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::message_server::srv::GetLatestMessage_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::message_server::srv::GetLatestMessage_Response>()
{
  return message_server::srv::builder::Init_GetLatestMessage_Response_latest_message();
}

}  // namespace message_server

#endif  // MESSAGE_SERVER__SRV__DETAIL__GET_LATEST_MESSAGE__BUILDER_HPP_
