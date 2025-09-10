// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from message_server:srv/GetLatestMessage.idl
// generated code does not contain a copyright notice

#ifndef MESSAGE_SERVER__SRV__DETAIL__GET_LATEST_MESSAGE__STRUCT_HPP_
#define MESSAGE_SERVER__SRV__DETAIL__GET_LATEST_MESSAGE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__message_server__srv__GetLatestMessage_Request __attribute__((deprecated))
#else
# define DEPRECATED__message_server__srv__GetLatestMessage_Request __declspec(deprecated)
#endif

namespace message_server
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetLatestMessage_Request_
{
  using Type = GetLatestMessage_Request_<ContainerAllocator>;

  explicit GetLatestMessage_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit GetLatestMessage_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    message_server::srv::GetLatestMessage_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const message_server::srv::GetLatestMessage_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<message_server::srv::GetLatestMessage_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<message_server::srv::GetLatestMessage_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      message_server::srv::GetLatestMessage_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<message_server::srv::GetLatestMessage_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      message_server::srv::GetLatestMessage_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<message_server::srv::GetLatestMessage_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<message_server::srv::GetLatestMessage_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<message_server::srv::GetLatestMessage_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__message_server__srv__GetLatestMessage_Request
    std::shared_ptr<message_server::srv::GetLatestMessage_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__message_server__srv__GetLatestMessage_Request
    std::shared_ptr<message_server::srv::GetLatestMessage_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetLatestMessage_Request_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetLatestMessage_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetLatestMessage_Request_

// alias to use template instance with default allocator
using GetLatestMessage_Request =
  message_server::srv::GetLatestMessage_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace message_server


#ifndef _WIN32
# define DEPRECATED__message_server__srv__GetLatestMessage_Response __attribute__((deprecated))
#else
# define DEPRECATED__message_server__srv__GetLatestMessage_Response __declspec(deprecated)
#endif

namespace message_server
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetLatestMessage_Response_
{
  using Type = GetLatestMessage_Response_<ContainerAllocator>;

  explicit GetLatestMessage_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->latest_message = "";
    }
  }

  explicit GetLatestMessage_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : latest_message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->latest_message = "";
    }
  }

  // field types and members
  using _latest_message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _latest_message_type latest_message;

  // setters for named parameter idiom
  Type & set__latest_message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->latest_message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    message_server::srv::GetLatestMessage_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const message_server::srv::GetLatestMessage_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<message_server::srv::GetLatestMessage_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<message_server::srv::GetLatestMessage_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      message_server::srv::GetLatestMessage_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<message_server::srv::GetLatestMessage_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      message_server::srv::GetLatestMessage_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<message_server::srv::GetLatestMessage_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<message_server::srv::GetLatestMessage_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<message_server::srv::GetLatestMessage_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__message_server__srv__GetLatestMessage_Response
    std::shared_ptr<message_server::srv::GetLatestMessage_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__message_server__srv__GetLatestMessage_Response
    std::shared_ptr<message_server::srv::GetLatestMessage_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetLatestMessage_Response_ & other) const
  {
    if (this->latest_message != other.latest_message) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetLatestMessage_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetLatestMessage_Response_

// alias to use template instance with default allocator
using GetLatestMessage_Response =
  message_server::srv::GetLatestMessage_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace message_server

namespace message_server
{

namespace srv
{

struct GetLatestMessage
{
  using Request = message_server::srv::GetLatestMessage_Request;
  using Response = message_server::srv::GetLatestMessage_Response;
};

}  // namespace srv

}  // namespace message_server

#endif  // MESSAGE_SERVER__SRV__DETAIL__GET_LATEST_MESSAGE__STRUCT_HPP_
