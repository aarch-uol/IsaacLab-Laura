// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robot_state_message:srv/ReturnRobotState.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_STATE_MESSAGE__SRV__DETAIL__RETURN_ROBOT_STATE__STRUCT_HPP_
#define ROBOT_STATE_MESSAGE__SRV__DETAIL__RETURN_ROBOT_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__robot_state_message__srv__ReturnRobotState_Request __attribute__((deprecated))
#else
# define DEPRECATED__robot_state_message__srv__ReturnRobotState_Request __declspec(deprecated)
#endif

namespace robot_state_message
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ReturnRobotState_Request_
{
  using Type = ReturnRobotState_Request_<ContainerAllocator>;

  explicit ReturnRobotState_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sm_state = 0l;
      this->time = 0l;
    }
  }

  explicit ReturnRobotState_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sm_state = 0l;
      this->time = 0l;
    }
  }

  // field types and members
  using _joint_pos_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _joint_pos_type joint_pos;
  using _gripper_pos_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _gripper_pos_type gripper_pos;
  using _sm_state_type =
    int32_t;
  _sm_state_type sm_state;
  using _time_type =
    int32_t;
  _time_type time;

  // setters for named parameter idiom
  Type & set__joint_pos(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->joint_pos = _arg;
    return *this;
  }
  Type & set__gripper_pos(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->gripper_pos = _arg;
    return *this;
  }
  Type & set__sm_state(
    const int32_t & _arg)
  {
    this->sm_state = _arg;
    return *this;
  }
  Type & set__time(
    const int32_t & _arg)
  {
    this->time = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_state_message::srv::ReturnRobotState_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_state_message::srv::ReturnRobotState_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_state_message::srv::ReturnRobotState_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_state_message::srv::ReturnRobotState_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_state_message::srv::ReturnRobotState_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_state_message::srv::ReturnRobotState_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_state_message::srv::ReturnRobotState_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_state_message::srv::ReturnRobotState_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_state_message::srv::ReturnRobotState_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_state_message::srv::ReturnRobotState_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_state_message__srv__ReturnRobotState_Request
    std::shared_ptr<robot_state_message::srv::ReturnRobotState_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_state_message__srv__ReturnRobotState_Request
    std::shared_ptr<robot_state_message::srv::ReturnRobotState_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ReturnRobotState_Request_ & other) const
  {
    if (this->joint_pos != other.joint_pos) {
      return false;
    }
    if (this->gripper_pos != other.gripper_pos) {
      return false;
    }
    if (this->sm_state != other.sm_state) {
      return false;
    }
    if (this->time != other.time) {
      return false;
    }
    return true;
  }
  bool operator!=(const ReturnRobotState_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ReturnRobotState_Request_

// alias to use template instance with default allocator
using ReturnRobotState_Request =
  robot_state_message::srv::ReturnRobotState_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace robot_state_message


#ifndef _WIN32
# define DEPRECATED__robot_state_message__srv__ReturnRobotState_Response __attribute__((deprecated))
#else
# define DEPRECATED__robot_state_message__srv__ReturnRobotState_Response __declspec(deprecated)
#endif

namespace robot_state_message
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ReturnRobotState_Response_
{
  using Type = ReturnRobotState_Response_<ContainerAllocator>;

  explicit ReturnRobotState_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sm_state = 0l;
      this->time = 0l;
    }
  }

  explicit ReturnRobotState_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sm_state = 0l;
      this->time = 0l;
    }
  }

  // field types and members
  using _joint_pos_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _joint_pos_type joint_pos;
  using _gripper_pos_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _gripper_pos_type gripper_pos;
  using _sm_state_type =
    int32_t;
  _sm_state_type sm_state;
  using _time_type =
    int32_t;
  _time_type time;

  // setters for named parameter idiom
  Type & set__joint_pos(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->joint_pos = _arg;
    return *this;
  }
  Type & set__gripper_pos(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->gripper_pos = _arg;
    return *this;
  }
  Type & set__sm_state(
    const int32_t & _arg)
  {
    this->sm_state = _arg;
    return *this;
  }
  Type & set__time(
    const int32_t & _arg)
  {
    this->time = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_state_message::srv::ReturnRobotState_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_state_message::srv::ReturnRobotState_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_state_message::srv::ReturnRobotState_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_state_message::srv::ReturnRobotState_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_state_message::srv::ReturnRobotState_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_state_message::srv::ReturnRobotState_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_state_message::srv::ReturnRobotState_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_state_message::srv::ReturnRobotState_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_state_message::srv::ReturnRobotState_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_state_message::srv::ReturnRobotState_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_state_message__srv__ReturnRobotState_Response
    std::shared_ptr<robot_state_message::srv::ReturnRobotState_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_state_message__srv__ReturnRobotState_Response
    std::shared_ptr<robot_state_message::srv::ReturnRobotState_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ReturnRobotState_Response_ & other) const
  {
    if (this->joint_pos != other.joint_pos) {
      return false;
    }
    if (this->gripper_pos != other.gripper_pos) {
      return false;
    }
    if (this->sm_state != other.sm_state) {
      return false;
    }
    if (this->time != other.time) {
      return false;
    }
    return true;
  }
  bool operator!=(const ReturnRobotState_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ReturnRobotState_Response_

// alias to use template instance with default allocator
using ReturnRobotState_Response =
  robot_state_message::srv::ReturnRobotState_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace robot_state_message

namespace robot_state_message
{

namespace srv
{

struct ReturnRobotState
{
  using Request = robot_state_message::srv::ReturnRobotState_Request;
  using Response = robot_state_message::srv::ReturnRobotState_Response;
};

}  // namespace srv

}  // namespace robot_state_message

#endif  // ROBOT_STATE_MESSAGE__SRV__DETAIL__RETURN_ROBOT_STATE__STRUCT_HPP_
