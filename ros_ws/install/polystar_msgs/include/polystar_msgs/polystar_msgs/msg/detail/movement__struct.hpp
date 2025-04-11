// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from polystar_msgs:msg/Movement.idl
// generated code does not contain a copyright notice

#ifndef POLYSTAR_MSGS__MSG__DETAIL__MOVEMENT__STRUCT_HPP_
#define POLYSTAR_MSGS__MSG__DETAIL__MOVEMENT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__polystar_msgs__msg__Movement __attribute__((deprecated))
#else
# define DEPRECATED__polystar_msgs__msg__Movement __declspec(deprecated)
#endif

namespace polystar_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Movement_
{
  using Type = Movement_<ContainerAllocator>;

  explicit Movement_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->v_x = 0.0f;
      this->v_y = 0.0f;
      this->omega = 0.0f;
    }
  }

  explicit Movement_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->v_x = 0.0f;
      this->v_y = 0.0f;
      this->omega = 0.0f;
    }
  }

  // field types and members
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;
  using _v_x_type =
    float;
  _v_x_type v_x;
  using _v_y_type =
    float;
  _v_y_type v_y;
  using _omega_type =
    float;
  _omega_type omega;

  // setters for named parameter idiom
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }
  Type & set__v_x(
    const float & _arg)
  {
    this->v_x = _arg;
    return *this;
  }
  Type & set__v_y(
    const float & _arg)
  {
    this->v_y = _arg;
    return *this;
  }
  Type & set__omega(
    const float & _arg)
  {
    this->omega = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    polystar_msgs::msg::Movement_<ContainerAllocator> *;
  using ConstRawPtr =
    const polystar_msgs::msg::Movement_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<polystar_msgs::msg::Movement_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<polystar_msgs::msg::Movement_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      polystar_msgs::msg::Movement_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<polystar_msgs::msg::Movement_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      polystar_msgs::msg::Movement_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<polystar_msgs::msg::Movement_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<polystar_msgs::msg::Movement_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<polystar_msgs::msg::Movement_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__polystar_msgs__msg__Movement
    std::shared_ptr<polystar_msgs::msg::Movement_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__polystar_msgs__msg__Movement
    std::shared_ptr<polystar_msgs::msg::Movement_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Movement_ & other) const
  {
    if (this->stamp != other.stamp) {
      return false;
    }
    if (this->v_x != other.v_x) {
      return false;
    }
    if (this->v_y != other.v_y) {
      return false;
    }
    if (this->omega != other.omega) {
      return false;
    }
    return true;
  }
  bool operator!=(const Movement_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Movement_

// alias to use template instance with default allocator
using Movement =
  polystar_msgs::msg::Movement_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace polystar_msgs

#endif  // POLYSTAR_MSGS__MSG__DETAIL__MOVEMENT__STRUCT_HPP_
