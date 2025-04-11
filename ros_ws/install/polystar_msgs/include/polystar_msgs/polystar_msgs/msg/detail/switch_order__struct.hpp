// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from polystar_msgs:msg/SwitchOrder.idl
// generated code does not contain a copyright notice

#ifndef POLYSTAR_MSGS__MSG__DETAIL__SWITCH_ORDER__STRUCT_HPP_
#define POLYSTAR_MSGS__MSG__DETAIL__SWITCH_ORDER__STRUCT_HPP_

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
# define DEPRECATED__polystar_msgs__msg__SwitchOrder __attribute__((deprecated))
#else
# define DEPRECATED__polystar_msgs__msg__SwitchOrder __declspec(deprecated)
#endif

namespace polystar_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SwitchOrder_
{
  using Type = SwitchOrder_<ContainerAllocator>;

  explicit SwitchOrder_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->order = 0;
    }
  }

  explicit SwitchOrder_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->order = 0;
    }
  }

  // field types and members
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;
  using _order_type =
    uint8_t;
  _order_type order;

  // setters for named parameter idiom
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }
  Type & set__order(
    const uint8_t & _arg)
  {
    this->order = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t ORDER_NOTHING =
    0u;
  static constexpr uint8_t ORDER_NEXT =
    1u;
  static constexpr uint8_t ORDER_RIGHT =
    2u;
  static constexpr uint8_t ORDER_LEFT =
    3u;

  // pointer types
  using RawPtr =
    polystar_msgs::msg::SwitchOrder_<ContainerAllocator> *;
  using ConstRawPtr =
    const polystar_msgs::msg::SwitchOrder_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<polystar_msgs::msg::SwitchOrder_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<polystar_msgs::msg::SwitchOrder_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      polystar_msgs::msg::SwitchOrder_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<polystar_msgs::msg::SwitchOrder_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      polystar_msgs::msg::SwitchOrder_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<polystar_msgs::msg::SwitchOrder_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<polystar_msgs::msg::SwitchOrder_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<polystar_msgs::msg::SwitchOrder_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__polystar_msgs__msg__SwitchOrder
    std::shared_ptr<polystar_msgs::msg::SwitchOrder_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__polystar_msgs__msg__SwitchOrder
    std::shared_ptr<polystar_msgs::msg::SwitchOrder_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SwitchOrder_ & other) const
  {
    if (this->stamp != other.stamp) {
      return false;
    }
    if (this->order != other.order) {
      return false;
    }
    return true;
  }
  bool operator!=(const SwitchOrder_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SwitchOrder_

// alias to use template instance with default allocator
using SwitchOrder =
  polystar_msgs::msg::SwitchOrder_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SwitchOrder_<ContainerAllocator>::ORDER_NOTHING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SwitchOrder_<ContainerAllocator>::ORDER_NEXT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SwitchOrder_<ContainerAllocator>::ORDER_RIGHT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SwitchOrder_<ContainerAllocator>::ORDER_LEFT;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace polystar_msgs

#endif  // POLYSTAR_MSGS__MSG__DETAIL__SWITCH_ORDER__STRUCT_HPP_
