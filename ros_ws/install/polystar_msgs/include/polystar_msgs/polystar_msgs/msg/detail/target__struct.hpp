// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from polystar_msgs:msg/Target.idl
// generated code does not contain a copyright notice

#ifndef POLYSTAR_MSGS__MSG__DETAIL__TARGET__STRUCT_HPP_
#define POLYSTAR_MSGS__MSG__DETAIL__TARGET__STRUCT_HPP_

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
# define DEPRECATED__polystar_msgs__msg__Target __attribute__((deprecated))
#else
# define DEPRECATED__polystar_msgs__msg__Target __declspec(deprecated)
#endif

namespace polystar_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Target_
{
  using Type = Target_<ContainerAllocator>;

  explicit Target_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->located = false;
      this->distance_center = 0.0f;
      this->theta = 0;
      this->phi = 0;
      this->dist = 0;
    }
  }

  explicit Target_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->located = false;
      this->distance_center = 0.0f;
      this->theta = 0;
      this->phi = 0;
      this->dist = 0;
    }
  }

  // field types and members
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;
  using _located_type =
    bool;
  _located_type located;
  using _distance_center_type =
    float;
  _distance_center_type distance_center;
  using _theta_type =
    int16_t;
  _theta_type theta;
  using _phi_type =
    int16_t;
  _phi_type phi;
  using _dist_type =
    uint16_t;
  _dist_type dist;

  // setters for named parameter idiom
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }
  Type & set__located(
    const bool & _arg)
  {
    this->located = _arg;
    return *this;
  }
  Type & set__distance_center(
    const float & _arg)
  {
    this->distance_center = _arg;
    return *this;
  }
  Type & set__theta(
    const int16_t & _arg)
  {
    this->theta = _arg;
    return *this;
  }
  Type & set__phi(
    const int16_t & _arg)
  {
    this->phi = _arg;
    return *this;
  }
  Type & set__dist(
    const uint16_t & _arg)
  {
    this->dist = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    polystar_msgs::msg::Target_<ContainerAllocator> *;
  using ConstRawPtr =
    const polystar_msgs::msg::Target_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<polystar_msgs::msg::Target_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<polystar_msgs::msg::Target_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      polystar_msgs::msg::Target_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<polystar_msgs::msg::Target_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      polystar_msgs::msg::Target_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<polystar_msgs::msg::Target_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<polystar_msgs::msg::Target_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<polystar_msgs::msg::Target_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__polystar_msgs__msg__Target
    std::shared_ptr<polystar_msgs::msg::Target_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__polystar_msgs__msg__Target
    std::shared_ptr<polystar_msgs::msg::Target_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Target_ & other) const
  {
    if (this->stamp != other.stamp) {
      return false;
    }
    if (this->located != other.located) {
      return false;
    }
    if (this->distance_center != other.distance_center) {
      return false;
    }
    if (this->theta != other.theta) {
      return false;
    }
    if (this->phi != other.phi) {
      return false;
    }
    if (this->dist != other.dist) {
      return false;
    }
    return true;
  }
  bool operator!=(const Target_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Target_

// alias to use template instance with default allocator
using Target =
  polystar_msgs::msg::Target_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace polystar_msgs

#endif  // POLYSTAR_MSGS__MSG__DETAIL__TARGET__STRUCT_HPP_
