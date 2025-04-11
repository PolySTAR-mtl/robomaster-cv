// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from polystar_msgs:msg/HP.idl
// generated code does not contain a copyright notice

#ifndef POLYSTAR_MSGS__MSG__DETAIL__HP__STRUCT_HPP_
#define POLYSTAR_MSGS__MSG__DETAIL__HP__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__polystar_msgs__msg__HP __attribute__((deprecated))
#else
# define DEPRECATED__polystar_msgs__msg__HP __declspec(deprecated)
#endif

namespace polystar_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct HP_
{
  using Type = HP_<ContainerAllocator>;

  explicit HP_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->foe_hero = 0;
      this->foe_standard1 = 0;
      this->foe_standard2 = 0;
      this->foe_sentry = 0;
      this->ally_hero = 0;
      this->ally_standard1 = 0;
      this->ally_standard2 = 0;
      this->ally_sentry = 0;
    }
  }

  explicit HP_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->foe_hero = 0;
      this->foe_standard1 = 0;
      this->foe_standard2 = 0;
      this->foe_sentry = 0;
      this->ally_hero = 0;
      this->ally_standard1 = 0;
      this->ally_standard2 = 0;
      this->ally_sentry = 0;
    }
  }

  // field types and members
  using _foe_hero_type =
    uint16_t;
  _foe_hero_type foe_hero;
  using _foe_standard1_type =
    uint16_t;
  _foe_standard1_type foe_standard1;
  using _foe_standard2_type =
    uint16_t;
  _foe_standard2_type foe_standard2;
  using _foe_sentry_type =
    uint16_t;
  _foe_sentry_type foe_sentry;
  using _ally_hero_type =
    uint16_t;
  _ally_hero_type ally_hero;
  using _ally_standard1_type =
    uint16_t;
  _ally_standard1_type ally_standard1;
  using _ally_standard2_type =
    uint16_t;
  _ally_standard2_type ally_standard2;
  using _ally_sentry_type =
    uint16_t;
  _ally_sentry_type ally_sentry;

  // setters for named parameter idiom
  Type & set__foe_hero(
    const uint16_t & _arg)
  {
    this->foe_hero = _arg;
    return *this;
  }
  Type & set__foe_standard1(
    const uint16_t & _arg)
  {
    this->foe_standard1 = _arg;
    return *this;
  }
  Type & set__foe_standard2(
    const uint16_t & _arg)
  {
    this->foe_standard2 = _arg;
    return *this;
  }
  Type & set__foe_sentry(
    const uint16_t & _arg)
  {
    this->foe_sentry = _arg;
    return *this;
  }
  Type & set__ally_hero(
    const uint16_t & _arg)
  {
    this->ally_hero = _arg;
    return *this;
  }
  Type & set__ally_standard1(
    const uint16_t & _arg)
  {
    this->ally_standard1 = _arg;
    return *this;
  }
  Type & set__ally_standard2(
    const uint16_t & _arg)
  {
    this->ally_standard2 = _arg;
    return *this;
  }
  Type & set__ally_sentry(
    const uint16_t & _arg)
  {
    this->ally_sentry = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    polystar_msgs::msg::HP_<ContainerAllocator> *;
  using ConstRawPtr =
    const polystar_msgs::msg::HP_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<polystar_msgs::msg::HP_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<polystar_msgs::msg::HP_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      polystar_msgs::msg::HP_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<polystar_msgs::msg::HP_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      polystar_msgs::msg::HP_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<polystar_msgs::msg::HP_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<polystar_msgs::msg::HP_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<polystar_msgs::msg::HP_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__polystar_msgs__msg__HP
    std::shared_ptr<polystar_msgs::msg::HP_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__polystar_msgs__msg__HP
    std::shared_ptr<polystar_msgs::msg::HP_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const HP_ & other) const
  {
    if (this->foe_hero != other.foe_hero) {
      return false;
    }
    if (this->foe_standard1 != other.foe_standard1) {
      return false;
    }
    if (this->foe_standard2 != other.foe_standard2) {
      return false;
    }
    if (this->foe_sentry != other.foe_sentry) {
      return false;
    }
    if (this->ally_hero != other.ally_hero) {
      return false;
    }
    if (this->ally_standard1 != other.ally_standard1) {
      return false;
    }
    if (this->ally_standard2 != other.ally_standard2) {
      return false;
    }
    if (this->ally_sentry != other.ally_sentry) {
      return false;
    }
    return true;
  }
  bool operator!=(const HP_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct HP_

// alias to use template instance with default allocator
using HP =
  polystar_msgs::msg::HP_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace polystar_msgs

#endif  // POLYSTAR_MSGS__MSG__DETAIL__HP__STRUCT_HPP_
