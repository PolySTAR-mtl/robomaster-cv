// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from polystar_msgs:msg/GameStatus.idl
// generated code does not contain a copyright notice

#ifndef POLYSTAR_MSGS__MSG__DETAIL__GAME_STATUS__STRUCT_HPP_
#define POLYSTAR_MSGS__MSG__DETAIL__GAME_STATUS__STRUCT_HPP_

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
# define DEPRECATED__polystar_msgs__msg__GameStatus __attribute__((deprecated))
#else
# define DEPRECATED__polystar_msgs__msg__GameStatus __declspec(deprecated)
#endif

namespace polystar_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GameStatus_
{
  using Type = GameStatus_<ContainerAllocator>;

  explicit GameStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_type = 0;
      this->red_std_hp = 0;
      this->red_hro_hp = 0;
      this->red_sty_hp = 0;
      this->blu_std_hp = 0;
      this->blu_hro_hp = 0;
      this->blu_sty_hp = 0;
      this->mode = 0;
    }
  }

  explicit GameStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_type = 0;
      this->red_std_hp = 0;
      this->red_hro_hp = 0;
      this->red_sty_hp = 0;
      this->blu_std_hp = 0;
      this->blu_hro_hp = 0;
      this->blu_sty_hp = 0;
      this->mode = 0;
    }
  }

  // field types and members
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;
  using _robot_type_type =
    uint8_t;
  _robot_type_type robot_type;
  using _red_std_hp_type =
    uint16_t;
  _red_std_hp_type red_std_hp;
  using _red_hro_hp_type =
    uint16_t;
  _red_hro_hp_type red_hro_hp;
  using _red_sty_hp_type =
    uint16_t;
  _red_sty_hp_type red_sty_hp;
  using _blu_std_hp_type =
    uint16_t;
  _blu_std_hp_type blu_std_hp;
  using _blu_hro_hp_type =
    uint16_t;
  _blu_hro_hp_type blu_hro_hp;
  using _blu_sty_hp_type =
    uint16_t;
  _blu_sty_hp_type blu_sty_hp;
  using _mode_type =
    uint8_t;
  _mode_type mode;

  // setters for named parameter idiom
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }
  Type & set__robot_type(
    const uint8_t & _arg)
  {
    this->robot_type = _arg;
    return *this;
  }
  Type & set__red_std_hp(
    const uint16_t & _arg)
  {
    this->red_std_hp = _arg;
    return *this;
  }
  Type & set__red_hro_hp(
    const uint16_t & _arg)
  {
    this->red_hro_hp = _arg;
    return *this;
  }
  Type & set__red_sty_hp(
    const uint16_t & _arg)
  {
    this->red_sty_hp = _arg;
    return *this;
  }
  Type & set__blu_std_hp(
    const uint16_t & _arg)
  {
    this->blu_std_hp = _arg;
    return *this;
  }
  Type & set__blu_hro_hp(
    const uint16_t & _arg)
  {
    this->blu_hro_hp = _arg;
    return *this;
  }
  Type & set__blu_sty_hp(
    const uint16_t & _arg)
  {
    this->blu_sty_hp = _arg;
    return *this;
  }
  Type & set__mode(
    const uint8_t & _arg)
  {
    this->mode = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    polystar_msgs::msg::GameStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const polystar_msgs::msg::GameStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<polystar_msgs::msg::GameStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<polystar_msgs::msg::GameStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      polystar_msgs::msg::GameStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<polystar_msgs::msg::GameStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      polystar_msgs::msg::GameStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<polystar_msgs::msg::GameStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<polystar_msgs::msg::GameStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<polystar_msgs::msg::GameStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__polystar_msgs__msg__GameStatus
    std::shared_ptr<polystar_msgs::msg::GameStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__polystar_msgs__msg__GameStatus
    std::shared_ptr<polystar_msgs::msg::GameStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GameStatus_ & other) const
  {
    if (this->stamp != other.stamp) {
      return false;
    }
    if (this->robot_type != other.robot_type) {
      return false;
    }
    if (this->red_std_hp != other.red_std_hp) {
      return false;
    }
    if (this->red_hro_hp != other.red_hro_hp) {
      return false;
    }
    if (this->red_sty_hp != other.red_sty_hp) {
      return false;
    }
    if (this->blu_std_hp != other.blu_std_hp) {
      return false;
    }
    if (this->blu_hro_hp != other.blu_hro_hp) {
      return false;
    }
    if (this->blu_sty_hp != other.blu_sty_hp) {
      return false;
    }
    if (this->mode != other.mode) {
      return false;
    }
    return true;
  }
  bool operator!=(const GameStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GameStatus_

// alias to use template instance with default allocator
using GameStatus =
  polystar_msgs::msg::GameStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace polystar_msgs

#endif  // POLYSTAR_MSGS__MSG__DETAIL__GAME_STATUS__STRUCT_HPP_
