// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from polystar_msgs:msg/PositionFeedback.idl
// generated code does not contain a copyright notice

#ifndef POLYSTAR_MSGS__MSG__DETAIL__POSITION_FEEDBACK__STRUCT_HPP_
#define POLYSTAR_MSGS__MSG__DETAIL__POSITION_FEEDBACK__STRUCT_HPP_

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
# define DEPRECATED__polystar_msgs__msg__PositionFeedback __attribute__((deprecated))
#else
# define DEPRECATED__polystar_msgs__msg__PositionFeedback __declspec(deprecated)
#endif

namespace polystar_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PositionFeedback_
{
  using Type = PositionFeedback_<ContainerAllocator>;

  explicit PositionFeedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->imu_ax = 0.0f;
      this->imu_ay = 0.0f;
      this->imu_az = 0.0f;
      this->imu_gx = 0.0f;
      this->imu_gy = 0.0f;
      this->imu_gz = 0.0f;
      this->imu_rx = 0.0f;
      this->imu_ry = 0.0f;
      this->imu_rz = 0.0f;
      this->enc_1 = 0ll;
      this->enc_2 = 0ll;
      this->enc_3 = 0ll;
      this->enc_4 = 0ll;
      this->v_enc_1 = 0.0f;
      this->v_enc_2 = 0.0f;
      this->v_enc_3 = 0.0f;
      this->v_enc_4 = 0.0f;
    }
  }

  explicit PositionFeedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->imu_ax = 0.0f;
      this->imu_ay = 0.0f;
      this->imu_az = 0.0f;
      this->imu_gx = 0.0f;
      this->imu_gy = 0.0f;
      this->imu_gz = 0.0f;
      this->imu_rx = 0.0f;
      this->imu_ry = 0.0f;
      this->imu_rz = 0.0f;
      this->enc_1 = 0ll;
      this->enc_2 = 0ll;
      this->enc_3 = 0ll;
      this->enc_4 = 0ll;
      this->v_enc_1 = 0.0f;
      this->v_enc_2 = 0.0f;
      this->v_enc_3 = 0.0f;
      this->v_enc_4 = 0.0f;
    }
  }

  // field types and members
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;
  using _imu_ax_type =
    float;
  _imu_ax_type imu_ax;
  using _imu_ay_type =
    float;
  _imu_ay_type imu_ay;
  using _imu_az_type =
    float;
  _imu_az_type imu_az;
  using _imu_gx_type =
    float;
  _imu_gx_type imu_gx;
  using _imu_gy_type =
    float;
  _imu_gy_type imu_gy;
  using _imu_gz_type =
    float;
  _imu_gz_type imu_gz;
  using _imu_rx_type =
    float;
  _imu_rx_type imu_rx;
  using _imu_ry_type =
    float;
  _imu_ry_type imu_ry;
  using _imu_rz_type =
    float;
  _imu_rz_type imu_rz;
  using _enc_1_type =
    int64_t;
  _enc_1_type enc_1;
  using _enc_2_type =
    int64_t;
  _enc_2_type enc_2;
  using _enc_3_type =
    int64_t;
  _enc_3_type enc_3;
  using _enc_4_type =
    int64_t;
  _enc_4_type enc_4;
  using _v_enc_1_type =
    float;
  _v_enc_1_type v_enc_1;
  using _v_enc_2_type =
    float;
  _v_enc_2_type v_enc_2;
  using _v_enc_3_type =
    float;
  _v_enc_3_type v_enc_3;
  using _v_enc_4_type =
    float;
  _v_enc_4_type v_enc_4;

  // setters for named parameter idiom
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }
  Type & set__imu_ax(
    const float & _arg)
  {
    this->imu_ax = _arg;
    return *this;
  }
  Type & set__imu_ay(
    const float & _arg)
  {
    this->imu_ay = _arg;
    return *this;
  }
  Type & set__imu_az(
    const float & _arg)
  {
    this->imu_az = _arg;
    return *this;
  }
  Type & set__imu_gx(
    const float & _arg)
  {
    this->imu_gx = _arg;
    return *this;
  }
  Type & set__imu_gy(
    const float & _arg)
  {
    this->imu_gy = _arg;
    return *this;
  }
  Type & set__imu_gz(
    const float & _arg)
  {
    this->imu_gz = _arg;
    return *this;
  }
  Type & set__imu_rx(
    const float & _arg)
  {
    this->imu_rx = _arg;
    return *this;
  }
  Type & set__imu_ry(
    const float & _arg)
  {
    this->imu_ry = _arg;
    return *this;
  }
  Type & set__imu_rz(
    const float & _arg)
  {
    this->imu_rz = _arg;
    return *this;
  }
  Type & set__enc_1(
    const int64_t & _arg)
  {
    this->enc_1 = _arg;
    return *this;
  }
  Type & set__enc_2(
    const int64_t & _arg)
  {
    this->enc_2 = _arg;
    return *this;
  }
  Type & set__enc_3(
    const int64_t & _arg)
  {
    this->enc_3 = _arg;
    return *this;
  }
  Type & set__enc_4(
    const int64_t & _arg)
  {
    this->enc_4 = _arg;
    return *this;
  }
  Type & set__v_enc_1(
    const float & _arg)
  {
    this->v_enc_1 = _arg;
    return *this;
  }
  Type & set__v_enc_2(
    const float & _arg)
  {
    this->v_enc_2 = _arg;
    return *this;
  }
  Type & set__v_enc_3(
    const float & _arg)
  {
    this->v_enc_3 = _arg;
    return *this;
  }
  Type & set__v_enc_4(
    const float & _arg)
  {
    this->v_enc_4 = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    polystar_msgs::msg::PositionFeedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const polystar_msgs::msg::PositionFeedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<polystar_msgs::msg::PositionFeedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<polystar_msgs::msg::PositionFeedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      polystar_msgs::msg::PositionFeedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<polystar_msgs::msg::PositionFeedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      polystar_msgs::msg::PositionFeedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<polystar_msgs::msg::PositionFeedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<polystar_msgs::msg::PositionFeedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<polystar_msgs::msg::PositionFeedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__polystar_msgs__msg__PositionFeedback
    std::shared_ptr<polystar_msgs::msg::PositionFeedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__polystar_msgs__msg__PositionFeedback
    std::shared_ptr<polystar_msgs::msg::PositionFeedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PositionFeedback_ & other) const
  {
    if (this->stamp != other.stamp) {
      return false;
    }
    if (this->imu_ax != other.imu_ax) {
      return false;
    }
    if (this->imu_ay != other.imu_ay) {
      return false;
    }
    if (this->imu_az != other.imu_az) {
      return false;
    }
    if (this->imu_gx != other.imu_gx) {
      return false;
    }
    if (this->imu_gy != other.imu_gy) {
      return false;
    }
    if (this->imu_gz != other.imu_gz) {
      return false;
    }
    if (this->imu_rx != other.imu_rx) {
      return false;
    }
    if (this->imu_ry != other.imu_ry) {
      return false;
    }
    if (this->imu_rz != other.imu_rz) {
      return false;
    }
    if (this->enc_1 != other.enc_1) {
      return false;
    }
    if (this->enc_2 != other.enc_2) {
      return false;
    }
    if (this->enc_3 != other.enc_3) {
      return false;
    }
    if (this->enc_4 != other.enc_4) {
      return false;
    }
    if (this->v_enc_1 != other.v_enc_1) {
      return false;
    }
    if (this->v_enc_2 != other.v_enc_2) {
      return false;
    }
    if (this->v_enc_3 != other.v_enc_3) {
      return false;
    }
    if (this->v_enc_4 != other.v_enc_4) {
      return false;
    }
    return true;
  }
  bool operator!=(const PositionFeedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PositionFeedback_

// alias to use template instance with default allocator
using PositionFeedback =
  polystar_msgs::msg::PositionFeedback_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace polystar_msgs

#endif  // POLYSTAR_MSGS__MSG__DETAIL__POSITION_FEEDBACK__STRUCT_HPP_
