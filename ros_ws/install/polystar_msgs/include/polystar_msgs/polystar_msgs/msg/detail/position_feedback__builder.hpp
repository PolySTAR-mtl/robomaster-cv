// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from polystar_msgs:msg/PositionFeedback.idl
// generated code does not contain a copyright notice

#ifndef POLYSTAR_MSGS__MSG__DETAIL__POSITION_FEEDBACK__BUILDER_HPP_
#define POLYSTAR_MSGS__MSG__DETAIL__POSITION_FEEDBACK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "polystar_msgs/msg/detail/position_feedback__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace polystar_msgs
{

namespace msg
{

namespace builder
{

class Init_PositionFeedback_v_enc_4
{
public:
  explicit Init_PositionFeedback_v_enc_4(::polystar_msgs::msg::PositionFeedback & msg)
  : msg_(msg)
  {}
  ::polystar_msgs::msg::PositionFeedback v_enc_4(::polystar_msgs::msg::PositionFeedback::_v_enc_4_type arg)
  {
    msg_.v_enc_4 = std::move(arg);
    return std::move(msg_);
  }

private:
  ::polystar_msgs::msg::PositionFeedback msg_;
};

class Init_PositionFeedback_v_enc_3
{
public:
  explicit Init_PositionFeedback_v_enc_3(::polystar_msgs::msg::PositionFeedback & msg)
  : msg_(msg)
  {}
  Init_PositionFeedback_v_enc_4 v_enc_3(::polystar_msgs::msg::PositionFeedback::_v_enc_3_type arg)
  {
    msg_.v_enc_3 = std::move(arg);
    return Init_PositionFeedback_v_enc_4(msg_);
  }

private:
  ::polystar_msgs::msg::PositionFeedback msg_;
};

class Init_PositionFeedback_v_enc_2
{
public:
  explicit Init_PositionFeedback_v_enc_2(::polystar_msgs::msg::PositionFeedback & msg)
  : msg_(msg)
  {}
  Init_PositionFeedback_v_enc_3 v_enc_2(::polystar_msgs::msg::PositionFeedback::_v_enc_2_type arg)
  {
    msg_.v_enc_2 = std::move(arg);
    return Init_PositionFeedback_v_enc_3(msg_);
  }

private:
  ::polystar_msgs::msg::PositionFeedback msg_;
};

class Init_PositionFeedback_v_enc_1
{
public:
  explicit Init_PositionFeedback_v_enc_1(::polystar_msgs::msg::PositionFeedback & msg)
  : msg_(msg)
  {}
  Init_PositionFeedback_v_enc_2 v_enc_1(::polystar_msgs::msg::PositionFeedback::_v_enc_1_type arg)
  {
    msg_.v_enc_1 = std::move(arg);
    return Init_PositionFeedback_v_enc_2(msg_);
  }

private:
  ::polystar_msgs::msg::PositionFeedback msg_;
};

class Init_PositionFeedback_enc_4
{
public:
  explicit Init_PositionFeedback_enc_4(::polystar_msgs::msg::PositionFeedback & msg)
  : msg_(msg)
  {}
  Init_PositionFeedback_v_enc_1 enc_4(::polystar_msgs::msg::PositionFeedback::_enc_4_type arg)
  {
    msg_.enc_4 = std::move(arg);
    return Init_PositionFeedback_v_enc_1(msg_);
  }

private:
  ::polystar_msgs::msg::PositionFeedback msg_;
};

class Init_PositionFeedback_enc_3
{
public:
  explicit Init_PositionFeedback_enc_3(::polystar_msgs::msg::PositionFeedback & msg)
  : msg_(msg)
  {}
  Init_PositionFeedback_enc_4 enc_3(::polystar_msgs::msg::PositionFeedback::_enc_3_type arg)
  {
    msg_.enc_3 = std::move(arg);
    return Init_PositionFeedback_enc_4(msg_);
  }

private:
  ::polystar_msgs::msg::PositionFeedback msg_;
};

class Init_PositionFeedback_enc_2
{
public:
  explicit Init_PositionFeedback_enc_2(::polystar_msgs::msg::PositionFeedback & msg)
  : msg_(msg)
  {}
  Init_PositionFeedback_enc_3 enc_2(::polystar_msgs::msg::PositionFeedback::_enc_2_type arg)
  {
    msg_.enc_2 = std::move(arg);
    return Init_PositionFeedback_enc_3(msg_);
  }

private:
  ::polystar_msgs::msg::PositionFeedback msg_;
};

class Init_PositionFeedback_enc_1
{
public:
  explicit Init_PositionFeedback_enc_1(::polystar_msgs::msg::PositionFeedback & msg)
  : msg_(msg)
  {}
  Init_PositionFeedback_enc_2 enc_1(::polystar_msgs::msg::PositionFeedback::_enc_1_type arg)
  {
    msg_.enc_1 = std::move(arg);
    return Init_PositionFeedback_enc_2(msg_);
  }

private:
  ::polystar_msgs::msg::PositionFeedback msg_;
};

class Init_PositionFeedback_imu_rz
{
public:
  explicit Init_PositionFeedback_imu_rz(::polystar_msgs::msg::PositionFeedback & msg)
  : msg_(msg)
  {}
  Init_PositionFeedback_enc_1 imu_rz(::polystar_msgs::msg::PositionFeedback::_imu_rz_type arg)
  {
    msg_.imu_rz = std::move(arg);
    return Init_PositionFeedback_enc_1(msg_);
  }

private:
  ::polystar_msgs::msg::PositionFeedback msg_;
};

class Init_PositionFeedback_imu_ry
{
public:
  explicit Init_PositionFeedback_imu_ry(::polystar_msgs::msg::PositionFeedback & msg)
  : msg_(msg)
  {}
  Init_PositionFeedback_imu_rz imu_ry(::polystar_msgs::msg::PositionFeedback::_imu_ry_type arg)
  {
    msg_.imu_ry = std::move(arg);
    return Init_PositionFeedback_imu_rz(msg_);
  }

private:
  ::polystar_msgs::msg::PositionFeedback msg_;
};

class Init_PositionFeedback_imu_rx
{
public:
  explicit Init_PositionFeedback_imu_rx(::polystar_msgs::msg::PositionFeedback & msg)
  : msg_(msg)
  {}
  Init_PositionFeedback_imu_ry imu_rx(::polystar_msgs::msg::PositionFeedback::_imu_rx_type arg)
  {
    msg_.imu_rx = std::move(arg);
    return Init_PositionFeedback_imu_ry(msg_);
  }

private:
  ::polystar_msgs::msg::PositionFeedback msg_;
};

class Init_PositionFeedback_imu_gz
{
public:
  explicit Init_PositionFeedback_imu_gz(::polystar_msgs::msg::PositionFeedback & msg)
  : msg_(msg)
  {}
  Init_PositionFeedback_imu_rx imu_gz(::polystar_msgs::msg::PositionFeedback::_imu_gz_type arg)
  {
    msg_.imu_gz = std::move(arg);
    return Init_PositionFeedback_imu_rx(msg_);
  }

private:
  ::polystar_msgs::msg::PositionFeedback msg_;
};

class Init_PositionFeedback_imu_gy
{
public:
  explicit Init_PositionFeedback_imu_gy(::polystar_msgs::msg::PositionFeedback & msg)
  : msg_(msg)
  {}
  Init_PositionFeedback_imu_gz imu_gy(::polystar_msgs::msg::PositionFeedback::_imu_gy_type arg)
  {
    msg_.imu_gy = std::move(arg);
    return Init_PositionFeedback_imu_gz(msg_);
  }

private:
  ::polystar_msgs::msg::PositionFeedback msg_;
};

class Init_PositionFeedback_imu_gx
{
public:
  explicit Init_PositionFeedback_imu_gx(::polystar_msgs::msg::PositionFeedback & msg)
  : msg_(msg)
  {}
  Init_PositionFeedback_imu_gy imu_gx(::polystar_msgs::msg::PositionFeedback::_imu_gx_type arg)
  {
    msg_.imu_gx = std::move(arg);
    return Init_PositionFeedback_imu_gy(msg_);
  }

private:
  ::polystar_msgs::msg::PositionFeedback msg_;
};

class Init_PositionFeedback_imu_az
{
public:
  explicit Init_PositionFeedback_imu_az(::polystar_msgs::msg::PositionFeedback & msg)
  : msg_(msg)
  {}
  Init_PositionFeedback_imu_gx imu_az(::polystar_msgs::msg::PositionFeedback::_imu_az_type arg)
  {
    msg_.imu_az = std::move(arg);
    return Init_PositionFeedback_imu_gx(msg_);
  }

private:
  ::polystar_msgs::msg::PositionFeedback msg_;
};

class Init_PositionFeedback_imu_ay
{
public:
  explicit Init_PositionFeedback_imu_ay(::polystar_msgs::msg::PositionFeedback & msg)
  : msg_(msg)
  {}
  Init_PositionFeedback_imu_az imu_ay(::polystar_msgs::msg::PositionFeedback::_imu_ay_type arg)
  {
    msg_.imu_ay = std::move(arg);
    return Init_PositionFeedback_imu_az(msg_);
  }

private:
  ::polystar_msgs::msg::PositionFeedback msg_;
};

class Init_PositionFeedback_imu_ax
{
public:
  explicit Init_PositionFeedback_imu_ax(::polystar_msgs::msg::PositionFeedback & msg)
  : msg_(msg)
  {}
  Init_PositionFeedback_imu_ay imu_ax(::polystar_msgs::msg::PositionFeedback::_imu_ax_type arg)
  {
    msg_.imu_ax = std::move(arg);
    return Init_PositionFeedback_imu_ay(msg_);
  }

private:
  ::polystar_msgs::msg::PositionFeedback msg_;
};

class Init_PositionFeedback_stamp
{
public:
  Init_PositionFeedback_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PositionFeedback_imu_ax stamp(::polystar_msgs::msg::PositionFeedback::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_PositionFeedback_imu_ax(msg_);
  }

private:
  ::polystar_msgs::msg::PositionFeedback msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::polystar_msgs::msg::PositionFeedback>()
{
  return polystar_msgs::msg::builder::Init_PositionFeedback_stamp();
}

}  // namespace polystar_msgs

#endif  // POLYSTAR_MSGS__MSG__DETAIL__POSITION_FEEDBACK__BUILDER_HPP_
