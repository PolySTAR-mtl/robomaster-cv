// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from polystar_msgs:msg/PositionFeedback.idl
// generated code does not contain a copyright notice
#include "polystar_msgs/msg/detail/position_feedback__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
polystar_msgs__msg__PositionFeedback__init(polystar_msgs__msg__PositionFeedback * msg)
{
  if (!msg) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    polystar_msgs__msg__PositionFeedback__fini(msg);
    return false;
  }
  // imu_ax
  // imu_ay
  // imu_az
  // imu_gx
  // imu_gy
  // imu_gz
  // imu_rx
  // imu_ry
  // imu_rz
  // enc_1
  // enc_2
  // enc_3
  // enc_4
  // v_enc_1
  // v_enc_2
  // v_enc_3
  // v_enc_4
  return true;
}

void
polystar_msgs__msg__PositionFeedback__fini(polystar_msgs__msg__PositionFeedback * msg)
{
  if (!msg) {
    return;
  }
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
  // imu_ax
  // imu_ay
  // imu_az
  // imu_gx
  // imu_gy
  // imu_gz
  // imu_rx
  // imu_ry
  // imu_rz
  // enc_1
  // enc_2
  // enc_3
  // enc_4
  // v_enc_1
  // v_enc_2
  // v_enc_3
  // v_enc_4
}

bool
polystar_msgs__msg__PositionFeedback__are_equal(const polystar_msgs__msg__PositionFeedback * lhs, const polystar_msgs__msg__PositionFeedback * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->stamp), &(rhs->stamp)))
  {
    return false;
  }
  // imu_ax
  if (lhs->imu_ax != rhs->imu_ax) {
    return false;
  }
  // imu_ay
  if (lhs->imu_ay != rhs->imu_ay) {
    return false;
  }
  // imu_az
  if (lhs->imu_az != rhs->imu_az) {
    return false;
  }
  // imu_gx
  if (lhs->imu_gx != rhs->imu_gx) {
    return false;
  }
  // imu_gy
  if (lhs->imu_gy != rhs->imu_gy) {
    return false;
  }
  // imu_gz
  if (lhs->imu_gz != rhs->imu_gz) {
    return false;
  }
  // imu_rx
  if (lhs->imu_rx != rhs->imu_rx) {
    return false;
  }
  // imu_ry
  if (lhs->imu_ry != rhs->imu_ry) {
    return false;
  }
  // imu_rz
  if (lhs->imu_rz != rhs->imu_rz) {
    return false;
  }
  // enc_1
  if (lhs->enc_1 != rhs->enc_1) {
    return false;
  }
  // enc_2
  if (lhs->enc_2 != rhs->enc_2) {
    return false;
  }
  // enc_3
  if (lhs->enc_3 != rhs->enc_3) {
    return false;
  }
  // enc_4
  if (lhs->enc_4 != rhs->enc_4) {
    return false;
  }
  // v_enc_1
  if (lhs->v_enc_1 != rhs->v_enc_1) {
    return false;
  }
  // v_enc_2
  if (lhs->v_enc_2 != rhs->v_enc_2) {
    return false;
  }
  // v_enc_3
  if (lhs->v_enc_3 != rhs->v_enc_3) {
    return false;
  }
  // v_enc_4
  if (lhs->v_enc_4 != rhs->v_enc_4) {
    return false;
  }
  return true;
}

bool
polystar_msgs__msg__PositionFeedback__copy(
  const polystar_msgs__msg__PositionFeedback * input,
  polystar_msgs__msg__PositionFeedback * output)
{
  if (!input || !output) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->stamp), &(output->stamp)))
  {
    return false;
  }
  // imu_ax
  output->imu_ax = input->imu_ax;
  // imu_ay
  output->imu_ay = input->imu_ay;
  // imu_az
  output->imu_az = input->imu_az;
  // imu_gx
  output->imu_gx = input->imu_gx;
  // imu_gy
  output->imu_gy = input->imu_gy;
  // imu_gz
  output->imu_gz = input->imu_gz;
  // imu_rx
  output->imu_rx = input->imu_rx;
  // imu_ry
  output->imu_ry = input->imu_ry;
  // imu_rz
  output->imu_rz = input->imu_rz;
  // enc_1
  output->enc_1 = input->enc_1;
  // enc_2
  output->enc_2 = input->enc_2;
  // enc_3
  output->enc_3 = input->enc_3;
  // enc_4
  output->enc_4 = input->enc_4;
  // v_enc_1
  output->v_enc_1 = input->v_enc_1;
  // v_enc_2
  output->v_enc_2 = input->v_enc_2;
  // v_enc_3
  output->v_enc_3 = input->v_enc_3;
  // v_enc_4
  output->v_enc_4 = input->v_enc_4;
  return true;
}

polystar_msgs__msg__PositionFeedback *
polystar_msgs__msg__PositionFeedback__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  polystar_msgs__msg__PositionFeedback * msg = (polystar_msgs__msg__PositionFeedback *)allocator.allocate(sizeof(polystar_msgs__msg__PositionFeedback), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(polystar_msgs__msg__PositionFeedback));
  bool success = polystar_msgs__msg__PositionFeedback__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
polystar_msgs__msg__PositionFeedback__destroy(polystar_msgs__msg__PositionFeedback * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    polystar_msgs__msg__PositionFeedback__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
polystar_msgs__msg__PositionFeedback__Sequence__init(polystar_msgs__msg__PositionFeedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  polystar_msgs__msg__PositionFeedback * data = NULL;

  if (size) {
    data = (polystar_msgs__msg__PositionFeedback *)allocator.zero_allocate(size, sizeof(polystar_msgs__msg__PositionFeedback), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = polystar_msgs__msg__PositionFeedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        polystar_msgs__msg__PositionFeedback__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
polystar_msgs__msg__PositionFeedback__Sequence__fini(polystar_msgs__msg__PositionFeedback__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      polystar_msgs__msg__PositionFeedback__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

polystar_msgs__msg__PositionFeedback__Sequence *
polystar_msgs__msg__PositionFeedback__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  polystar_msgs__msg__PositionFeedback__Sequence * array = (polystar_msgs__msg__PositionFeedback__Sequence *)allocator.allocate(sizeof(polystar_msgs__msg__PositionFeedback__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = polystar_msgs__msg__PositionFeedback__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
polystar_msgs__msg__PositionFeedback__Sequence__destroy(polystar_msgs__msg__PositionFeedback__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    polystar_msgs__msg__PositionFeedback__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
polystar_msgs__msg__PositionFeedback__Sequence__are_equal(const polystar_msgs__msg__PositionFeedback__Sequence * lhs, const polystar_msgs__msg__PositionFeedback__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!polystar_msgs__msg__PositionFeedback__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
polystar_msgs__msg__PositionFeedback__Sequence__copy(
  const polystar_msgs__msg__PositionFeedback__Sequence * input,
  polystar_msgs__msg__PositionFeedback__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(polystar_msgs__msg__PositionFeedback);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    polystar_msgs__msg__PositionFeedback * data =
      (polystar_msgs__msg__PositionFeedback *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!polystar_msgs__msg__PositionFeedback__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          polystar_msgs__msg__PositionFeedback__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!polystar_msgs__msg__PositionFeedback__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
