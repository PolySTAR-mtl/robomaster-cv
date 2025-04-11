// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from polystar_msgs:msg/TurretFeedback.idl
// generated code does not contain a copyright notice
#include "polystar_msgs/msg/detail/turret_feedback__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
polystar_msgs__msg__TurretFeedback__init(polystar_msgs__msg__TurretFeedback * msg)
{
  if (!msg) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    polystar_msgs__msg__TurretFeedback__fini(msg);
    return false;
  }
  // pitch
  // yaw
  return true;
}

void
polystar_msgs__msg__TurretFeedback__fini(polystar_msgs__msg__TurretFeedback * msg)
{
  if (!msg) {
    return;
  }
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
  // pitch
  // yaw
}

bool
polystar_msgs__msg__TurretFeedback__are_equal(const polystar_msgs__msg__TurretFeedback * lhs, const polystar_msgs__msg__TurretFeedback * rhs)
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
  // pitch
  if (lhs->pitch != rhs->pitch) {
    return false;
  }
  // yaw
  if (lhs->yaw != rhs->yaw) {
    return false;
  }
  return true;
}

bool
polystar_msgs__msg__TurretFeedback__copy(
  const polystar_msgs__msg__TurretFeedback * input,
  polystar_msgs__msg__TurretFeedback * output)
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
  // pitch
  output->pitch = input->pitch;
  // yaw
  output->yaw = input->yaw;
  return true;
}

polystar_msgs__msg__TurretFeedback *
polystar_msgs__msg__TurretFeedback__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  polystar_msgs__msg__TurretFeedback * msg = (polystar_msgs__msg__TurretFeedback *)allocator.allocate(sizeof(polystar_msgs__msg__TurretFeedback), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(polystar_msgs__msg__TurretFeedback));
  bool success = polystar_msgs__msg__TurretFeedback__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
polystar_msgs__msg__TurretFeedback__destroy(polystar_msgs__msg__TurretFeedback * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    polystar_msgs__msg__TurretFeedback__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
polystar_msgs__msg__TurretFeedback__Sequence__init(polystar_msgs__msg__TurretFeedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  polystar_msgs__msg__TurretFeedback * data = NULL;

  if (size) {
    data = (polystar_msgs__msg__TurretFeedback *)allocator.zero_allocate(size, sizeof(polystar_msgs__msg__TurretFeedback), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = polystar_msgs__msg__TurretFeedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        polystar_msgs__msg__TurretFeedback__fini(&data[i - 1]);
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
polystar_msgs__msg__TurretFeedback__Sequence__fini(polystar_msgs__msg__TurretFeedback__Sequence * array)
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
      polystar_msgs__msg__TurretFeedback__fini(&array->data[i]);
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

polystar_msgs__msg__TurretFeedback__Sequence *
polystar_msgs__msg__TurretFeedback__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  polystar_msgs__msg__TurretFeedback__Sequence * array = (polystar_msgs__msg__TurretFeedback__Sequence *)allocator.allocate(sizeof(polystar_msgs__msg__TurretFeedback__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = polystar_msgs__msg__TurretFeedback__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
polystar_msgs__msg__TurretFeedback__Sequence__destroy(polystar_msgs__msg__TurretFeedback__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    polystar_msgs__msg__TurretFeedback__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
polystar_msgs__msg__TurretFeedback__Sequence__are_equal(const polystar_msgs__msg__TurretFeedback__Sequence * lhs, const polystar_msgs__msg__TurretFeedback__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!polystar_msgs__msg__TurretFeedback__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
polystar_msgs__msg__TurretFeedback__Sequence__copy(
  const polystar_msgs__msg__TurretFeedback__Sequence * input,
  polystar_msgs__msg__TurretFeedback__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(polystar_msgs__msg__TurretFeedback);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    polystar_msgs__msg__TurretFeedback * data =
      (polystar_msgs__msg__TurretFeedback *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!polystar_msgs__msg__TurretFeedback__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          polystar_msgs__msg__TurretFeedback__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!polystar_msgs__msg__TurretFeedback__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
