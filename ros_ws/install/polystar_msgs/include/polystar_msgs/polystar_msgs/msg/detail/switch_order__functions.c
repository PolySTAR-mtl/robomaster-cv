// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from polystar_msgs:msg/SwitchOrder.idl
// generated code does not contain a copyright notice
#include "polystar_msgs/msg/detail/switch_order__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
polystar_msgs__msg__SwitchOrder__init(polystar_msgs__msg__SwitchOrder * msg)
{
  if (!msg) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    polystar_msgs__msg__SwitchOrder__fini(msg);
    return false;
  }
  // order
  return true;
}

void
polystar_msgs__msg__SwitchOrder__fini(polystar_msgs__msg__SwitchOrder * msg)
{
  if (!msg) {
    return;
  }
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
  // order
}

bool
polystar_msgs__msg__SwitchOrder__are_equal(const polystar_msgs__msg__SwitchOrder * lhs, const polystar_msgs__msg__SwitchOrder * rhs)
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
  // order
  if (lhs->order != rhs->order) {
    return false;
  }
  return true;
}

bool
polystar_msgs__msg__SwitchOrder__copy(
  const polystar_msgs__msg__SwitchOrder * input,
  polystar_msgs__msg__SwitchOrder * output)
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
  // order
  output->order = input->order;
  return true;
}

polystar_msgs__msg__SwitchOrder *
polystar_msgs__msg__SwitchOrder__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  polystar_msgs__msg__SwitchOrder * msg = (polystar_msgs__msg__SwitchOrder *)allocator.allocate(sizeof(polystar_msgs__msg__SwitchOrder), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(polystar_msgs__msg__SwitchOrder));
  bool success = polystar_msgs__msg__SwitchOrder__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
polystar_msgs__msg__SwitchOrder__destroy(polystar_msgs__msg__SwitchOrder * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    polystar_msgs__msg__SwitchOrder__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
polystar_msgs__msg__SwitchOrder__Sequence__init(polystar_msgs__msg__SwitchOrder__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  polystar_msgs__msg__SwitchOrder * data = NULL;

  if (size) {
    data = (polystar_msgs__msg__SwitchOrder *)allocator.zero_allocate(size, sizeof(polystar_msgs__msg__SwitchOrder), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = polystar_msgs__msg__SwitchOrder__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        polystar_msgs__msg__SwitchOrder__fini(&data[i - 1]);
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
polystar_msgs__msg__SwitchOrder__Sequence__fini(polystar_msgs__msg__SwitchOrder__Sequence * array)
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
      polystar_msgs__msg__SwitchOrder__fini(&array->data[i]);
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

polystar_msgs__msg__SwitchOrder__Sequence *
polystar_msgs__msg__SwitchOrder__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  polystar_msgs__msg__SwitchOrder__Sequence * array = (polystar_msgs__msg__SwitchOrder__Sequence *)allocator.allocate(sizeof(polystar_msgs__msg__SwitchOrder__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = polystar_msgs__msg__SwitchOrder__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
polystar_msgs__msg__SwitchOrder__Sequence__destroy(polystar_msgs__msg__SwitchOrder__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    polystar_msgs__msg__SwitchOrder__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
polystar_msgs__msg__SwitchOrder__Sequence__are_equal(const polystar_msgs__msg__SwitchOrder__Sequence * lhs, const polystar_msgs__msg__SwitchOrder__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!polystar_msgs__msg__SwitchOrder__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
polystar_msgs__msg__SwitchOrder__Sequence__copy(
  const polystar_msgs__msg__SwitchOrder__Sequence * input,
  polystar_msgs__msg__SwitchOrder__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(polystar_msgs__msg__SwitchOrder);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    polystar_msgs__msg__SwitchOrder * data =
      (polystar_msgs__msg__SwitchOrder *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!polystar_msgs__msg__SwitchOrder__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          polystar_msgs__msg__SwitchOrder__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!polystar_msgs__msg__SwitchOrder__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
