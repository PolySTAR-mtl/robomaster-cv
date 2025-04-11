// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from polystar_msgs:msg/Movement.idl
// generated code does not contain a copyright notice
#include "polystar_msgs/msg/detail/movement__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
polystar_msgs__msg__Movement__init(polystar_msgs__msg__Movement * msg)
{
  if (!msg) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    polystar_msgs__msg__Movement__fini(msg);
    return false;
  }
  // v_x
  // v_y
  // omega
  return true;
}

void
polystar_msgs__msg__Movement__fini(polystar_msgs__msg__Movement * msg)
{
  if (!msg) {
    return;
  }
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
  // v_x
  // v_y
  // omega
}

bool
polystar_msgs__msg__Movement__are_equal(const polystar_msgs__msg__Movement * lhs, const polystar_msgs__msg__Movement * rhs)
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
  // v_x
  if (lhs->v_x != rhs->v_x) {
    return false;
  }
  // v_y
  if (lhs->v_y != rhs->v_y) {
    return false;
  }
  // omega
  if (lhs->omega != rhs->omega) {
    return false;
  }
  return true;
}

bool
polystar_msgs__msg__Movement__copy(
  const polystar_msgs__msg__Movement * input,
  polystar_msgs__msg__Movement * output)
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
  // v_x
  output->v_x = input->v_x;
  // v_y
  output->v_y = input->v_y;
  // omega
  output->omega = input->omega;
  return true;
}

polystar_msgs__msg__Movement *
polystar_msgs__msg__Movement__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  polystar_msgs__msg__Movement * msg = (polystar_msgs__msg__Movement *)allocator.allocate(sizeof(polystar_msgs__msg__Movement), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(polystar_msgs__msg__Movement));
  bool success = polystar_msgs__msg__Movement__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
polystar_msgs__msg__Movement__destroy(polystar_msgs__msg__Movement * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    polystar_msgs__msg__Movement__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
polystar_msgs__msg__Movement__Sequence__init(polystar_msgs__msg__Movement__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  polystar_msgs__msg__Movement * data = NULL;

  if (size) {
    data = (polystar_msgs__msg__Movement *)allocator.zero_allocate(size, sizeof(polystar_msgs__msg__Movement), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = polystar_msgs__msg__Movement__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        polystar_msgs__msg__Movement__fini(&data[i - 1]);
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
polystar_msgs__msg__Movement__Sequence__fini(polystar_msgs__msg__Movement__Sequence * array)
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
      polystar_msgs__msg__Movement__fini(&array->data[i]);
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

polystar_msgs__msg__Movement__Sequence *
polystar_msgs__msg__Movement__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  polystar_msgs__msg__Movement__Sequence * array = (polystar_msgs__msg__Movement__Sequence *)allocator.allocate(sizeof(polystar_msgs__msg__Movement__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = polystar_msgs__msg__Movement__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
polystar_msgs__msg__Movement__Sequence__destroy(polystar_msgs__msg__Movement__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    polystar_msgs__msg__Movement__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
polystar_msgs__msg__Movement__Sequence__are_equal(const polystar_msgs__msg__Movement__Sequence * lhs, const polystar_msgs__msg__Movement__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!polystar_msgs__msg__Movement__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
polystar_msgs__msg__Movement__Sequence__copy(
  const polystar_msgs__msg__Movement__Sequence * input,
  polystar_msgs__msg__Movement__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(polystar_msgs__msg__Movement);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    polystar_msgs__msg__Movement * data =
      (polystar_msgs__msg__Movement *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!polystar_msgs__msg__Movement__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          polystar_msgs__msg__Movement__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!polystar_msgs__msg__Movement__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
