// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from polystar_msgs:msg/HP.idl
// generated code does not contain a copyright notice
#include "polystar_msgs/msg/detail/hp__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
polystar_msgs__msg__HP__init(polystar_msgs__msg__HP * msg)
{
  if (!msg) {
    return false;
  }
  // foe_hero
  // foe_standard1
  // foe_standard2
  // foe_sentry
  // ally_hero
  // ally_standard1
  // ally_standard2
  // ally_sentry
  return true;
}

void
polystar_msgs__msg__HP__fini(polystar_msgs__msg__HP * msg)
{
  if (!msg) {
    return;
  }
  // foe_hero
  // foe_standard1
  // foe_standard2
  // foe_sentry
  // ally_hero
  // ally_standard1
  // ally_standard2
  // ally_sentry
}

bool
polystar_msgs__msg__HP__are_equal(const polystar_msgs__msg__HP * lhs, const polystar_msgs__msg__HP * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // foe_hero
  if (lhs->foe_hero != rhs->foe_hero) {
    return false;
  }
  // foe_standard1
  if (lhs->foe_standard1 != rhs->foe_standard1) {
    return false;
  }
  // foe_standard2
  if (lhs->foe_standard2 != rhs->foe_standard2) {
    return false;
  }
  // foe_sentry
  if (lhs->foe_sentry != rhs->foe_sentry) {
    return false;
  }
  // ally_hero
  if (lhs->ally_hero != rhs->ally_hero) {
    return false;
  }
  // ally_standard1
  if (lhs->ally_standard1 != rhs->ally_standard1) {
    return false;
  }
  // ally_standard2
  if (lhs->ally_standard2 != rhs->ally_standard2) {
    return false;
  }
  // ally_sentry
  if (lhs->ally_sentry != rhs->ally_sentry) {
    return false;
  }
  return true;
}

bool
polystar_msgs__msg__HP__copy(
  const polystar_msgs__msg__HP * input,
  polystar_msgs__msg__HP * output)
{
  if (!input || !output) {
    return false;
  }
  // foe_hero
  output->foe_hero = input->foe_hero;
  // foe_standard1
  output->foe_standard1 = input->foe_standard1;
  // foe_standard2
  output->foe_standard2 = input->foe_standard2;
  // foe_sentry
  output->foe_sentry = input->foe_sentry;
  // ally_hero
  output->ally_hero = input->ally_hero;
  // ally_standard1
  output->ally_standard1 = input->ally_standard1;
  // ally_standard2
  output->ally_standard2 = input->ally_standard2;
  // ally_sentry
  output->ally_sentry = input->ally_sentry;
  return true;
}

polystar_msgs__msg__HP *
polystar_msgs__msg__HP__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  polystar_msgs__msg__HP * msg = (polystar_msgs__msg__HP *)allocator.allocate(sizeof(polystar_msgs__msg__HP), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(polystar_msgs__msg__HP));
  bool success = polystar_msgs__msg__HP__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
polystar_msgs__msg__HP__destroy(polystar_msgs__msg__HP * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    polystar_msgs__msg__HP__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
polystar_msgs__msg__HP__Sequence__init(polystar_msgs__msg__HP__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  polystar_msgs__msg__HP * data = NULL;

  if (size) {
    data = (polystar_msgs__msg__HP *)allocator.zero_allocate(size, sizeof(polystar_msgs__msg__HP), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = polystar_msgs__msg__HP__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        polystar_msgs__msg__HP__fini(&data[i - 1]);
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
polystar_msgs__msg__HP__Sequence__fini(polystar_msgs__msg__HP__Sequence * array)
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
      polystar_msgs__msg__HP__fini(&array->data[i]);
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

polystar_msgs__msg__HP__Sequence *
polystar_msgs__msg__HP__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  polystar_msgs__msg__HP__Sequence * array = (polystar_msgs__msg__HP__Sequence *)allocator.allocate(sizeof(polystar_msgs__msg__HP__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = polystar_msgs__msg__HP__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
polystar_msgs__msg__HP__Sequence__destroy(polystar_msgs__msg__HP__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    polystar_msgs__msg__HP__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
polystar_msgs__msg__HP__Sequence__are_equal(const polystar_msgs__msg__HP__Sequence * lhs, const polystar_msgs__msg__HP__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!polystar_msgs__msg__HP__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
polystar_msgs__msg__HP__Sequence__copy(
  const polystar_msgs__msg__HP__Sequence * input,
  polystar_msgs__msg__HP__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(polystar_msgs__msg__HP);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    polystar_msgs__msg__HP * data =
      (polystar_msgs__msg__HP *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!polystar_msgs__msg__HP__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          polystar_msgs__msg__HP__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!polystar_msgs__msg__HP__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
