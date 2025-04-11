// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from polystar_msgs:msg/GameStatus.idl
// generated code does not contain a copyright notice
#include "polystar_msgs/msg/detail/game_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
polystar_msgs__msg__GameStatus__init(polystar_msgs__msg__GameStatus * msg)
{
  if (!msg) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    polystar_msgs__msg__GameStatus__fini(msg);
    return false;
  }
  // robot_type
  // red_std_hp
  // red_hro_hp
  // red_sty_hp
  // blu_std_hp
  // blu_hro_hp
  // blu_sty_hp
  // mode
  return true;
}

void
polystar_msgs__msg__GameStatus__fini(polystar_msgs__msg__GameStatus * msg)
{
  if (!msg) {
    return;
  }
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
  // robot_type
  // red_std_hp
  // red_hro_hp
  // red_sty_hp
  // blu_std_hp
  // blu_hro_hp
  // blu_sty_hp
  // mode
}

bool
polystar_msgs__msg__GameStatus__are_equal(const polystar_msgs__msg__GameStatus * lhs, const polystar_msgs__msg__GameStatus * rhs)
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
  // robot_type
  if (lhs->robot_type != rhs->robot_type) {
    return false;
  }
  // red_std_hp
  if (lhs->red_std_hp != rhs->red_std_hp) {
    return false;
  }
  // red_hro_hp
  if (lhs->red_hro_hp != rhs->red_hro_hp) {
    return false;
  }
  // red_sty_hp
  if (lhs->red_sty_hp != rhs->red_sty_hp) {
    return false;
  }
  // blu_std_hp
  if (lhs->blu_std_hp != rhs->blu_std_hp) {
    return false;
  }
  // blu_hro_hp
  if (lhs->blu_hro_hp != rhs->blu_hro_hp) {
    return false;
  }
  // blu_sty_hp
  if (lhs->blu_sty_hp != rhs->blu_sty_hp) {
    return false;
  }
  // mode
  if (lhs->mode != rhs->mode) {
    return false;
  }
  return true;
}

bool
polystar_msgs__msg__GameStatus__copy(
  const polystar_msgs__msg__GameStatus * input,
  polystar_msgs__msg__GameStatus * output)
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
  // robot_type
  output->robot_type = input->robot_type;
  // red_std_hp
  output->red_std_hp = input->red_std_hp;
  // red_hro_hp
  output->red_hro_hp = input->red_hro_hp;
  // red_sty_hp
  output->red_sty_hp = input->red_sty_hp;
  // blu_std_hp
  output->blu_std_hp = input->blu_std_hp;
  // blu_hro_hp
  output->blu_hro_hp = input->blu_hro_hp;
  // blu_sty_hp
  output->blu_sty_hp = input->blu_sty_hp;
  // mode
  output->mode = input->mode;
  return true;
}

polystar_msgs__msg__GameStatus *
polystar_msgs__msg__GameStatus__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  polystar_msgs__msg__GameStatus * msg = (polystar_msgs__msg__GameStatus *)allocator.allocate(sizeof(polystar_msgs__msg__GameStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(polystar_msgs__msg__GameStatus));
  bool success = polystar_msgs__msg__GameStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
polystar_msgs__msg__GameStatus__destroy(polystar_msgs__msg__GameStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    polystar_msgs__msg__GameStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
polystar_msgs__msg__GameStatus__Sequence__init(polystar_msgs__msg__GameStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  polystar_msgs__msg__GameStatus * data = NULL;

  if (size) {
    data = (polystar_msgs__msg__GameStatus *)allocator.zero_allocate(size, sizeof(polystar_msgs__msg__GameStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = polystar_msgs__msg__GameStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        polystar_msgs__msg__GameStatus__fini(&data[i - 1]);
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
polystar_msgs__msg__GameStatus__Sequence__fini(polystar_msgs__msg__GameStatus__Sequence * array)
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
      polystar_msgs__msg__GameStatus__fini(&array->data[i]);
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

polystar_msgs__msg__GameStatus__Sequence *
polystar_msgs__msg__GameStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  polystar_msgs__msg__GameStatus__Sequence * array = (polystar_msgs__msg__GameStatus__Sequence *)allocator.allocate(sizeof(polystar_msgs__msg__GameStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = polystar_msgs__msg__GameStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
polystar_msgs__msg__GameStatus__Sequence__destroy(polystar_msgs__msg__GameStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    polystar_msgs__msg__GameStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
polystar_msgs__msg__GameStatus__Sequence__are_equal(const polystar_msgs__msg__GameStatus__Sequence * lhs, const polystar_msgs__msg__GameStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!polystar_msgs__msg__GameStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
polystar_msgs__msg__GameStatus__Sequence__copy(
  const polystar_msgs__msg__GameStatus__Sequence * input,
  polystar_msgs__msg__GameStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(polystar_msgs__msg__GameStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    polystar_msgs__msg__GameStatus * data =
      (polystar_msgs__msg__GameStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!polystar_msgs__msg__GameStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          polystar_msgs__msg__GameStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!polystar_msgs__msg__GameStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
