// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ngc_interfaces:msg/NuDot.idl
// generated code does not contain a copyright notice
#include "ngc_interfaces/msg/detail/nu_dot__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
ngc_interfaces__msg__NuDot__init(ngc_interfaces__msg__NuDot * msg)
{
  if (!msg) {
    return false;
  }
  // u_dot
  // v_dot
  // w_dot
  // p_dot
  // q_dot
  // r_dot
  return true;
}

void
ngc_interfaces__msg__NuDot__fini(ngc_interfaces__msg__NuDot * msg)
{
  if (!msg) {
    return;
  }
  // u_dot
  // v_dot
  // w_dot
  // p_dot
  // q_dot
  // r_dot
}

bool
ngc_interfaces__msg__NuDot__are_equal(const ngc_interfaces__msg__NuDot * lhs, const ngc_interfaces__msg__NuDot * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // u_dot
  if (lhs->u_dot != rhs->u_dot) {
    return false;
  }
  // v_dot
  if (lhs->v_dot != rhs->v_dot) {
    return false;
  }
  // w_dot
  if (lhs->w_dot != rhs->w_dot) {
    return false;
  }
  // p_dot
  if (lhs->p_dot != rhs->p_dot) {
    return false;
  }
  // q_dot
  if (lhs->q_dot != rhs->q_dot) {
    return false;
  }
  // r_dot
  if (lhs->r_dot != rhs->r_dot) {
    return false;
  }
  return true;
}

bool
ngc_interfaces__msg__NuDot__copy(
  const ngc_interfaces__msg__NuDot * input,
  ngc_interfaces__msg__NuDot * output)
{
  if (!input || !output) {
    return false;
  }
  // u_dot
  output->u_dot = input->u_dot;
  // v_dot
  output->v_dot = input->v_dot;
  // w_dot
  output->w_dot = input->w_dot;
  // p_dot
  output->p_dot = input->p_dot;
  // q_dot
  output->q_dot = input->q_dot;
  // r_dot
  output->r_dot = input->r_dot;
  return true;
}

ngc_interfaces__msg__NuDot *
ngc_interfaces__msg__NuDot__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ngc_interfaces__msg__NuDot * msg = (ngc_interfaces__msg__NuDot *)allocator.allocate(sizeof(ngc_interfaces__msg__NuDot), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ngc_interfaces__msg__NuDot));
  bool success = ngc_interfaces__msg__NuDot__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ngc_interfaces__msg__NuDot__destroy(ngc_interfaces__msg__NuDot * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ngc_interfaces__msg__NuDot__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ngc_interfaces__msg__NuDot__Sequence__init(ngc_interfaces__msg__NuDot__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ngc_interfaces__msg__NuDot * data = NULL;

  if (size) {
    data = (ngc_interfaces__msg__NuDot *)allocator.zero_allocate(size, sizeof(ngc_interfaces__msg__NuDot), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ngc_interfaces__msg__NuDot__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ngc_interfaces__msg__NuDot__fini(&data[i - 1]);
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
ngc_interfaces__msg__NuDot__Sequence__fini(ngc_interfaces__msg__NuDot__Sequence * array)
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
      ngc_interfaces__msg__NuDot__fini(&array->data[i]);
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

ngc_interfaces__msg__NuDot__Sequence *
ngc_interfaces__msg__NuDot__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ngc_interfaces__msg__NuDot__Sequence * array = (ngc_interfaces__msg__NuDot__Sequence *)allocator.allocate(sizeof(ngc_interfaces__msg__NuDot__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ngc_interfaces__msg__NuDot__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ngc_interfaces__msg__NuDot__Sequence__destroy(ngc_interfaces__msg__NuDot__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ngc_interfaces__msg__NuDot__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ngc_interfaces__msg__NuDot__Sequence__are_equal(const ngc_interfaces__msg__NuDot__Sequence * lhs, const ngc_interfaces__msg__NuDot__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ngc_interfaces__msg__NuDot__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ngc_interfaces__msg__NuDot__Sequence__copy(
  const ngc_interfaces__msg__NuDot__Sequence * input,
  ngc_interfaces__msg__NuDot__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ngc_interfaces__msg__NuDot);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ngc_interfaces__msg__NuDot * data =
      (ngc_interfaces__msg__NuDot *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ngc_interfaces__msg__NuDot__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ngc_interfaces__msg__NuDot__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ngc_interfaces__msg__NuDot__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
