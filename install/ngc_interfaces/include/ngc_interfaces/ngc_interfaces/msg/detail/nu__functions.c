// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ngc_interfaces:msg/Nu.idl
// generated code does not contain a copyright notice
#include "ngc_interfaces/msg/detail/nu__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
ngc_interfaces__msg__Nu__init(ngc_interfaces__msg__Nu * msg)
{
  if (!msg) {
    return false;
  }
  // u
  // v
  // w
  // p
  // q
  // r
  return true;
}

void
ngc_interfaces__msg__Nu__fini(ngc_interfaces__msg__Nu * msg)
{
  if (!msg) {
    return;
  }
  // u
  // v
  // w
  // p
  // q
  // r
}

bool
ngc_interfaces__msg__Nu__are_equal(const ngc_interfaces__msg__Nu * lhs, const ngc_interfaces__msg__Nu * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // u
  if (lhs->u != rhs->u) {
    return false;
  }
  // v
  if (lhs->v != rhs->v) {
    return false;
  }
  // w
  if (lhs->w != rhs->w) {
    return false;
  }
  // p
  if (lhs->p != rhs->p) {
    return false;
  }
  // q
  if (lhs->q != rhs->q) {
    return false;
  }
  // r
  if (lhs->r != rhs->r) {
    return false;
  }
  return true;
}

bool
ngc_interfaces__msg__Nu__copy(
  const ngc_interfaces__msg__Nu * input,
  ngc_interfaces__msg__Nu * output)
{
  if (!input || !output) {
    return false;
  }
  // u
  output->u = input->u;
  // v
  output->v = input->v;
  // w
  output->w = input->w;
  // p
  output->p = input->p;
  // q
  output->q = input->q;
  // r
  output->r = input->r;
  return true;
}

ngc_interfaces__msg__Nu *
ngc_interfaces__msg__Nu__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ngc_interfaces__msg__Nu * msg = (ngc_interfaces__msg__Nu *)allocator.allocate(sizeof(ngc_interfaces__msg__Nu), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ngc_interfaces__msg__Nu));
  bool success = ngc_interfaces__msg__Nu__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ngc_interfaces__msg__Nu__destroy(ngc_interfaces__msg__Nu * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ngc_interfaces__msg__Nu__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ngc_interfaces__msg__Nu__Sequence__init(ngc_interfaces__msg__Nu__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ngc_interfaces__msg__Nu * data = NULL;

  if (size) {
    data = (ngc_interfaces__msg__Nu *)allocator.zero_allocate(size, sizeof(ngc_interfaces__msg__Nu), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ngc_interfaces__msg__Nu__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ngc_interfaces__msg__Nu__fini(&data[i - 1]);
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
ngc_interfaces__msg__Nu__Sequence__fini(ngc_interfaces__msg__Nu__Sequence * array)
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
      ngc_interfaces__msg__Nu__fini(&array->data[i]);
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

ngc_interfaces__msg__Nu__Sequence *
ngc_interfaces__msg__Nu__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ngc_interfaces__msg__Nu__Sequence * array = (ngc_interfaces__msg__Nu__Sequence *)allocator.allocate(sizeof(ngc_interfaces__msg__Nu__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ngc_interfaces__msg__Nu__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ngc_interfaces__msg__Nu__Sequence__destroy(ngc_interfaces__msg__Nu__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ngc_interfaces__msg__Nu__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ngc_interfaces__msg__Nu__Sequence__are_equal(const ngc_interfaces__msg__Nu__Sequence * lhs, const ngc_interfaces__msg__Nu__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ngc_interfaces__msg__Nu__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ngc_interfaces__msg__Nu__Sequence__copy(
  const ngc_interfaces__msg__Nu__Sequence * input,
  ngc_interfaces__msg__Nu__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ngc_interfaces__msg__Nu);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ngc_interfaces__msg__Nu * data =
      (ngc_interfaces__msg__Nu *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ngc_interfaces__msg__Nu__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ngc_interfaces__msg__Nu__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ngc_interfaces__msg__Nu__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
