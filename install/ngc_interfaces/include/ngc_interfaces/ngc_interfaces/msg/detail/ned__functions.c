// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ngc_interfaces:msg/Ned.idl
// generated code does not contain a copyright notice
#include "ngc_interfaces/msg/detail/ned__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
ngc_interfaces__msg__Ned__init(ngc_interfaces__msg__Ned * msg)
{
  if (!msg) {
    return false;
  }
  // x
  // y
  // z
  // phi
  // theta
  // psi
  return true;
}

void
ngc_interfaces__msg__Ned__fini(ngc_interfaces__msg__Ned * msg)
{
  if (!msg) {
    return;
  }
  // x
  // y
  // z
  // phi
  // theta
  // psi
}

bool
ngc_interfaces__msg__Ned__are_equal(const ngc_interfaces__msg__Ned * lhs, const ngc_interfaces__msg__Ned * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // x
  if (lhs->x != rhs->x) {
    return false;
  }
  // y
  if (lhs->y != rhs->y) {
    return false;
  }
  // z
  if (lhs->z != rhs->z) {
    return false;
  }
  // phi
  if (lhs->phi != rhs->phi) {
    return false;
  }
  // theta
  if (lhs->theta != rhs->theta) {
    return false;
  }
  // psi
  if (lhs->psi != rhs->psi) {
    return false;
  }
  return true;
}

bool
ngc_interfaces__msg__Ned__copy(
  const ngc_interfaces__msg__Ned * input,
  ngc_interfaces__msg__Ned * output)
{
  if (!input || !output) {
    return false;
  }
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  // z
  output->z = input->z;
  // phi
  output->phi = input->phi;
  // theta
  output->theta = input->theta;
  // psi
  output->psi = input->psi;
  return true;
}

ngc_interfaces__msg__Ned *
ngc_interfaces__msg__Ned__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ngc_interfaces__msg__Ned * msg = (ngc_interfaces__msg__Ned *)allocator.allocate(sizeof(ngc_interfaces__msg__Ned), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ngc_interfaces__msg__Ned));
  bool success = ngc_interfaces__msg__Ned__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ngc_interfaces__msg__Ned__destroy(ngc_interfaces__msg__Ned * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ngc_interfaces__msg__Ned__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ngc_interfaces__msg__Ned__Sequence__init(ngc_interfaces__msg__Ned__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ngc_interfaces__msg__Ned * data = NULL;

  if (size) {
    data = (ngc_interfaces__msg__Ned *)allocator.zero_allocate(size, sizeof(ngc_interfaces__msg__Ned), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ngc_interfaces__msg__Ned__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ngc_interfaces__msg__Ned__fini(&data[i - 1]);
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
ngc_interfaces__msg__Ned__Sequence__fini(ngc_interfaces__msg__Ned__Sequence * array)
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
      ngc_interfaces__msg__Ned__fini(&array->data[i]);
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

ngc_interfaces__msg__Ned__Sequence *
ngc_interfaces__msg__Ned__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ngc_interfaces__msg__Ned__Sequence * array = (ngc_interfaces__msg__Ned__Sequence *)allocator.allocate(sizeof(ngc_interfaces__msg__Ned__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ngc_interfaces__msg__Ned__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ngc_interfaces__msg__Ned__Sequence__destroy(ngc_interfaces__msg__Ned__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ngc_interfaces__msg__Ned__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ngc_interfaces__msg__Ned__Sequence__are_equal(const ngc_interfaces__msg__Ned__Sequence * lhs, const ngc_interfaces__msg__Ned__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ngc_interfaces__msg__Ned__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ngc_interfaces__msg__Ned__Sequence__copy(
  const ngc_interfaces__msg__Ned__Sequence * input,
  ngc_interfaces__msg__Ned__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ngc_interfaces__msg__Ned);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ngc_interfaces__msg__Ned * data =
      (ngc_interfaces__msg__Ned *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ngc_interfaces__msg__Ned__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ngc_interfaces__msg__Ned__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ngc_interfaces__msg__Ned__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
