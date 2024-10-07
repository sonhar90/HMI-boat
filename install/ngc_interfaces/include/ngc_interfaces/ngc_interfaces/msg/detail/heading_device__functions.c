// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ngc_interfaces:msg/HeadingDevice.idl
// generated code does not contain a copyright notice
#include "ngc_interfaces/msg/detail/heading_device__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
ngc_interfaces__msg__HeadingDevice__init(ngc_interfaces__msg__HeadingDevice * msg)
{
  if (!msg) {
    return false;
  }
  // heading
  // rot
  // valid_signal
  return true;
}

void
ngc_interfaces__msg__HeadingDevice__fini(ngc_interfaces__msg__HeadingDevice * msg)
{
  if (!msg) {
    return;
  }
  // heading
  // rot
  // valid_signal
}

bool
ngc_interfaces__msg__HeadingDevice__are_equal(const ngc_interfaces__msg__HeadingDevice * lhs, const ngc_interfaces__msg__HeadingDevice * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // heading
  if (lhs->heading != rhs->heading) {
    return false;
  }
  // rot
  if (lhs->rot != rhs->rot) {
    return false;
  }
  // valid_signal
  if (lhs->valid_signal != rhs->valid_signal) {
    return false;
  }
  return true;
}

bool
ngc_interfaces__msg__HeadingDevice__copy(
  const ngc_interfaces__msg__HeadingDevice * input,
  ngc_interfaces__msg__HeadingDevice * output)
{
  if (!input || !output) {
    return false;
  }
  // heading
  output->heading = input->heading;
  // rot
  output->rot = input->rot;
  // valid_signal
  output->valid_signal = input->valid_signal;
  return true;
}

ngc_interfaces__msg__HeadingDevice *
ngc_interfaces__msg__HeadingDevice__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ngc_interfaces__msg__HeadingDevice * msg = (ngc_interfaces__msg__HeadingDevice *)allocator.allocate(sizeof(ngc_interfaces__msg__HeadingDevice), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ngc_interfaces__msg__HeadingDevice));
  bool success = ngc_interfaces__msg__HeadingDevice__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ngc_interfaces__msg__HeadingDevice__destroy(ngc_interfaces__msg__HeadingDevice * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ngc_interfaces__msg__HeadingDevice__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ngc_interfaces__msg__HeadingDevice__Sequence__init(ngc_interfaces__msg__HeadingDevice__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ngc_interfaces__msg__HeadingDevice * data = NULL;

  if (size) {
    data = (ngc_interfaces__msg__HeadingDevice *)allocator.zero_allocate(size, sizeof(ngc_interfaces__msg__HeadingDevice), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ngc_interfaces__msg__HeadingDevice__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ngc_interfaces__msg__HeadingDevice__fini(&data[i - 1]);
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
ngc_interfaces__msg__HeadingDevice__Sequence__fini(ngc_interfaces__msg__HeadingDevice__Sequence * array)
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
      ngc_interfaces__msg__HeadingDevice__fini(&array->data[i]);
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

ngc_interfaces__msg__HeadingDevice__Sequence *
ngc_interfaces__msg__HeadingDevice__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ngc_interfaces__msg__HeadingDevice__Sequence * array = (ngc_interfaces__msg__HeadingDevice__Sequence *)allocator.allocate(sizeof(ngc_interfaces__msg__HeadingDevice__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ngc_interfaces__msg__HeadingDevice__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ngc_interfaces__msg__HeadingDevice__Sequence__destroy(ngc_interfaces__msg__HeadingDevice__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ngc_interfaces__msg__HeadingDevice__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ngc_interfaces__msg__HeadingDevice__Sequence__are_equal(const ngc_interfaces__msg__HeadingDevice__Sequence * lhs, const ngc_interfaces__msg__HeadingDevice__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ngc_interfaces__msg__HeadingDevice__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ngc_interfaces__msg__HeadingDevice__Sequence__copy(
  const ngc_interfaces__msg__HeadingDevice__Sequence * input,
  ngc_interfaces__msg__HeadingDevice__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ngc_interfaces__msg__HeadingDevice);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ngc_interfaces__msg__HeadingDevice * data =
      (ngc_interfaces__msg__HeadingDevice *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ngc_interfaces__msg__HeadingDevice__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ngc_interfaces__msg__HeadingDevice__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ngc_interfaces__msg__HeadingDevice__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
