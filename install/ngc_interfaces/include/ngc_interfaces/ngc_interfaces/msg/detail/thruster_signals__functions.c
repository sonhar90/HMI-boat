// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ngc_interfaces:msg/ThrusterSignals.idl
// generated code does not contain a copyright notice
#include "ngc_interfaces/msg/detail/thruster_signals__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
ngc_interfaces__msg__ThrusterSignals__init(ngc_interfaces__msg__ThrusterSignals * msg)
{
  if (!msg) {
    return false;
  }
  // thruster_id
  // rps
  // pitch
  // azimuth_deg
  // active
  // error
  return true;
}

void
ngc_interfaces__msg__ThrusterSignals__fini(ngc_interfaces__msg__ThrusterSignals * msg)
{
  if (!msg) {
    return;
  }
  // thruster_id
  // rps
  // pitch
  // azimuth_deg
  // active
  // error
}

bool
ngc_interfaces__msg__ThrusterSignals__are_equal(const ngc_interfaces__msg__ThrusterSignals * lhs, const ngc_interfaces__msg__ThrusterSignals * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // thruster_id
  if (lhs->thruster_id != rhs->thruster_id) {
    return false;
  }
  // rps
  if (lhs->rps != rhs->rps) {
    return false;
  }
  // pitch
  if (lhs->pitch != rhs->pitch) {
    return false;
  }
  // azimuth_deg
  if (lhs->azimuth_deg != rhs->azimuth_deg) {
    return false;
  }
  // active
  if (lhs->active != rhs->active) {
    return false;
  }
  // error
  if (lhs->error != rhs->error) {
    return false;
  }
  return true;
}

bool
ngc_interfaces__msg__ThrusterSignals__copy(
  const ngc_interfaces__msg__ThrusterSignals * input,
  ngc_interfaces__msg__ThrusterSignals * output)
{
  if (!input || !output) {
    return false;
  }
  // thruster_id
  output->thruster_id = input->thruster_id;
  // rps
  output->rps = input->rps;
  // pitch
  output->pitch = input->pitch;
  // azimuth_deg
  output->azimuth_deg = input->azimuth_deg;
  // active
  output->active = input->active;
  // error
  output->error = input->error;
  return true;
}

ngc_interfaces__msg__ThrusterSignals *
ngc_interfaces__msg__ThrusterSignals__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ngc_interfaces__msg__ThrusterSignals * msg = (ngc_interfaces__msg__ThrusterSignals *)allocator.allocate(sizeof(ngc_interfaces__msg__ThrusterSignals), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ngc_interfaces__msg__ThrusterSignals));
  bool success = ngc_interfaces__msg__ThrusterSignals__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ngc_interfaces__msg__ThrusterSignals__destroy(ngc_interfaces__msg__ThrusterSignals * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ngc_interfaces__msg__ThrusterSignals__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ngc_interfaces__msg__ThrusterSignals__Sequence__init(ngc_interfaces__msg__ThrusterSignals__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ngc_interfaces__msg__ThrusterSignals * data = NULL;

  if (size) {
    data = (ngc_interfaces__msg__ThrusterSignals *)allocator.zero_allocate(size, sizeof(ngc_interfaces__msg__ThrusterSignals), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ngc_interfaces__msg__ThrusterSignals__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ngc_interfaces__msg__ThrusterSignals__fini(&data[i - 1]);
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
ngc_interfaces__msg__ThrusterSignals__Sequence__fini(ngc_interfaces__msg__ThrusterSignals__Sequence * array)
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
      ngc_interfaces__msg__ThrusterSignals__fini(&array->data[i]);
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

ngc_interfaces__msg__ThrusterSignals__Sequence *
ngc_interfaces__msg__ThrusterSignals__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ngc_interfaces__msg__ThrusterSignals__Sequence * array = (ngc_interfaces__msg__ThrusterSignals__Sequence *)allocator.allocate(sizeof(ngc_interfaces__msg__ThrusterSignals__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ngc_interfaces__msg__ThrusterSignals__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ngc_interfaces__msg__ThrusterSignals__Sequence__destroy(ngc_interfaces__msg__ThrusterSignals__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ngc_interfaces__msg__ThrusterSignals__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ngc_interfaces__msg__ThrusterSignals__Sequence__are_equal(const ngc_interfaces__msg__ThrusterSignals__Sequence * lhs, const ngc_interfaces__msg__ThrusterSignals__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ngc_interfaces__msg__ThrusterSignals__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ngc_interfaces__msg__ThrusterSignals__Sequence__copy(
  const ngc_interfaces__msg__ThrusterSignals__Sequence * input,
  ngc_interfaces__msg__ThrusterSignals__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ngc_interfaces__msg__ThrusterSignals);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ngc_interfaces__msg__ThrusterSignals * data =
      (ngc_interfaces__msg__ThrusterSignals *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ngc_interfaces__msg__ThrusterSignals__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ngc_interfaces__msg__ThrusterSignals__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ngc_interfaces__msg__ThrusterSignals__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
