// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from ngc_interfaces:msg/Wind.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ngc_interfaces/msg/wind.h"


#ifndef NGC_INTERFACES__MSG__DETAIL__WIND__FUNCTIONS_H_
#define NGC_INTERFACES__MSG__DETAIL__WIND__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_runtime_c/type_description/type_description__struct.h"
#include "rosidl_runtime_c/type_description/type_source__struct.h"
#include "rosidl_runtime_c/type_hash.h"
#include "rosidl_runtime_c/visibility_control.h"
#include "ngc_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "ngc_interfaces/msg/detail/wind__struct.h"

/// Initialize msg/Wind message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * ngc_interfaces__msg__Wind
 * )) before or use
 * ngc_interfaces__msg__Wind__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_ngc_interfaces
bool
ngc_interfaces__msg__Wind__init(ngc_interfaces__msg__Wind * msg);

/// Finalize msg/Wind message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ngc_interfaces
void
ngc_interfaces__msg__Wind__fini(ngc_interfaces__msg__Wind * msg);

/// Create msg/Wind message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * ngc_interfaces__msg__Wind__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ngc_interfaces
ngc_interfaces__msg__Wind *
ngc_interfaces__msg__Wind__create(void);

/// Destroy msg/Wind message.
/**
 * It calls
 * ngc_interfaces__msg__Wind__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ngc_interfaces
void
ngc_interfaces__msg__Wind__destroy(ngc_interfaces__msg__Wind * msg);

/// Check for msg/Wind message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ngc_interfaces
bool
ngc_interfaces__msg__Wind__are_equal(const ngc_interfaces__msg__Wind * lhs, const ngc_interfaces__msg__Wind * rhs);

/// Copy a msg/Wind message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ngc_interfaces
bool
ngc_interfaces__msg__Wind__copy(
  const ngc_interfaces__msg__Wind * input,
  ngc_interfaces__msg__Wind * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_ngc_interfaces
const rosidl_type_hash_t *
ngc_interfaces__msg__Wind__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_ngc_interfaces
const rosidl_runtime_c__type_description__TypeDescription *
ngc_interfaces__msg__Wind__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_ngc_interfaces
const rosidl_runtime_c__type_description__TypeSource *
ngc_interfaces__msg__Wind__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_ngc_interfaces
const rosidl_runtime_c__type_description__TypeSource__Sequence *
ngc_interfaces__msg__Wind__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of msg/Wind messages.
/**
 * It allocates the memory for the number of elements and calls
 * ngc_interfaces__msg__Wind__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_ngc_interfaces
bool
ngc_interfaces__msg__Wind__Sequence__init(ngc_interfaces__msg__Wind__Sequence * array, size_t size);

/// Finalize array of msg/Wind messages.
/**
 * It calls
 * ngc_interfaces__msg__Wind__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ngc_interfaces
void
ngc_interfaces__msg__Wind__Sequence__fini(ngc_interfaces__msg__Wind__Sequence * array);

/// Create array of msg/Wind messages.
/**
 * It allocates the memory for the array and calls
 * ngc_interfaces__msg__Wind__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ngc_interfaces
ngc_interfaces__msg__Wind__Sequence *
ngc_interfaces__msg__Wind__Sequence__create(size_t size);

/// Destroy array of msg/Wind messages.
/**
 * It calls
 * ngc_interfaces__msg__Wind__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ngc_interfaces
void
ngc_interfaces__msg__Wind__Sequence__destroy(ngc_interfaces__msg__Wind__Sequence * array);

/// Check for msg/Wind message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ngc_interfaces
bool
ngc_interfaces__msg__Wind__Sequence__are_equal(const ngc_interfaces__msg__Wind__Sequence * lhs, const ngc_interfaces__msg__Wind__Sequence * rhs);

/// Copy an array of msg/Wind messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ngc_interfaces
bool
ngc_interfaces__msg__Wind__Sequence__copy(
  const ngc_interfaces__msg__Wind__Sequence * input,
  ngc_interfaces__msg__Wind__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // NGC_INTERFACES__MSG__DETAIL__WIND__FUNCTIONS_H_
