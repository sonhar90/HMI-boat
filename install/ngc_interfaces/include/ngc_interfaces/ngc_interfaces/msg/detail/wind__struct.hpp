// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ngc_interfaces:msg/Wind.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ngc_interfaces/msg/wind.hpp"


#ifndef NGC_INTERFACES__MSG__DETAIL__WIND__STRUCT_HPP_
#define NGC_INTERFACES__MSG__DETAIL__WIND__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ngc_interfaces__msg__Wind __attribute__((deprecated))
#else
# define DEPRECATED__ngc_interfaces__msg__Wind __declspec(deprecated)
#endif

namespace ngc_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Wind_
{
  using Type = Wind_<ContainerAllocator>;

  explicit Wind_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->direction_relative_deg = 0.0f;
      this->magnitude_ms = 0.0f;
    }
  }

  explicit Wind_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->direction_relative_deg = 0.0f;
      this->magnitude_ms = 0.0f;
    }
  }

  // field types and members
  using _direction_relative_deg_type =
    float;
  _direction_relative_deg_type direction_relative_deg;
  using _magnitude_ms_type =
    float;
  _magnitude_ms_type magnitude_ms;

  // setters for named parameter idiom
  Type & set__direction_relative_deg(
    const float & _arg)
  {
    this->direction_relative_deg = _arg;
    return *this;
  }
  Type & set__magnitude_ms(
    const float & _arg)
  {
    this->magnitude_ms = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ngc_interfaces::msg::Wind_<ContainerAllocator> *;
  using ConstRawPtr =
    const ngc_interfaces::msg::Wind_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ngc_interfaces::msg::Wind_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ngc_interfaces::msg::Wind_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ngc_interfaces::msg::Wind_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ngc_interfaces::msg::Wind_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ngc_interfaces::msg::Wind_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ngc_interfaces::msg::Wind_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ngc_interfaces::msg::Wind_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ngc_interfaces::msg::Wind_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ngc_interfaces__msg__Wind
    std::shared_ptr<ngc_interfaces::msg::Wind_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ngc_interfaces__msg__Wind
    std::shared_ptr<ngc_interfaces::msg::Wind_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Wind_ & other) const
  {
    if (this->direction_relative_deg != other.direction_relative_deg) {
      return false;
    }
    if (this->magnitude_ms != other.magnitude_ms) {
      return false;
    }
    return true;
  }
  bool operator!=(const Wind_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Wind_

// alias to use template instance with default allocator
using Wind =
  ngc_interfaces::msg::Wind_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ngc_interfaces

#endif  // NGC_INTERFACES__MSG__DETAIL__WIND__STRUCT_HPP_
