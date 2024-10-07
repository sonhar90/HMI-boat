// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ngc_interfaces:msg/NuDot.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ngc_interfaces/msg/nu_dot.hpp"


#ifndef NGC_INTERFACES__MSG__DETAIL__NU_DOT__STRUCT_HPP_
#define NGC_INTERFACES__MSG__DETAIL__NU_DOT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ngc_interfaces__msg__NuDot __attribute__((deprecated))
#else
# define DEPRECATED__ngc_interfaces__msg__NuDot __declspec(deprecated)
#endif

namespace ngc_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct NuDot_
{
  using Type = NuDot_<ContainerAllocator>;

  explicit NuDot_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->u_dot = 0.0f;
      this->v_dot = 0.0f;
      this->w_dot = 0.0f;
      this->p_dot = 0.0f;
      this->q_dot = 0.0f;
      this->r_dot = 0.0f;
    }
  }

  explicit NuDot_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->u_dot = 0.0f;
      this->v_dot = 0.0f;
      this->w_dot = 0.0f;
      this->p_dot = 0.0f;
      this->q_dot = 0.0f;
      this->r_dot = 0.0f;
    }
  }

  // field types and members
  using _u_dot_type =
    float;
  _u_dot_type u_dot;
  using _v_dot_type =
    float;
  _v_dot_type v_dot;
  using _w_dot_type =
    float;
  _w_dot_type w_dot;
  using _p_dot_type =
    float;
  _p_dot_type p_dot;
  using _q_dot_type =
    float;
  _q_dot_type q_dot;
  using _r_dot_type =
    float;
  _r_dot_type r_dot;

  // setters for named parameter idiom
  Type & set__u_dot(
    const float & _arg)
  {
    this->u_dot = _arg;
    return *this;
  }
  Type & set__v_dot(
    const float & _arg)
  {
    this->v_dot = _arg;
    return *this;
  }
  Type & set__w_dot(
    const float & _arg)
  {
    this->w_dot = _arg;
    return *this;
  }
  Type & set__p_dot(
    const float & _arg)
  {
    this->p_dot = _arg;
    return *this;
  }
  Type & set__q_dot(
    const float & _arg)
  {
    this->q_dot = _arg;
    return *this;
  }
  Type & set__r_dot(
    const float & _arg)
  {
    this->r_dot = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ngc_interfaces::msg::NuDot_<ContainerAllocator> *;
  using ConstRawPtr =
    const ngc_interfaces::msg::NuDot_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ngc_interfaces::msg::NuDot_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ngc_interfaces::msg::NuDot_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ngc_interfaces::msg::NuDot_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ngc_interfaces::msg::NuDot_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ngc_interfaces::msg::NuDot_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ngc_interfaces::msg::NuDot_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ngc_interfaces::msg::NuDot_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ngc_interfaces::msg::NuDot_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ngc_interfaces__msg__NuDot
    std::shared_ptr<ngc_interfaces::msg::NuDot_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ngc_interfaces__msg__NuDot
    std::shared_ptr<ngc_interfaces::msg::NuDot_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const NuDot_ & other) const
  {
    if (this->u_dot != other.u_dot) {
      return false;
    }
    if (this->v_dot != other.v_dot) {
      return false;
    }
    if (this->w_dot != other.w_dot) {
      return false;
    }
    if (this->p_dot != other.p_dot) {
      return false;
    }
    if (this->q_dot != other.q_dot) {
      return false;
    }
    if (this->r_dot != other.r_dot) {
      return false;
    }
    return true;
  }
  bool operator!=(const NuDot_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct NuDot_

// alias to use template instance with default allocator
using NuDot =
  ngc_interfaces::msg::NuDot_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ngc_interfaces

#endif  // NGC_INTERFACES__MSG__DETAIL__NU_DOT__STRUCT_HPP_
