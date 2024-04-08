// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ngc_interfaces:msg/Nu.idl
// generated code does not contain a copyright notice

#ifndef NGC_INTERFACES__MSG__DETAIL__NU__STRUCT_HPP_
#define NGC_INTERFACES__MSG__DETAIL__NU__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ngc_interfaces__msg__Nu __attribute__((deprecated))
#else
# define DEPRECATED__ngc_interfaces__msg__Nu __declspec(deprecated)
#endif

namespace ngc_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Nu_
{
  using Type = Nu_<ContainerAllocator>;

  explicit Nu_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->u = 0.0f;
      this->v = 0.0f;
      this->w = 0.0f;
      this->p = 0.0f;
      this->q = 0.0f;
      this->r = 0.0f;
    }
  }

  explicit Nu_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->u = 0.0f;
      this->v = 0.0f;
      this->w = 0.0f;
      this->p = 0.0f;
      this->q = 0.0f;
      this->r = 0.0f;
    }
  }

  // field types and members
  using _u_type =
    float;
  _u_type u;
  using _v_type =
    float;
  _v_type v;
  using _w_type =
    float;
  _w_type w;
  using _p_type =
    float;
  _p_type p;
  using _q_type =
    float;
  _q_type q;
  using _r_type =
    float;
  _r_type r;

  // setters for named parameter idiom
  Type & set__u(
    const float & _arg)
  {
    this->u = _arg;
    return *this;
  }
  Type & set__v(
    const float & _arg)
  {
    this->v = _arg;
    return *this;
  }
  Type & set__w(
    const float & _arg)
  {
    this->w = _arg;
    return *this;
  }
  Type & set__p(
    const float & _arg)
  {
    this->p = _arg;
    return *this;
  }
  Type & set__q(
    const float & _arg)
  {
    this->q = _arg;
    return *this;
  }
  Type & set__r(
    const float & _arg)
  {
    this->r = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ngc_interfaces::msg::Nu_<ContainerAllocator> *;
  using ConstRawPtr =
    const ngc_interfaces::msg::Nu_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ngc_interfaces::msg::Nu_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ngc_interfaces::msg::Nu_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ngc_interfaces::msg::Nu_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ngc_interfaces::msg::Nu_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ngc_interfaces::msg::Nu_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ngc_interfaces::msg::Nu_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ngc_interfaces::msg::Nu_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ngc_interfaces::msg::Nu_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ngc_interfaces__msg__Nu
    std::shared_ptr<ngc_interfaces::msg::Nu_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ngc_interfaces__msg__Nu
    std::shared_ptr<ngc_interfaces::msg::Nu_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Nu_ & other) const
  {
    if (this->u != other.u) {
      return false;
    }
    if (this->v != other.v) {
      return false;
    }
    if (this->w != other.w) {
      return false;
    }
    if (this->p != other.p) {
      return false;
    }
    if (this->q != other.q) {
      return false;
    }
    if (this->r != other.r) {
      return false;
    }
    return true;
  }
  bool operator!=(const Nu_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Nu_

// alias to use template instance with default allocator
using Nu =
  ngc_interfaces::msg::Nu_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ngc_interfaces

#endif  // NGC_INTERFACES__MSG__DETAIL__NU__STRUCT_HPP_
