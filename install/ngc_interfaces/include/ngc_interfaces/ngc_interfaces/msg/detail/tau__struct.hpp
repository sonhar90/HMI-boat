// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ngc_interfaces:msg/Tau.idl
// generated code does not contain a copyright notice

#ifndef NGC_INTERFACES__MSG__DETAIL__TAU__STRUCT_HPP_
#define NGC_INTERFACES__MSG__DETAIL__TAU__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ngc_interfaces__msg__Tau __attribute__((deprecated))
#else
# define DEPRECATED__ngc_interfaces__msg__Tau __declspec(deprecated)
#endif

namespace ngc_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Tau_
{
  using Type = Tau_<ContainerAllocator>;

  explicit Tau_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->surge_x = 0.0f;
      this->sway_y = 0.0f;
      this->heave_z = 0.0f;
      this->roll_k = 0.0f;
      this->pitch_m = 0.0f;
      this->yaw_n = 0.0f;
    }
  }

  explicit Tau_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->surge_x = 0.0f;
      this->sway_y = 0.0f;
      this->heave_z = 0.0f;
      this->roll_k = 0.0f;
      this->pitch_m = 0.0f;
      this->yaw_n = 0.0f;
    }
  }

  // field types and members
  using _surge_x_type =
    float;
  _surge_x_type surge_x;
  using _sway_y_type =
    float;
  _sway_y_type sway_y;
  using _heave_z_type =
    float;
  _heave_z_type heave_z;
  using _roll_k_type =
    float;
  _roll_k_type roll_k;
  using _pitch_m_type =
    float;
  _pitch_m_type pitch_m;
  using _yaw_n_type =
    float;
  _yaw_n_type yaw_n;

  // setters for named parameter idiom
  Type & set__surge_x(
    const float & _arg)
  {
    this->surge_x = _arg;
    return *this;
  }
  Type & set__sway_y(
    const float & _arg)
  {
    this->sway_y = _arg;
    return *this;
  }
  Type & set__heave_z(
    const float & _arg)
  {
    this->heave_z = _arg;
    return *this;
  }
  Type & set__roll_k(
    const float & _arg)
  {
    this->roll_k = _arg;
    return *this;
  }
  Type & set__pitch_m(
    const float & _arg)
  {
    this->pitch_m = _arg;
    return *this;
  }
  Type & set__yaw_n(
    const float & _arg)
  {
    this->yaw_n = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ngc_interfaces::msg::Tau_<ContainerAllocator> *;
  using ConstRawPtr =
    const ngc_interfaces::msg::Tau_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ngc_interfaces::msg::Tau_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ngc_interfaces::msg::Tau_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ngc_interfaces::msg::Tau_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ngc_interfaces::msg::Tau_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ngc_interfaces::msg::Tau_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ngc_interfaces::msg::Tau_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ngc_interfaces::msg::Tau_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ngc_interfaces::msg::Tau_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ngc_interfaces__msg__Tau
    std::shared_ptr<ngc_interfaces::msg::Tau_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ngc_interfaces__msg__Tau
    std::shared_ptr<ngc_interfaces::msg::Tau_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Tau_ & other) const
  {
    if (this->surge_x != other.surge_x) {
      return false;
    }
    if (this->sway_y != other.sway_y) {
      return false;
    }
    if (this->heave_z != other.heave_z) {
      return false;
    }
    if (this->roll_k != other.roll_k) {
      return false;
    }
    if (this->pitch_m != other.pitch_m) {
      return false;
    }
    if (this->yaw_n != other.yaw_n) {
      return false;
    }
    return true;
  }
  bool operator!=(const Tau_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Tau_

// alias to use template instance with default allocator
using Tau =
  ngc_interfaces::msg::Tau_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ngc_interfaces

#endif  // NGC_INTERFACES__MSG__DETAIL__TAU__STRUCT_HPP_
