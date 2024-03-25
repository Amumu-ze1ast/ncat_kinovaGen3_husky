// Generated by gencpp from file kortex_driver/JointTorque.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_JOINTTORQUE_H
#define KORTEX_DRIVER_MESSAGE_JOINTTORQUE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace kortex_driver
{
template <class ContainerAllocator>
struct JointTorque_
{
  typedef JointTorque_<ContainerAllocator> Type;

  JointTorque_()
    : joint_identifier(0)
    , value(0.0)
    , duration(0)  {
    }
  JointTorque_(const ContainerAllocator& _alloc)
    : joint_identifier(0)
    , value(0.0)
    , duration(0)  {
  (void)_alloc;
    }



   typedef uint32_t _joint_identifier_type;
  _joint_identifier_type joint_identifier;

   typedef float _value_type;
  _value_type value;

   typedef uint32_t _duration_type;
  _duration_type duration;





  typedef boost::shared_ptr< ::kortex_driver::JointTorque_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::JointTorque_<ContainerAllocator> const> ConstPtr;

}; // struct JointTorque_

typedef ::kortex_driver::JointTorque_<std::allocator<void> > JointTorque;

typedef boost::shared_ptr< ::kortex_driver::JointTorque > JointTorquePtr;
typedef boost::shared_ptr< ::kortex_driver::JointTorque const> JointTorqueConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::JointTorque_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::JointTorque_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::JointTorque_<ContainerAllocator1> & lhs, const ::kortex_driver::JointTorque_<ContainerAllocator2> & rhs)
{
  return lhs.joint_identifier == rhs.joint_identifier &&
    lhs.value == rhs.value &&
    lhs.duration == rhs.duration;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::JointTorque_<ContainerAllocator1> & lhs, const ::kortex_driver::JointTorque_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::JointTorque_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::JointTorque_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::JointTorque_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::JointTorque_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::JointTorque_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::JointTorque_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::JointTorque_<ContainerAllocator> >
{
  static const char* value()
  {
    return "fd26fce9b45cd51d86512ba0cf80d2bd";
  }

  static const char* value(const ::kortex_driver::JointTorque_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xfd26fce9b45cd51dULL;
  static const uint64_t static_value2 = 0x86512ba0cf80d2bdULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::JointTorque_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/JointTorque";
  }

  static const char* value(const ::kortex_driver::JointTorque_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::JointTorque_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"uint32 joint_identifier\n"
"float32 value\n"
"uint32 duration\n"
;
  }

  static const char* value(const ::kortex_driver::JointTorque_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::JointTorque_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.joint_identifier);
      stream.next(m.value);
      stream.next(m.duration);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct JointTorque_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::JointTorque_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::JointTorque_<ContainerAllocator>& v)
  {
    s << indent << "joint_identifier: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.joint_identifier);
    s << indent << "value: ";
    Printer<float>::stream(s, indent + "  ", v.value);
    s << indent << "duration: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.duration);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_JOINTTORQUE_H
