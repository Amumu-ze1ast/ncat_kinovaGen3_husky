// Generated by gencpp from file kortex_driver/MappingHandle.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_MAPPINGHANDLE_H
#define KORTEX_DRIVER_MESSAGE_MAPPINGHANDLE_H


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
struct MappingHandle_
{
  typedef MappingHandle_<ContainerAllocator> Type;

  MappingHandle_()
    : identifier(0)
    , permission(0)  {
    }
  MappingHandle_(const ContainerAllocator& _alloc)
    : identifier(0)
    , permission(0)  {
  (void)_alloc;
    }



   typedef uint32_t _identifier_type;
  _identifier_type identifier;

   typedef uint32_t _permission_type;
  _permission_type permission;





  typedef boost::shared_ptr< ::kortex_driver::MappingHandle_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::MappingHandle_<ContainerAllocator> const> ConstPtr;

}; // struct MappingHandle_

typedef ::kortex_driver::MappingHandle_<std::allocator<void> > MappingHandle;

typedef boost::shared_ptr< ::kortex_driver::MappingHandle > MappingHandlePtr;
typedef boost::shared_ptr< ::kortex_driver::MappingHandle const> MappingHandleConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::MappingHandle_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::MappingHandle_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::MappingHandle_<ContainerAllocator1> & lhs, const ::kortex_driver::MappingHandle_<ContainerAllocator2> & rhs)
{
  return lhs.identifier == rhs.identifier &&
    lhs.permission == rhs.permission;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::MappingHandle_<ContainerAllocator1> & lhs, const ::kortex_driver::MappingHandle_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::MappingHandle_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::MappingHandle_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::MappingHandle_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::MappingHandle_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::MappingHandle_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::MappingHandle_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::MappingHandle_<ContainerAllocator> >
{
  static const char* value()
  {
    return "10d841c6abc3fd3596d9b10510d50074";
  }

  static const char* value(const ::kortex_driver::MappingHandle_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x10d841c6abc3fd35ULL;
  static const uint64_t static_value2 = 0x96d9b10510d50074ULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::MappingHandle_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/MappingHandle";
  }

  static const char* value(const ::kortex_driver::MappingHandle_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::MappingHandle_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"uint32 identifier\n"
"uint32 permission\n"
;
  }

  static const char* value(const ::kortex_driver::MappingHandle_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::MappingHandle_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.identifier);
      stream.next(m.permission);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MappingHandle_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::MappingHandle_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::MappingHandle_<ContainerAllocator>& v)
  {
    s << indent << "identifier: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.identifier);
    s << indent << "permission: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.permission);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_MAPPINGHANDLE_H
