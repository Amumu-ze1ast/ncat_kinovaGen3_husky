// Generated by gencpp from file kortex_driver/NetworkHandle.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_NETWORKHANDLE_H
#define KORTEX_DRIVER_MESSAGE_NETWORKHANDLE_H


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
struct NetworkHandle_
{
  typedef NetworkHandle_<ContainerAllocator> Type;

  NetworkHandle_()
    : type(0)  {
    }
  NetworkHandle_(const ContainerAllocator& _alloc)
    : type(0)  {
  (void)_alloc;
    }



   typedef uint32_t _type_type;
  _type_type type;





  typedef boost::shared_ptr< ::kortex_driver::NetworkHandle_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::NetworkHandle_<ContainerAllocator> const> ConstPtr;

}; // struct NetworkHandle_

typedef ::kortex_driver::NetworkHandle_<std::allocator<void> > NetworkHandle;

typedef boost::shared_ptr< ::kortex_driver::NetworkHandle > NetworkHandlePtr;
typedef boost::shared_ptr< ::kortex_driver::NetworkHandle const> NetworkHandleConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::NetworkHandle_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::NetworkHandle_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::NetworkHandle_<ContainerAllocator1> & lhs, const ::kortex_driver::NetworkHandle_<ContainerAllocator2> & rhs)
{
  return lhs.type == rhs.type;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::NetworkHandle_<ContainerAllocator1> & lhs, const ::kortex_driver::NetworkHandle_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::NetworkHandle_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::NetworkHandle_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::NetworkHandle_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::NetworkHandle_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::NetworkHandle_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::NetworkHandle_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::NetworkHandle_<ContainerAllocator> >
{
  static const char* value()
  {
    return "fddb7a9f6752fdb043992b8a34032ae9";
  }

  static const char* value(const ::kortex_driver::NetworkHandle_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xfddb7a9f6752fdb0ULL;
  static const uint64_t static_value2 = 0x43992b8a34032ae9ULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::NetworkHandle_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/NetworkHandle";
  }

  static const char* value(const ::kortex_driver::NetworkHandle_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::NetworkHandle_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"uint32 type\n"
;
  }

  static const char* value(const ::kortex_driver::NetworkHandle_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::NetworkHandle_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.type);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct NetworkHandle_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::NetworkHandle_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::NetworkHandle_<ContainerAllocator>& v)
  {
    s << indent << "type: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.type);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_NETWORKHANDLE_H
