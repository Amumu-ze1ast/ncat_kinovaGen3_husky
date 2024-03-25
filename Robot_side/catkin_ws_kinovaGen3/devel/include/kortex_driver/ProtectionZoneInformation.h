// Generated by gencpp from file kortex_driver/ProtectionZoneInformation.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_PROTECTIONZONEINFORMATION_H
#define KORTEX_DRIVER_MESSAGE_PROTECTIONZONEINFORMATION_H


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
struct ProtectionZoneInformation_
{
  typedef ProtectionZoneInformation_<ContainerAllocator> Type;

  ProtectionZoneInformation_()
    : event(0)  {
    }
  ProtectionZoneInformation_(const ContainerAllocator& _alloc)
    : event(0)  {
  (void)_alloc;
    }



   typedef uint32_t _event_type;
  _event_type event;





  typedef boost::shared_ptr< ::kortex_driver::ProtectionZoneInformation_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::ProtectionZoneInformation_<ContainerAllocator> const> ConstPtr;

}; // struct ProtectionZoneInformation_

typedef ::kortex_driver::ProtectionZoneInformation_<std::allocator<void> > ProtectionZoneInformation;

typedef boost::shared_ptr< ::kortex_driver::ProtectionZoneInformation > ProtectionZoneInformationPtr;
typedef boost::shared_ptr< ::kortex_driver::ProtectionZoneInformation const> ProtectionZoneInformationConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::ProtectionZoneInformation_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::ProtectionZoneInformation_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::ProtectionZoneInformation_<ContainerAllocator1> & lhs, const ::kortex_driver::ProtectionZoneInformation_<ContainerAllocator2> & rhs)
{
  return lhs.event == rhs.event;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::ProtectionZoneInformation_<ContainerAllocator1> & lhs, const ::kortex_driver::ProtectionZoneInformation_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::ProtectionZoneInformation_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::ProtectionZoneInformation_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::ProtectionZoneInformation_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::ProtectionZoneInformation_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::ProtectionZoneInformation_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::ProtectionZoneInformation_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::ProtectionZoneInformation_<ContainerAllocator> >
{
  static const char* value()
  {
    return "85038978649328eee44c6513e8ebb7e6";
  }

  static const char* value(const ::kortex_driver::ProtectionZoneInformation_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x85038978649328eeULL;
  static const uint64_t static_value2 = 0xe44c6513e8ebb7e6ULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::ProtectionZoneInformation_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/ProtectionZoneInformation";
  }

  static const char* value(const ::kortex_driver::ProtectionZoneInformation_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::ProtectionZoneInformation_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"uint32 event\n"
;
  }

  static const char* value(const ::kortex_driver::ProtectionZoneInformation_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::ProtectionZoneInformation_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.event);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ProtectionZoneInformation_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::ProtectionZoneInformation_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::ProtectionZoneInformation_<ContainerAllocator>& v)
  {
    s << indent << "event: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.event);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_PROTECTIONZONEINFORMATION_H