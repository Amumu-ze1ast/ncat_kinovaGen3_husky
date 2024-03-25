// Generated by gencpp from file kortex_driver/Base_ControlModeNotification.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_BASE_CONTROLMODENOTIFICATION_H
#define KORTEX_DRIVER_MESSAGE_BASE_CONTROLMODENOTIFICATION_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <kortex_driver/Timestamp.h>
#include <kortex_driver/UserProfileHandle.h>
#include <kortex_driver/Connection.h>

namespace kortex_driver
{
template <class ContainerAllocator>
struct Base_ControlModeNotification_
{
  typedef Base_ControlModeNotification_<ContainerAllocator> Type;

  Base_ControlModeNotification_()
    : control_mode(0)
    , timestamp()
    , user_handle()
    , connection()  {
    }
  Base_ControlModeNotification_(const ContainerAllocator& _alloc)
    : control_mode(0)
    , timestamp(_alloc)
    , user_handle(_alloc)
    , connection(_alloc)  {
  (void)_alloc;
    }



   typedef uint32_t _control_mode_type;
  _control_mode_type control_mode;

   typedef  ::kortex_driver::Timestamp_<ContainerAllocator>  _timestamp_type;
  _timestamp_type timestamp;

   typedef  ::kortex_driver::UserProfileHandle_<ContainerAllocator>  _user_handle_type;
  _user_handle_type user_handle;

   typedef  ::kortex_driver::Connection_<ContainerAllocator>  _connection_type;
  _connection_type connection;





  typedef boost::shared_ptr< ::kortex_driver::Base_ControlModeNotification_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::Base_ControlModeNotification_<ContainerAllocator> const> ConstPtr;

}; // struct Base_ControlModeNotification_

typedef ::kortex_driver::Base_ControlModeNotification_<std::allocator<void> > Base_ControlModeNotification;

typedef boost::shared_ptr< ::kortex_driver::Base_ControlModeNotification > Base_ControlModeNotificationPtr;
typedef boost::shared_ptr< ::kortex_driver::Base_ControlModeNotification const> Base_ControlModeNotificationConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::Base_ControlModeNotification_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::Base_ControlModeNotification_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::Base_ControlModeNotification_<ContainerAllocator1> & lhs, const ::kortex_driver::Base_ControlModeNotification_<ContainerAllocator2> & rhs)
{
  return lhs.control_mode == rhs.control_mode &&
    lhs.timestamp == rhs.timestamp &&
    lhs.user_handle == rhs.user_handle &&
    lhs.connection == rhs.connection;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::Base_ControlModeNotification_<ContainerAllocator1> & lhs, const ::kortex_driver::Base_ControlModeNotification_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::Base_ControlModeNotification_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::Base_ControlModeNotification_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::Base_ControlModeNotification_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::Base_ControlModeNotification_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::Base_ControlModeNotification_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::Base_ControlModeNotification_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::Base_ControlModeNotification_<ContainerAllocator> >
{
  static const char* value()
  {
    return "de6c0474dc1823fff14e6223b9e6b591";
  }

  static const char* value(const ::kortex_driver::Base_ControlModeNotification_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xde6c0474dc1823ffULL;
  static const uint64_t static_value2 = 0xf14e6223b9e6b591ULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::Base_ControlModeNotification_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/Base_ControlModeNotification";
  }

  static const char* value(const ::kortex_driver::Base_ControlModeNotification_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::Base_ControlModeNotification_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"uint32 control_mode\n"
"Timestamp timestamp\n"
"UserProfileHandle user_handle\n"
"Connection connection\n"
"================================================================================\n"
"MSG: kortex_driver/Timestamp\n"
"\n"
"uint32 sec\n"
"uint32 usec\n"
"================================================================================\n"
"MSG: kortex_driver/UserProfileHandle\n"
"\n"
"uint32 identifier\n"
"uint32 permission\n"
"================================================================================\n"
"MSG: kortex_driver/Connection\n"
"\n"
"UserProfileHandle user_handle\n"
"string connection_information\n"
"uint32 connection_identifier\n"
;
  }

  static const char* value(const ::kortex_driver::Base_ControlModeNotification_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::Base_ControlModeNotification_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.control_mode);
      stream.next(m.timestamp);
      stream.next(m.user_handle);
      stream.next(m.connection);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Base_ControlModeNotification_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::Base_ControlModeNotification_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::Base_ControlModeNotification_<ContainerAllocator>& v)
  {
    s << indent << "control_mode: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.control_mode);
    s << indent << "timestamp: ";
    s << std::endl;
    Printer< ::kortex_driver::Timestamp_<ContainerAllocator> >::stream(s, indent + "  ", v.timestamp);
    s << indent << "user_handle: ";
    s << std::endl;
    Printer< ::kortex_driver::UserProfileHandle_<ContainerAllocator> >::stream(s, indent + "  ", v.user_handle);
    s << indent << "connection: ";
    s << std::endl;
    Printer< ::kortex_driver::Connection_<ContainerAllocator> >::stream(s, indent + "  ", v.connection);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_BASE_CONTROLMODENOTIFICATION_H
