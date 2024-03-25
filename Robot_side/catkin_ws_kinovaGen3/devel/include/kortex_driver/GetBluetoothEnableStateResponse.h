// Generated by gencpp from file kortex_driver/GetBluetoothEnableStateResponse.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_GETBLUETOOTHENABLESTATERESPONSE_H
#define KORTEX_DRIVER_MESSAGE_GETBLUETOOTHENABLESTATERESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <kortex_driver/BluetoothEnableState.h>

namespace kortex_driver
{
template <class ContainerAllocator>
struct GetBluetoothEnableStateResponse_
{
  typedef GetBluetoothEnableStateResponse_<ContainerAllocator> Type;

  GetBluetoothEnableStateResponse_()
    : output()  {
    }
  GetBluetoothEnableStateResponse_(const ContainerAllocator& _alloc)
    : output(_alloc)  {
  (void)_alloc;
    }



   typedef  ::kortex_driver::BluetoothEnableState_<ContainerAllocator>  _output_type;
  _output_type output;





  typedef boost::shared_ptr< ::kortex_driver::GetBluetoothEnableStateResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::GetBluetoothEnableStateResponse_<ContainerAllocator> const> ConstPtr;

}; // struct GetBluetoothEnableStateResponse_

typedef ::kortex_driver::GetBluetoothEnableStateResponse_<std::allocator<void> > GetBluetoothEnableStateResponse;

typedef boost::shared_ptr< ::kortex_driver::GetBluetoothEnableStateResponse > GetBluetoothEnableStateResponsePtr;
typedef boost::shared_ptr< ::kortex_driver::GetBluetoothEnableStateResponse const> GetBluetoothEnableStateResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::GetBluetoothEnableStateResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::GetBluetoothEnableStateResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::GetBluetoothEnableStateResponse_<ContainerAllocator1> & lhs, const ::kortex_driver::GetBluetoothEnableStateResponse_<ContainerAllocator2> & rhs)
{
  return lhs.output == rhs.output;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::GetBluetoothEnableStateResponse_<ContainerAllocator1> & lhs, const ::kortex_driver::GetBluetoothEnableStateResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::GetBluetoothEnableStateResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::GetBluetoothEnableStateResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::GetBluetoothEnableStateResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::GetBluetoothEnableStateResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::GetBluetoothEnableStateResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::GetBluetoothEnableStateResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::GetBluetoothEnableStateResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "9bbefebbfb9963ee8c08183f71f2ccc5";
  }

  static const char* value(const ::kortex_driver::GetBluetoothEnableStateResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x9bbefebbfb9963eeULL;
  static const uint64_t static_value2 = 0x8c08183f71f2ccc5ULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::GetBluetoothEnableStateResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/GetBluetoothEnableStateResponse";
  }

  static const char* value(const ::kortex_driver::GetBluetoothEnableStateResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::GetBluetoothEnableStateResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "BluetoothEnableState output\n"
"\n"
"================================================================================\n"
"MSG: kortex_driver/BluetoothEnableState\n"
"\n"
"bool enabled\n"
;
  }

  static const char* value(const ::kortex_driver::GetBluetoothEnableStateResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::GetBluetoothEnableStateResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.output);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetBluetoothEnableStateResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::GetBluetoothEnableStateResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::GetBluetoothEnableStateResponse_<ContainerAllocator>& v)
  {
    s << indent << "output: ";
    s << std::endl;
    Printer< ::kortex_driver::BluetoothEnableState_<ContainerAllocator> >::stream(s, indent + "  ", v.output);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_GETBLUETOOTHENABLESTATERESPONSE_H
