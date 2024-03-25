// Generated by gencpp from file kortex_driver/ControllerConfiguration.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_CONTROLLERCONFIGURATION_H
#define KORTEX_DRIVER_MESSAGE_CONTROLLERCONFIGURATION_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <kortex_driver/ControllerHandle.h>
#include <kortex_driver/MappingHandle.h>

namespace kortex_driver
{
template <class ContainerAllocator>
struct ControllerConfiguration_
{
  typedef ControllerConfiguration_<ContainerAllocator> Type;

  ControllerConfiguration_()
    : handle()
    , name()
    , active_mapping_handle()
    , analog_input_identifier_enum()
    , digital_input_identifier_enum()  {
    }
  ControllerConfiguration_(const ContainerAllocator& _alloc)
    : handle(_alloc)
    , name(_alloc)
    , active_mapping_handle(_alloc)
    , analog_input_identifier_enum(_alloc)
    , digital_input_identifier_enum(_alloc)  {
  (void)_alloc;
    }



   typedef  ::kortex_driver::ControllerHandle_<ContainerAllocator>  _handle_type;
  _handle_type handle;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _name_type;
  _name_type name;

   typedef  ::kortex_driver::MappingHandle_<ContainerAllocator>  _active_mapping_handle_type;
  _active_mapping_handle_type active_mapping_handle;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _analog_input_identifier_enum_type;
  _analog_input_identifier_enum_type analog_input_identifier_enum;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _digital_input_identifier_enum_type;
  _digital_input_identifier_enum_type digital_input_identifier_enum;





  typedef boost::shared_ptr< ::kortex_driver::ControllerConfiguration_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::ControllerConfiguration_<ContainerAllocator> const> ConstPtr;

}; // struct ControllerConfiguration_

typedef ::kortex_driver::ControllerConfiguration_<std::allocator<void> > ControllerConfiguration;

typedef boost::shared_ptr< ::kortex_driver::ControllerConfiguration > ControllerConfigurationPtr;
typedef boost::shared_ptr< ::kortex_driver::ControllerConfiguration const> ControllerConfigurationConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::ControllerConfiguration_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::ControllerConfiguration_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::ControllerConfiguration_<ContainerAllocator1> & lhs, const ::kortex_driver::ControllerConfiguration_<ContainerAllocator2> & rhs)
{
  return lhs.handle == rhs.handle &&
    lhs.name == rhs.name &&
    lhs.active_mapping_handle == rhs.active_mapping_handle &&
    lhs.analog_input_identifier_enum == rhs.analog_input_identifier_enum &&
    lhs.digital_input_identifier_enum == rhs.digital_input_identifier_enum;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::ControllerConfiguration_<ContainerAllocator1> & lhs, const ::kortex_driver::ControllerConfiguration_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::ControllerConfiguration_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::ControllerConfiguration_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::ControllerConfiguration_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::ControllerConfiguration_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::ControllerConfiguration_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::ControllerConfiguration_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::ControllerConfiguration_<ContainerAllocator> >
{
  static const char* value()
  {
    return "0a2e41d50fc9c491b9a0c9000a90ca85";
  }

  static const char* value(const ::kortex_driver::ControllerConfiguration_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x0a2e41d50fc9c491ULL;
  static const uint64_t static_value2 = 0xb9a0c9000a90ca85ULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::ControllerConfiguration_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/ControllerConfiguration";
  }

  static const char* value(const ::kortex_driver::ControllerConfiguration_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::ControllerConfiguration_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"ControllerHandle handle\n"
"string name\n"
"MappingHandle active_mapping_handle\n"
"string analog_input_identifier_enum\n"
"string digital_input_identifier_enum\n"
"================================================================================\n"
"MSG: kortex_driver/ControllerHandle\n"
"\n"
"uint32 type\n"
"uint32 controller_identifier\n"
"================================================================================\n"
"MSG: kortex_driver/MappingHandle\n"
"\n"
"uint32 identifier\n"
"uint32 permission\n"
;
  }

  static const char* value(const ::kortex_driver::ControllerConfiguration_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::ControllerConfiguration_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.handle);
      stream.next(m.name);
      stream.next(m.active_mapping_handle);
      stream.next(m.analog_input_identifier_enum);
      stream.next(m.digital_input_identifier_enum);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ControllerConfiguration_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::ControllerConfiguration_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::ControllerConfiguration_<ContainerAllocator>& v)
  {
    s << indent << "handle: ";
    s << std::endl;
    Printer< ::kortex_driver::ControllerHandle_<ContainerAllocator> >::stream(s, indent + "  ", v.handle);
    s << indent << "name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.name);
    s << indent << "active_mapping_handle: ";
    s << std::endl;
    Printer< ::kortex_driver::MappingHandle_<ContainerAllocator> >::stream(s, indent + "  ", v.active_mapping_handle);
    s << indent << "analog_input_identifier_enum: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.analog_input_identifier_enum);
    s << indent << "digital_input_identifier_enum: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.digital_input_identifier_enum);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_CONTROLLERCONFIGURATION_H
