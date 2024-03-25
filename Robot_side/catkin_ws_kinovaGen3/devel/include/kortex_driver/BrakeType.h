// Generated by gencpp from file kortex_driver/BrakeType.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_BRAKETYPE_H
#define KORTEX_DRIVER_MESSAGE_BRAKETYPE_H


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
struct BrakeType_
{
  typedef BrakeType_<ContainerAllocator> Type;

  BrakeType_()
    {
    }
  BrakeType_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }





// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(BRAKE_TYPE_UNSPECIFIED)
  #undef BRAKE_TYPE_UNSPECIFIED
#endif
#if defined(_WIN32) && defined(BRAKE_TYPE_NOT_INSTALLED)
  #undef BRAKE_TYPE_NOT_INSTALLED
#endif
#if defined(_WIN32) && defined(BRAKE_TYPE_SPOKE)
  #undef BRAKE_TYPE_SPOKE
#endif
#if defined(_WIN32) && defined(BRAKE_TYPE_CLUTCH)
  #undef BRAKE_TYPE_CLUTCH
#endif

  enum {
    BRAKE_TYPE_UNSPECIFIED = 0u,
    BRAKE_TYPE_NOT_INSTALLED = 1u,
    BRAKE_TYPE_SPOKE = 2u,
    BRAKE_TYPE_CLUTCH = 3u,
  };


  typedef boost::shared_ptr< ::kortex_driver::BrakeType_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::BrakeType_<ContainerAllocator> const> ConstPtr;

}; // struct BrakeType_

typedef ::kortex_driver::BrakeType_<std::allocator<void> > BrakeType;

typedef boost::shared_ptr< ::kortex_driver::BrakeType > BrakeTypePtr;
typedef boost::shared_ptr< ::kortex_driver::BrakeType const> BrakeTypeConstPtr;

// constants requiring out of line definition

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::BrakeType_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::BrakeType_<ContainerAllocator> >::stream(s, "", v);
return s;
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::BrakeType_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::BrakeType_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::BrakeType_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::BrakeType_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::BrakeType_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::BrakeType_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::BrakeType_<ContainerAllocator> >
{
  static const char* value()
  {
    return "856901ae6854740e9adfa01cae483501";
  }

  static const char* value(const ::kortex_driver::BrakeType_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x856901ae6854740eULL;
  static const uint64_t static_value2 = 0x9adfa01cae483501ULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::BrakeType_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/BrakeType";
  }

  static const char* value(const ::kortex_driver::BrakeType_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::BrakeType_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"uint32 BRAKE_TYPE_UNSPECIFIED = 0\n"
"\n"
"uint32 BRAKE_TYPE_NOT_INSTALLED = 1\n"
"\n"
"uint32 BRAKE_TYPE_SPOKE = 2\n"
"\n"
"uint32 BRAKE_TYPE_CLUTCH = 3\n"
;
  }

  static const char* value(const ::kortex_driver::BrakeType_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::BrakeType_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct BrakeType_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::BrakeType_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::kortex_driver::BrakeType_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_BRAKETYPE_H