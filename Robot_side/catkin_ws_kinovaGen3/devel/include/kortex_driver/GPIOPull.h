// Generated by gencpp from file kortex_driver/GPIOPull.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_GPIOPULL_H
#define KORTEX_DRIVER_MESSAGE_GPIOPULL_H


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
struct GPIOPull_
{
  typedef GPIOPull_<ContainerAllocator> Type;

  GPIOPull_()
    {
    }
  GPIOPull_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }





// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(GPIO_PULL_UNSPECIFIED)
  #undef GPIO_PULL_UNSPECIFIED
#endif
#if defined(_WIN32) && defined(GPIO_PULL_NONE)
  #undef GPIO_PULL_NONE
#endif
#if defined(_WIN32) && defined(GPIO_PULL_UP)
  #undef GPIO_PULL_UP
#endif
#if defined(_WIN32) && defined(GPIO_PULL_DOWN)
  #undef GPIO_PULL_DOWN
#endif

  enum {
    GPIO_PULL_UNSPECIFIED = 0u,
    GPIO_PULL_NONE = 1u,
    GPIO_PULL_UP = 2u,
    GPIO_PULL_DOWN = 3u,
  };


  typedef boost::shared_ptr< ::kortex_driver::GPIOPull_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::GPIOPull_<ContainerAllocator> const> ConstPtr;

}; // struct GPIOPull_

typedef ::kortex_driver::GPIOPull_<std::allocator<void> > GPIOPull;

typedef boost::shared_ptr< ::kortex_driver::GPIOPull > GPIOPullPtr;
typedef boost::shared_ptr< ::kortex_driver::GPIOPull const> GPIOPullConstPtr;

// constants requiring out of line definition

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::GPIOPull_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::GPIOPull_<ContainerAllocator> >::stream(s, "", v);
return s;
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::GPIOPull_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::GPIOPull_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::GPIOPull_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::GPIOPull_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::GPIOPull_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::GPIOPull_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::GPIOPull_<ContainerAllocator> >
{
  static const char* value()
  {
    return "76d74f4a9f22d9530bd83becdc56fba5";
  }

  static const char* value(const ::kortex_driver::GPIOPull_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x76d74f4a9f22d953ULL;
  static const uint64_t static_value2 = 0x0bd83becdc56fba5ULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::GPIOPull_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/GPIOPull";
  }

  static const char* value(const ::kortex_driver::GPIOPull_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::GPIOPull_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"uint32 GPIO_PULL_UNSPECIFIED = 0\n"
"\n"
"uint32 GPIO_PULL_NONE = 1\n"
"\n"
"uint32 GPIO_PULL_UP = 2\n"
"\n"
"uint32 GPIO_PULL_DOWN = 3\n"
;
  }

  static const char* value(const ::kortex_driver::GPIOPull_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::GPIOPull_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GPIOPull_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::GPIOPull_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::kortex_driver::GPIOPull_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_GPIOPULL_H
