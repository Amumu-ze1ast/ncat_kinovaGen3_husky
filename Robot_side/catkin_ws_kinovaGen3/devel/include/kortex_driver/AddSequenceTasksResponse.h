// Generated by gencpp from file kortex_driver/AddSequenceTasksResponse.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_ADDSEQUENCETASKSRESPONSE_H
#define KORTEX_DRIVER_MESSAGE_ADDSEQUENCETASKSRESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <kortex_driver/SequenceTasksRange.h>

namespace kortex_driver
{
template <class ContainerAllocator>
struct AddSequenceTasksResponse_
{
  typedef AddSequenceTasksResponse_<ContainerAllocator> Type;

  AddSequenceTasksResponse_()
    : output()  {
    }
  AddSequenceTasksResponse_(const ContainerAllocator& _alloc)
    : output(_alloc)  {
  (void)_alloc;
    }



   typedef  ::kortex_driver::SequenceTasksRange_<ContainerAllocator>  _output_type;
  _output_type output;





  typedef boost::shared_ptr< ::kortex_driver::AddSequenceTasksResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::AddSequenceTasksResponse_<ContainerAllocator> const> ConstPtr;

}; // struct AddSequenceTasksResponse_

typedef ::kortex_driver::AddSequenceTasksResponse_<std::allocator<void> > AddSequenceTasksResponse;

typedef boost::shared_ptr< ::kortex_driver::AddSequenceTasksResponse > AddSequenceTasksResponsePtr;
typedef boost::shared_ptr< ::kortex_driver::AddSequenceTasksResponse const> AddSequenceTasksResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::AddSequenceTasksResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::AddSequenceTasksResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::AddSequenceTasksResponse_<ContainerAllocator1> & lhs, const ::kortex_driver::AddSequenceTasksResponse_<ContainerAllocator2> & rhs)
{
  return lhs.output == rhs.output;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::AddSequenceTasksResponse_<ContainerAllocator1> & lhs, const ::kortex_driver::AddSequenceTasksResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::AddSequenceTasksResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::AddSequenceTasksResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::AddSequenceTasksResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::AddSequenceTasksResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::AddSequenceTasksResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::AddSequenceTasksResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::AddSequenceTasksResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "9c02034a64042a4eef7cbc1283167b6a";
  }

  static const char* value(const ::kortex_driver::AddSequenceTasksResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x9c02034a64042a4eULL;
  static const uint64_t static_value2 = 0xef7cbc1283167b6aULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::AddSequenceTasksResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/AddSequenceTasksResponse";
  }

  static const char* value(const ::kortex_driver::AddSequenceTasksResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::AddSequenceTasksResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "SequenceTasksRange output\n"
"\n"
"================================================================================\n"
"MSG: kortex_driver/SequenceTasksRange\n"
"\n"
"uint32 first_task_index\n"
"uint32 second_task_index\n"
;
  }

  static const char* value(const ::kortex_driver::AddSequenceTasksResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::AddSequenceTasksResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.output);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct AddSequenceTasksResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::AddSequenceTasksResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::AddSequenceTasksResponse_<ContainerAllocator>& v)
  {
    s << indent << "output: ";
    s << std::endl;
    Printer< ::kortex_driver::SequenceTasksRange_<ContainerAllocator> >::stream(s, indent + "  ", v.output);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_ADDSEQUENCETASKSRESPONSE_H
