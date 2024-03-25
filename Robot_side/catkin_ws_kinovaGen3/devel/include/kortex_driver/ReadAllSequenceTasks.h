// Generated by gencpp from file kortex_driver/ReadAllSequenceTasks.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_READALLSEQUENCETASKS_H
#define KORTEX_DRIVER_MESSAGE_READALLSEQUENCETASKS_H

#include <ros/service_traits.h>


#include <kortex_driver/ReadAllSequenceTasksRequest.h>
#include <kortex_driver/ReadAllSequenceTasksResponse.h>


namespace kortex_driver
{

struct ReadAllSequenceTasks
{

typedef ReadAllSequenceTasksRequest Request;
typedef ReadAllSequenceTasksResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct ReadAllSequenceTasks
} // namespace kortex_driver


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::kortex_driver::ReadAllSequenceTasks > {
  static const char* value()
  {
    return "32ac1eb4a8f3fd9be623e97e0e0fbfa0";
  }

  static const char* value(const ::kortex_driver::ReadAllSequenceTasks&) { return value(); }
};

template<>
struct DataType< ::kortex_driver::ReadAllSequenceTasks > {
  static const char* value()
  {
    return "kortex_driver/ReadAllSequenceTasks";
  }

  static const char* value(const ::kortex_driver::ReadAllSequenceTasks&) { return value(); }
};


// service_traits::MD5Sum< ::kortex_driver::ReadAllSequenceTasksRequest> should match
// service_traits::MD5Sum< ::kortex_driver::ReadAllSequenceTasks >
template<>
struct MD5Sum< ::kortex_driver::ReadAllSequenceTasksRequest>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::ReadAllSequenceTasks >::value();
  }
  static const char* value(const ::kortex_driver::ReadAllSequenceTasksRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::ReadAllSequenceTasksRequest> should match
// service_traits::DataType< ::kortex_driver::ReadAllSequenceTasks >
template<>
struct DataType< ::kortex_driver::ReadAllSequenceTasksRequest>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::ReadAllSequenceTasks >::value();
  }
  static const char* value(const ::kortex_driver::ReadAllSequenceTasksRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::kortex_driver::ReadAllSequenceTasksResponse> should match
// service_traits::MD5Sum< ::kortex_driver::ReadAllSequenceTasks >
template<>
struct MD5Sum< ::kortex_driver::ReadAllSequenceTasksResponse>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::ReadAllSequenceTasks >::value();
  }
  static const char* value(const ::kortex_driver::ReadAllSequenceTasksResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::ReadAllSequenceTasksResponse> should match
// service_traits::DataType< ::kortex_driver::ReadAllSequenceTasks >
template<>
struct DataType< ::kortex_driver::ReadAllSequenceTasksResponse>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::ReadAllSequenceTasks >::value();
  }
  static const char* value(const ::kortex_driver::ReadAllSequenceTasksResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_READALLSEQUENCETASKS_H