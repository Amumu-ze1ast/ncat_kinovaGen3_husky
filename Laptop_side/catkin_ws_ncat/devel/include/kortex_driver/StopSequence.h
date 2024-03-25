// Generated by gencpp from file kortex_driver/StopSequence.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_STOPSEQUENCE_H
#define KORTEX_DRIVER_MESSAGE_STOPSEQUENCE_H

#include <ros/service_traits.h>


#include <kortex_driver/StopSequenceRequest.h>
#include <kortex_driver/StopSequenceResponse.h>


namespace kortex_driver
{

struct StopSequence
{

typedef StopSequenceRequest Request;
typedef StopSequenceResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct StopSequence
} // namespace kortex_driver


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::kortex_driver::StopSequence > {
  static const char* value()
  {
    return "f335b819dc59099fe3124b36f140ad07";
  }

  static const char* value(const ::kortex_driver::StopSequence&) { return value(); }
};

template<>
struct DataType< ::kortex_driver::StopSequence > {
  static const char* value()
  {
    return "kortex_driver/StopSequence";
  }

  static const char* value(const ::kortex_driver::StopSequence&) { return value(); }
};


// service_traits::MD5Sum< ::kortex_driver::StopSequenceRequest> should match
// service_traits::MD5Sum< ::kortex_driver::StopSequence >
template<>
struct MD5Sum< ::kortex_driver::StopSequenceRequest>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::StopSequence >::value();
  }
  static const char* value(const ::kortex_driver::StopSequenceRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::StopSequenceRequest> should match
// service_traits::DataType< ::kortex_driver::StopSequence >
template<>
struct DataType< ::kortex_driver::StopSequenceRequest>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::StopSequence >::value();
  }
  static const char* value(const ::kortex_driver::StopSequenceRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::kortex_driver::StopSequenceResponse> should match
// service_traits::MD5Sum< ::kortex_driver::StopSequence >
template<>
struct MD5Sum< ::kortex_driver::StopSequenceResponse>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::StopSequence >::value();
  }
  static const char* value(const ::kortex_driver::StopSequenceResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::StopSequenceResponse> should match
// service_traits::DataType< ::kortex_driver::StopSequence >
template<>
struct DataType< ::kortex_driver::StopSequenceResponse>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::StopSequence >::value();
  }
  static const char* value(const ::kortex_driver::StopSequenceResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_STOPSEQUENCE_H