// Generated by gencpp from file kortex_driver/SetTwistAngularSoftLimit.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_SETTWISTANGULARSOFTLIMIT_H
#define KORTEX_DRIVER_MESSAGE_SETTWISTANGULARSOFTLIMIT_H

#include <ros/service_traits.h>


#include <kortex_driver/SetTwistAngularSoftLimitRequest.h>
#include <kortex_driver/SetTwistAngularSoftLimitResponse.h>


namespace kortex_driver
{

struct SetTwistAngularSoftLimit
{

typedef SetTwistAngularSoftLimitRequest Request;
typedef SetTwistAngularSoftLimitResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SetTwistAngularSoftLimit
} // namespace kortex_driver


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::kortex_driver::SetTwistAngularSoftLimit > {
  static const char* value()
  {
    return "3b523deda9069339b963e073a605096e";
  }

  static const char* value(const ::kortex_driver::SetTwistAngularSoftLimit&) { return value(); }
};

template<>
struct DataType< ::kortex_driver::SetTwistAngularSoftLimit > {
  static const char* value()
  {
    return "kortex_driver/SetTwistAngularSoftLimit";
  }

  static const char* value(const ::kortex_driver::SetTwistAngularSoftLimit&) { return value(); }
};


// service_traits::MD5Sum< ::kortex_driver::SetTwistAngularSoftLimitRequest> should match
// service_traits::MD5Sum< ::kortex_driver::SetTwistAngularSoftLimit >
template<>
struct MD5Sum< ::kortex_driver::SetTwistAngularSoftLimitRequest>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::SetTwistAngularSoftLimit >::value();
  }
  static const char* value(const ::kortex_driver::SetTwistAngularSoftLimitRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::SetTwistAngularSoftLimitRequest> should match
// service_traits::DataType< ::kortex_driver::SetTwistAngularSoftLimit >
template<>
struct DataType< ::kortex_driver::SetTwistAngularSoftLimitRequest>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::SetTwistAngularSoftLimit >::value();
  }
  static const char* value(const ::kortex_driver::SetTwistAngularSoftLimitRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::kortex_driver::SetTwistAngularSoftLimitResponse> should match
// service_traits::MD5Sum< ::kortex_driver::SetTwistAngularSoftLimit >
template<>
struct MD5Sum< ::kortex_driver::SetTwistAngularSoftLimitResponse>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::SetTwistAngularSoftLimit >::value();
  }
  static const char* value(const ::kortex_driver::SetTwistAngularSoftLimitResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::SetTwistAngularSoftLimitResponse> should match
// service_traits::DataType< ::kortex_driver::SetTwistAngularSoftLimit >
template<>
struct DataType< ::kortex_driver::SetTwistAngularSoftLimitResponse>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::SetTwistAngularSoftLimit >::value();
  }
  static const char* value(const ::kortex_driver::SetTwistAngularSoftLimitResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_SETTWISTANGULARSOFTLIMIT_H