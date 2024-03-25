// Generated by gencpp from file kortex_driver/GetWifiEnableState.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_GETWIFIENABLESTATE_H
#define KORTEX_DRIVER_MESSAGE_GETWIFIENABLESTATE_H

#include <ros/service_traits.h>


#include <kortex_driver/GetWifiEnableStateRequest.h>
#include <kortex_driver/GetWifiEnableStateResponse.h>


namespace kortex_driver
{

struct GetWifiEnableState
{

typedef GetWifiEnableStateRequest Request;
typedef GetWifiEnableStateResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct GetWifiEnableState
} // namespace kortex_driver


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::kortex_driver::GetWifiEnableState > {
  static const char* value()
  {
    return "9747040002a13b23ba7503e4b2f380fb";
  }

  static const char* value(const ::kortex_driver::GetWifiEnableState&) { return value(); }
};

template<>
struct DataType< ::kortex_driver::GetWifiEnableState > {
  static const char* value()
  {
    return "kortex_driver/GetWifiEnableState";
  }

  static const char* value(const ::kortex_driver::GetWifiEnableState&) { return value(); }
};


// service_traits::MD5Sum< ::kortex_driver::GetWifiEnableStateRequest> should match
// service_traits::MD5Sum< ::kortex_driver::GetWifiEnableState >
template<>
struct MD5Sum< ::kortex_driver::GetWifiEnableStateRequest>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::GetWifiEnableState >::value();
  }
  static const char* value(const ::kortex_driver::GetWifiEnableStateRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::GetWifiEnableStateRequest> should match
// service_traits::DataType< ::kortex_driver::GetWifiEnableState >
template<>
struct DataType< ::kortex_driver::GetWifiEnableStateRequest>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::GetWifiEnableState >::value();
  }
  static const char* value(const ::kortex_driver::GetWifiEnableStateRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::kortex_driver::GetWifiEnableStateResponse> should match
// service_traits::MD5Sum< ::kortex_driver::GetWifiEnableState >
template<>
struct MD5Sum< ::kortex_driver::GetWifiEnableStateResponse>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::GetWifiEnableState >::value();
  }
  static const char* value(const ::kortex_driver::GetWifiEnableStateResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::GetWifiEnableStateResponse> should match
// service_traits::DataType< ::kortex_driver::GetWifiEnableState >
template<>
struct DataType< ::kortex_driver::GetWifiEnableStateResponse>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::GetWifiEnableState >::value();
  }
  static const char* value(const ::kortex_driver::GetWifiEnableStateResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_GETWIFIENABLESTATE_H
