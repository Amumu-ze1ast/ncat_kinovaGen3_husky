// Generated by gencpp from file kortex_driver/GetWifiInformation.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_GETWIFIINFORMATION_H
#define KORTEX_DRIVER_MESSAGE_GETWIFIINFORMATION_H

#include <ros/service_traits.h>


#include <kortex_driver/GetWifiInformationRequest.h>
#include <kortex_driver/GetWifiInformationResponse.h>


namespace kortex_driver
{

struct GetWifiInformation
{

typedef GetWifiInformationRequest Request;
typedef GetWifiInformationResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct GetWifiInformation
} // namespace kortex_driver


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::kortex_driver::GetWifiInformation > {
  static const char* value()
  {
    return "54ddc4afd24b3f5a522f79c66970e8fc";
  }

  static const char* value(const ::kortex_driver::GetWifiInformation&) { return value(); }
};

template<>
struct DataType< ::kortex_driver::GetWifiInformation > {
  static const char* value()
  {
    return "kortex_driver/GetWifiInformation";
  }

  static const char* value(const ::kortex_driver::GetWifiInformation&) { return value(); }
};


// service_traits::MD5Sum< ::kortex_driver::GetWifiInformationRequest> should match
// service_traits::MD5Sum< ::kortex_driver::GetWifiInformation >
template<>
struct MD5Sum< ::kortex_driver::GetWifiInformationRequest>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::GetWifiInformation >::value();
  }
  static const char* value(const ::kortex_driver::GetWifiInformationRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::GetWifiInformationRequest> should match
// service_traits::DataType< ::kortex_driver::GetWifiInformation >
template<>
struct DataType< ::kortex_driver::GetWifiInformationRequest>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::GetWifiInformation >::value();
  }
  static const char* value(const ::kortex_driver::GetWifiInformationRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::kortex_driver::GetWifiInformationResponse> should match
// service_traits::MD5Sum< ::kortex_driver::GetWifiInformation >
template<>
struct MD5Sum< ::kortex_driver::GetWifiInformationResponse>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::GetWifiInformation >::value();
  }
  static const char* value(const ::kortex_driver::GetWifiInformationResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::GetWifiInformationResponse> should match
// service_traits::DataType< ::kortex_driver::GetWifiInformation >
template<>
struct DataType< ::kortex_driver::GetWifiInformationResponse>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::GetWifiInformation >::value();
  }
  static const char* value(const ::kortex_driver::GetWifiInformationResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_GETWIFIINFORMATION_H
