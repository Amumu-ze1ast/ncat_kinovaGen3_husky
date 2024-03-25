// Generated by gencpp from file kortex_driver/UpdateProtectionZone.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_UPDATEPROTECTIONZONE_H
#define KORTEX_DRIVER_MESSAGE_UPDATEPROTECTIONZONE_H

#include <ros/service_traits.h>


#include <kortex_driver/UpdateProtectionZoneRequest.h>
#include <kortex_driver/UpdateProtectionZoneResponse.h>


namespace kortex_driver
{

struct UpdateProtectionZone
{

typedef UpdateProtectionZoneRequest Request;
typedef UpdateProtectionZoneResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct UpdateProtectionZone
} // namespace kortex_driver


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::kortex_driver::UpdateProtectionZone > {
  static const char* value()
  {
    return "b1c7755249865d381b4b70557a125e12";
  }

  static const char* value(const ::kortex_driver::UpdateProtectionZone&) { return value(); }
};

template<>
struct DataType< ::kortex_driver::UpdateProtectionZone > {
  static const char* value()
  {
    return "kortex_driver/UpdateProtectionZone";
  }

  static const char* value(const ::kortex_driver::UpdateProtectionZone&) { return value(); }
};


// service_traits::MD5Sum< ::kortex_driver::UpdateProtectionZoneRequest> should match
// service_traits::MD5Sum< ::kortex_driver::UpdateProtectionZone >
template<>
struct MD5Sum< ::kortex_driver::UpdateProtectionZoneRequest>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::UpdateProtectionZone >::value();
  }
  static const char* value(const ::kortex_driver::UpdateProtectionZoneRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::UpdateProtectionZoneRequest> should match
// service_traits::DataType< ::kortex_driver::UpdateProtectionZone >
template<>
struct DataType< ::kortex_driver::UpdateProtectionZoneRequest>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::UpdateProtectionZone >::value();
  }
  static const char* value(const ::kortex_driver::UpdateProtectionZoneRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::kortex_driver::UpdateProtectionZoneResponse> should match
// service_traits::MD5Sum< ::kortex_driver::UpdateProtectionZone >
template<>
struct MD5Sum< ::kortex_driver::UpdateProtectionZoneResponse>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::UpdateProtectionZone >::value();
  }
  static const char* value(const ::kortex_driver::UpdateProtectionZoneResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::UpdateProtectionZoneResponse> should match
// service_traits::DataType< ::kortex_driver::UpdateProtectionZone >
template<>
struct DataType< ::kortex_driver::UpdateProtectionZoneResponse>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::UpdateProtectionZone >::value();
  }
  static const char* value(const ::kortex_driver::UpdateProtectionZoneResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_UPDATEPROTECTIONZONE_H
