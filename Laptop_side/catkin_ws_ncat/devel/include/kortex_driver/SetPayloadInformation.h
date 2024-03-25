// Generated by gencpp from file kortex_driver/SetPayloadInformation.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_SETPAYLOADINFORMATION_H
#define KORTEX_DRIVER_MESSAGE_SETPAYLOADINFORMATION_H

#include <ros/service_traits.h>


#include <kortex_driver/SetPayloadInformationRequest.h>
#include <kortex_driver/SetPayloadInformationResponse.h>


namespace kortex_driver
{

struct SetPayloadInformation
{

typedef SetPayloadInformationRequest Request;
typedef SetPayloadInformationResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SetPayloadInformation
} // namespace kortex_driver


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::kortex_driver::SetPayloadInformation > {
  static const char* value()
  {
    return "132bdc38d85ff3aec89e76f693ad5e54";
  }

  static const char* value(const ::kortex_driver::SetPayloadInformation&) { return value(); }
};

template<>
struct DataType< ::kortex_driver::SetPayloadInformation > {
  static const char* value()
  {
    return "kortex_driver/SetPayloadInformation";
  }

  static const char* value(const ::kortex_driver::SetPayloadInformation&) { return value(); }
};


// service_traits::MD5Sum< ::kortex_driver::SetPayloadInformationRequest> should match
// service_traits::MD5Sum< ::kortex_driver::SetPayloadInformation >
template<>
struct MD5Sum< ::kortex_driver::SetPayloadInformationRequest>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::SetPayloadInformation >::value();
  }
  static const char* value(const ::kortex_driver::SetPayloadInformationRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::SetPayloadInformationRequest> should match
// service_traits::DataType< ::kortex_driver::SetPayloadInformation >
template<>
struct DataType< ::kortex_driver::SetPayloadInformationRequest>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::SetPayloadInformation >::value();
  }
  static const char* value(const ::kortex_driver::SetPayloadInformationRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::kortex_driver::SetPayloadInformationResponse> should match
// service_traits::MD5Sum< ::kortex_driver::SetPayloadInformation >
template<>
struct MD5Sum< ::kortex_driver::SetPayloadInformationResponse>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::SetPayloadInformation >::value();
  }
  static const char* value(const ::kortex_driver::SetPayloadInformationResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::SetPayloadInformationResponse> should match
// service_traits::DataType< ::kortex_driver::SetPayloadInformation >
template<>
struct DataType< ::kortex_driver::SetPayloadInformationResponse>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::SetPayloadInformation >::value();
  }
  static const char* value(const ::kortex_driver::SetPayloadInformationResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_SETPAYLOADINFORMATION_H