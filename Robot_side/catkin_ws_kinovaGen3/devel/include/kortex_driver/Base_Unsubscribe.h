// Generated by gencpp from file kortex_driver/Base_Unsubscribe.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_BASE_UNSUBSCRIBE_H
#define KORTEX_DRIVER_MESSAGE_BASE_UNSUBSCRIBE_H

#include <ros/service_traits.h>


#include <kortex_driver/Base_UnsubscribeRequest.h>
#include <kortex_driver/Base_UnsubscribeResponse.h>


namespace kortex_driver
{

struct Base_Unsubscribe
{

typedef Base_UnsubscribeRequest Request;
typedef Base_UnsubscribeResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct Base_Unsubscribe
} // namespace kortex_driver


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::kortex_driver::Base_Unsubscribe > {
  static const char* value()
  {
    return "7960dab80443332660b3bb4bf774c9ab";
  }

  static const char* value(const ::kortex_driver::Base_Unsubscribe&) { return value(); }
};

template<>
struct DataType< ::kortex_driver::Base_Unsubscribe > {
  static const char* value()
  {
    return "kortex_driver/Base_Unsubscribe";
  }

  static const char* value(const ::kortex_driver::Base_Unsubscribe&) { return value(); }
};


// service_traits::MD5Sum< ::kortex_driver::Base_UnsubscribeRequest> should match
// service_traits::MD5Sum< ::kortex_driver::Base_Unsubscribe >
template<>
struct MD5Sum< ::kortex_driver::Base_UnsubscribeRequest>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::Base_Unsubscribe >::value();
  }
  static const char* value(const ::kortex_driver::Base_UnsubscribeRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::Base_UnsubscribeRequest> should match
// service_traits::DataType< ::kortex_driver::Base_Unsubscribe >
template<>
struct DataType< ::kortex_driver::Base_UnsubscribeRequest>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::Base_Unsubscribe >::value();
  }
  static const char* value(const ::kortex_driver::Base_UnsubscribeRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::kortex_driver::Base_UnsubscribeResponse> should match
// service_traits::MD5Sum< ::kortex_driver::Base_Unsubscribe >
template<>
struct MD5Sum< ::kortex_driver::Base_UnsubscribeResponse>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::Base_Unsubscribe >::value();
  }
  static const char* value(const ::kortex_driver::Base_UnsubscribeResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::Base_UnsubscribeResponse> should match
// service_traits::DataType< ::kortex_driver::Base_Unsubscribe >
template<>
struct DataType< ::kortex_driver::Base_UnsubscribeResponse>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::Base_Unsubscribe >::value();
  }
  static const char* value(const ::kortex_driver::Base_UnsubscribeResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_BASE_UNSUBSCRIBE_H