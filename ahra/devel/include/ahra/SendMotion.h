// Generated by gencpp from file ahra/SendMotion.msg
// DO NOT EDIT!


#ifndef AHRA_MESSAGE_SENDMOTION_H
#define AHRA_MESSAGE_SENDMOTION_H

#include <ros/service_traits.h>


#include <ahra/SendMotionRequest.h>
#include <ahra/SendMotionResponse.h>


namespace ahra
{

struct SendMotion
{

typedef SendMotionRequest Request;
typedef SendMotionResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SendMotion
} // namespace ahra


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::ahra::SendMotion > {
  static const char* value()
  {
    return "195b77fcb35ce347682e382fd8b4d70b";
  }

  static const char* value(const ::ahra::SendMotion&) { return value(); }
};

template<>
struct DataType< ::ahra::SendMotion > {
  static const char* value()
  {
    return "ahra/SendMotion";
  }

  static const char* value(const ::ahra::SendMotion&) { return value(); }
};


// service_traits::MD5Sum< ::ahra::SendMotionRequest> should match
// service_traits::MD5Sum< ::ahra::SendMotion >
template<>
struct MD5Sum< ::ahra::SendMotionRequest>
{
  static const char* value()
  {
    return MD5Sum< ::ahra::SendMotion >::value();
  }
  static const char* value(const ::ahra::SendMotionRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::ahra::SendMotionRequest> should match
// service_traits::DataType< ::ahra::SendMotion >
template<>
struct DataType< ::ahra::SendMotionRequest>
{
  static const char* value()
  {
    return DataType< ::ahra::SendMotion >::value();
  }
  static const char* value(const ::ahra::SendMotionRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::ahra::SendMotionResponse> should match
// service_traits::MD5Sum< ::ahra::SendMotion >
template<>
struct MD5Sum< ::ahra::SendMotionResponse>
{
  static const char* value()
  {
    return MD5Sum< ::ahra::SendMotion >::value();
  }
  static const char* value(const ::ahra::SendMotionResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::ahra::SendMotionResponse> should match
// service_traits::DataType< ::ahra::SendMotion >
template<>
struct DataType< ::ahra::SendMotionResponse>
{
  static const char* value()
  {
    return DataType< ::ahra::SendMotion >::value();
  }
  static const char* value(const ::ahra::SendMotionResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // AHRA_MESSAGE_SENDMOTION_H
