// Generated by gencpp from file ahra/UD_NeckAngleRequest.msg
// DO NOT EDIT!


#ifndef AHRA_MESSAGE_UD_NECKANGLEREQUEST_H
#define AHRA_MESSAGE_UD_NECKANGLEREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace ahra
{
template <class ContainerAllocator>
struct UD_NeckAngleRequest_
{
  typedef UD_NeckAngleRequest_<ContainerAllocator> Type;

  UD_NeckAngleRequest_()
    : finish(false)  {
    }
  UD_NeckAngleRequest_(const ContainerAllocator& _alloc)
    : finish(false)  {
  (void)_alloc;
    }



   typedef uint8_t _finish_type;
  _finish_type finish;





  typedef boost::shared_ptr< ::ahra::UD_NeckAngleRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ahra::UD_NeckAngleRequest_<ContainerAllocator> const> ConstPtr;

}; // struct UD_NeckAngleRequest_

typedef ::ahra::UD_NeckAngleRequest_<std::allocator<void> > UD_NeckAngleRequest;

typedef boost::shared_ptr< ::ahra::UD_NeckAngleRequest > UD_NeckAngleRequestPtr;
typedef boost::shared_ptr< ::ahra::UD_NeckAngleRequest const> UD_NeckAngleRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ahra::UD_NeckAngleRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ahra::UD_NeckAngleRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::ahra::UD_NeckAngleRequest_<ContainerAllocator1> & lhs, const ::ahra::UD_NeckAngleRequest_<ContainerAllocator2> & rhs)
{
  return lhs.finish == rhs.finish;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::ahra::UD_NeckAngleRequest_<ContainerAllocator1> & lhs, const ::ahra::UD_NeckAngleRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace ahra

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::ahra::UD_NeckAngleRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ahra::UD_NeckAngleRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ahra::UD_NeckAngleRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ahra::UD_NeckAngleRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ahra::UD_NeckAngleRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ahra::UD_NeckAngleRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ahra::UD_NeckAngleRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "474a58dbb494a45bb1ca99544cd64e45";
  }

  static const char* value(const ::ahra::UD_NeckAngleRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x474a58dbb494a45bULL;
  static const uint64_t static_value2 = 0xb1ca99544cd64e45ULL;
};

template<class ContainerAllocator>
struct DataType< ::ahra::UD_NeckAngleRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ahra/UD_NeckAngleRequest";
  }

  static const char* value(const ::ahra::UD_NeckAngleRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ahra::UD_NeckAngleRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool finish\n"
;
  }

  static const char* value(const ::ahra::UD_NeckAngleRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ahra::UD_NeckAngleRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.finish);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct UD_NeckAngleRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ahra::UD_NeckAngleRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ahra::UD_NeckAngleRequest_<ContainerAllocator>& v)
  {
    s << indent << "finish: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.finish);
  }
};

} // namespace message_operations
} // namespace ros

#endif // AHRA_MESSAGE_UD_NECKANGLEREQUEST_H
