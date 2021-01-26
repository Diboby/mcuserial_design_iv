// Generated by gencpp from file mcuserial_msgs/RequestParamResponse.msg
// DO NOT EDIT!


#ifndef MCUSERIAL_MSGS_MESSAGE_REQUESTPARAMRESPONSE_H
#define MCUSERIAL_MSGS_MESSAGE_REQUESTPARAMRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace mcuserial_msgs
{
template <class ContainerAllocator>
struct RequestParamResponse_
{
  typedef RequestParamResponse_<ContainerAllocator> Type;

  RequestParamResponse_()
    : ints()
    , floats()
    , strings()  {
    }
  RequestParamResponse_(const ContainerAllocator& _alloc)
    : ints(_alloc)
    , floats(_alloc)
    , strings(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other >  _ints_type;
  _ints_type ints;

   typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _floats_type;
  _floats_type floats;

   typedef std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  _strings_type;
  _strings_type strings;





  typedef boost::shared_ptr< ::mcuserial_msgs::RequestParamResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mcuserial_msgs::RequestParamResponse_<ContainerAllocator> const> ConstPtr;

}; // struct RequestParamResponse_

typedef ::mcuserial_msgs::RequestParamResponse_<std::allocator<void> > RequestParamResponse;

typedef boost::shared_ptr< ::mcuserial_msgs::RequestParamResponse > RequestParamResponsePtr;
typedef boost::shared_ptr< ::mcuserial_msgs::RequestParamResponse const> RequestParamResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mcuserial_msgs::RequestParamResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mcuserial_msgs::RequestParamResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::mcuserial_msgs::RequestParamResponse_<ContainerAllocator1> & lhs, const ::mcuserial_msgs::RequestParamResponse_<ContainerAllocator2> & rhs)
{
  return lhs.ints == rhs.ints &&
    lhs.floats == rhs.floats &&
    lhs.strings == rhs.strings;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::mcuserial_msgs::RequestParamResponse_<ContainerAllocator1> & lhs, const ::mcuserial_msgs::RequestParamResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace mcuserial_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::mcuserial_msgs::RequestParamResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mcuserial_msgs::RequestParamResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mcuserial_msgs::RequestParamResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mcuserial_msgs::RequestParamResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mcuserial_msgs::RequestParamResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mcuserial_msgs::RequestParamResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mcuserial_msgs::RequestParamResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "9f0e98bda65981986ddf53afa7a40e49";
  }

  static const char* value(const ::mcuserial_msgs::RequestParamResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x9f0e98bda6598198ULL;
  static const uint64_t static_value2 = 0x6ddf53afa7a40e49ULL;
};

template<class ContainerAllocator>
struct DataType< ::mcuserial_msgs::RequestParamResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mcuserial_msgs/RequestParamResponse";
  }

  static const char* value(const ::mcuserial_msgs::RequestParamResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mcuserial_msgs::RequestParamResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"int32[]   ints\n"
"float32[] floats\n"
"string[]  strings\n"
"\n"
;
  }

  static const char* value(const ::mcuserial_msgs::RequestParamResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mcuserial_msgs::RequestParamResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.ints);
      stream.next(m.floats);
      stream.next(m.strings);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct RequestParamResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mcuserial_msgs::RequestParamResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::mcuserial_msgs::RequestParamResponse_<ContainerAllocator>& v)
  {
    s << indent << "ints[]" << std::endl;
    for (size_t i = 0; i < v.ints.size(); ++i)
    {
      s << indent << "  ints[" << i << "]: ";
      Printer<int32_t>::stream(s, indent + "  ", v.ints[i]);
    }
    s << indent << "floats[]" << std::endl;
    for (size_t i = 0; i < v.floats.size(); ++i)
    {
      s << indent << "  floats[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.floats[i]);
    }
    s << indent << "strings[]" << std::endl;
    for (size_t i = 0; i < v.strings.size(); ++i)
    {
      s << indent << "  strings[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.strings[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // MCUSERIAL_MSGS_MESSAGE_REQUESTPARAMRESPONSE_H
