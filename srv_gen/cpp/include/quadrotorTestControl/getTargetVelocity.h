/* Auto-generated by genmsg_cpp for file /home/cooplab/jianxin/rosbuild_ws/sandbox/quadrotorTestControl/srv/getTargetVelocity.srv */
#ifndef QUADROTORTESTCONTROL_SERVICE_GETTARGETVELOCITY_H
#define QUADROTORTESTCONTROL_SERVICE_GETTARGETVELOCITY_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "ros/service_traits.h"




namespace quadrotorTestControl
{
template <class ContainerAllocator>
struct getTargetVelocityRequest_ {
  typedef getTargetVelocityRequest_<ContainerAllocator> Type;

  getTargetVelocityRequest_()
  {
  }

  getTargetVelocityRequest_(const ContainerAllocator& _alloc)
  {
  }


  typedef boost::shared_ptr< ::quadrotorTestControl::getTargetVelocityRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::quadrotorTestControl::getTargetVelocityRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct getTargetVelocityRequest
typedef  ::quadrotorTestControl::getTargetVelocityRequest_<std::allocator<void> > getTargetVelocityRequest;

typedef boost::shared_ptr< ::quadrotorTestControl::getTargetVelocityRequest> getTargetVelocityRequestPtr;
typedef boost::shared_ptr< ::quadrotorTestControl::getTargetVelocityRequest const> getTargetVelocityRequestConstPtr;



template <class ContainerAllocator>
struct getTargetVelocityResponse_ {
  typedef getTargetVelocityResponse_<ContainerAllocator> Type;

  getTargetVelocityResponse_()
  : x(0.0)
  , y(0.0)
  , z(0.0)
  {
  }

  getTargetVelocityResponse_(const ContainerAllocator& _alloc)
  : x(0.0)
  , y(0.0)
  , z(0.0)
  {
  }

  typedef float _x_type;
  float x;

  typedef float _y_type;
  float y;

  typedef float _z_type;
  float z;


  typedef boost::shared_ptr< ::quadrotorTestControl::getTargetVelocityResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::quadrotorTestControl::getTargetVelocityResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct getTargetVelocityResponse
typedef  ::quadrotorTestControl::getTargetVelocityResponse_<std::allocator<void> > getTargetVelocityResponse;

typedef boost::shared_ptr< ::quadrotorTestControl::getTargetVelocityResponse> getTargetVelocityResponsePtr;
typedef boost::shared_ptr< ::quadrotorTestControl::getTargetVelocityResponse const> getTargetVelocityResponseConstPtr;


struct getTargetVelocity
{

typedef getTargetVelocityRequest Request;
typedef getTargetVelocityResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct getTargetVelocity
} // namespace quadrotorTestControl

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::quadrotorTestControl::getTargetVelocityRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::quadrotorTestControl::getTargetVelocityRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::quadrotorTestControl::getTargetVelocityRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const  ::quadrotorTestControl::getTargetVelocityRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::quadrotorTestControl::getTargetVelocityRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "quadrotorTestControl/getTargetVelocityRequest";
  }

  static const char* value(const  ::quadrotorTestControl::getTargetVelocityRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::quadrotorTestControl::getTargetVelocityRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
";
  }

  static const char* value(const  ::quadrotorTestControl::getTargetVelocityRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::quadrotorTestControl::getTargetVelocityRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::quadrotorTestControl::getTargetVelocityResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::quadrotorTestControl::getTargetVelocityResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::quadrotorTestControl::getTargetVelocityResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "cc153912f1453b708d221682bc23d9ac";
  }

  static const char* value(const  ::quadrotorTestControl::getTargetVelocityResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xcc153912f1453b70ULL;
  static const uint64_t static_value2 = 0x8d221682bc23d9acULL;
};

template<class ContainerAllocator>
struct DataType< ::quadrotorTestControl::getTargetVelocityResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "quadrotorTestControl/getTargetVelocityResponse";
  }

  static const char* value(const  ::quadrotorTestControl::getTargetVelocityResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::quadrotorTestControl::getTargetVelocityResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "float32 x\n\
float32 y\n\
float32 z\n\
\n\
\n\
";
  }

  static const char* value(const  ::quadrotorTestControl::getTargetVelocityResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::quadrotorTestControl::getTargetVelocityResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::quadrotorTestControl::getTargetVelocityRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct getTargetVelocityRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::quadrotorTestControl::getTargetVelocityResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.x);
    stream.next(m.y);
    stream.next(m.z);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct getTargetVelocityResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<quadrotorTestControl::getTargetVelocity> {
  static const char* value() 
  {
    return "cc153912f1453b708d221682bc23d9ac";
  }

  static const char* value(const quadrotorTestControl::getTargetVelocity&) { return value(); } 
};

template<>
struct DataType<quadrotorTestControl::getTargetVelocity> {
  static const char* value() 
  {
    return "quadrotorTestControl/getTargetVelocity";
  }

  static const char* value(const quadrotorTestControl::getTargetVelocity&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<quadrotorTestControl::getTargetVelocityRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "cc153912f1453b708d221682bc23d9ac";
  }

  static const char* value(const quadrotorTestControl::getTargetVelocityRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<quadrotorTestControl::getTargetVelocityRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "quadrotorTestControl/getTargetVelocity";
  }

  static const char* value(const quadrotorTestControl::getTargetVelocityRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<quadrotorTestControl::getTargetVelocityResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "cc153912f1453b708d221682bc23d9ac";
  }

  static const char* value(const quadrotorTestControl::getTargetVelocityResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<quadrotorTestControl::getTargetVelocityResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "quadrotorTestControl/getTargetVelocity";
  }

  static const char* value(const quadrotorTestControl::getTargetVelocityResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // QUADROTORTESTCONTROL_SERVICE_GETTARGETVELOCITY_H

