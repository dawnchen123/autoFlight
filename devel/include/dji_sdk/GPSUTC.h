// Generated by gencpp from file dji_sdk/GPSUTC.msg
// DO NOT EDIT!


#ifndef DJI_SDK_MESSAGE_GPSUTC_H
#define DJI_SDK_MESSAGE_GPSUTC_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace dji_sdk
{
template <class ContainerAllocator>
struct GPSUTC_
{
  typedef GPSUTC_<ContainerAllocator> Type;

  GPSUTC_()
    : stamp()
    , UTCTimeData()  {
    }
  GPSUTC_(const ContainerAllocator& _alloc)
    : stamp()
    , UTCTimeData(_alloc)  {
  (void)_alloc;
    }



   typedef ros::Time _stamp_type;
  _stamp_type stamp;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _UTCTimeData_type;
  _UTCTimeData_type UTCTimeData;





  typedef boost::shared_ptr< ::dji_sdk::GPSUTC_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dji_sdk::GPSUTC_<ContainerAllocator> const> ConstPtr;

}; // struct GPSUTC_

typedef ::dji_sdk::GPSUTC_<std::allocator<void> > GPSUTC;

typedef boost::shared_ptr< ::dji_sdk::GPSUTC > GPSUTCPtr;
typedef boost::shared_ptr< ::dji_sdk::GPSUTC const> GPSUTCConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dji_sdk::GPSUTC_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dji_sdk::GPSUTC_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace dji_sdk

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'nav_msgs': ['/opt/ros/kinetic/share/nav_msgs/cmake/../msg'], 'dji_sdk': ['/home/hp-dawn/\xe6\xa1\x8c\xe9\x9d\xa2/Onboard-SDK-ROS-3.8/src/dji_sdk/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg'], 'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::dji_sdk::GPSUTC_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dji_sdk::GPSUTC_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dji_sdk::GPSUTC_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dji_sdk::GPSUTC_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dji_sdk::GPSUTC_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dji_sdk::GPSUTC_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dji_sdk::GPSUTC_<ContainerAllocator> >
{
  static const char* value()
  {
    return "034c4baeda26125f91567e2717d1cf5b";
  }

  static const char* value(const ::dji_sdk::GPSUTC_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x034c4baeda26125fULL;
  static const uint64_t static_value2 = 0x91567e2717d1cf5bULL;
};

template<class ContainerAllocator>
struct DataType< ::dji_sdk::GPSUTC_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dji_sdk/GPSUTC";
  }

  static const char* value(const ::dji_sdk::GPSUTC_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dji_sdk::GPSUTC_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# the time stamp of getting this data in the ROS system\n\
time stamp\n\
\n\
# the time stamp from GPS/RTK\n\
string UTCTimeData\n\
";
  }

  static const char* value(const ::dji_sdk::GPSUTC_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dji_sdk::GPSUTC_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.stamp);
      stream.next(m.UTCTimeData);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GPSUTC_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dji_sdk::GPSUTC_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dji_sdk::GPSUTC_<ContainerAllocator>& v)
  {
    s << indent << "stamp: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.stamp);
    s << indent << "UTCTimeData: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.UTCTimeData);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DJI_SDK_MESSAGE_GPSUTC_H