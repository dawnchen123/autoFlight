// Generated by gencpp from file dji_sdk/MissionWaypoint.msg
// DO NOT EDIT!


#ifndef DJI_SDK_MESSAGE_MISSIONWAYPOINT_H
#define DJI_SDK_MESSAGE_MISSIONWAYPOINT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <dji_sdk/MissionWaypointAction.h>

namespace dji_sdk
{
template <class ContainerAllocator>
struct MissionWaypoint_
{
  typedef MissionWaypoint_<ContainerAllocator> Type;

  MissionWaypoint_()
    : latitude(0.0)
    , longitude(0.0)
    , altitude(0.0)
    , damping_distance(0.0)
    , target_yaw(0)
    , target_gimbal_pitch(0)
    , turn_mode(0)
    , has_action(0)
    , action_time_limit(0)
    , waypoint_action()  {
    }
  MissionWaypoint_(const ContainerAllocator& _alloc)
    : latitude(0.0)
    , longitude(0.0)
    , altitude(0.0)
    , damping_distance(0.0)
    , target_yaw(0)
    , target_gimbal_pitch(0)
    , turn_mode(0)
    , has_action(0)
    , action_time_limit(0)
    , waypoint_action(_alloc)  {
  (void)_alloc;
    }



   typedef double _latitude_type;
  _latitude_type latitude;

   typedef double _longitude_type;
  _longitude_type longitude;

   typedef float _altitude_type;
  _altitude_type altitude;

   typedef float _damping_distance_type;
  _damping_distance_type damping_distance;

   typedef int16_t _target_yaw_type;
  _target_yaw_type target_yaw;

   typedef int16_t _target_gimbal_pitch_type;
  _target_gimbal_pitch_type target_gimbal_pitch;

   typedef uint8_t _turn_mode_type;
  _turn_mode_type turn_mode;

   typedef uint8_t _has_action_type;
  _has_action_type has_action;

   typedef uint16_t _action_time_limit_type;
  _action_time_limit_type action_time_limit;

   typedef  ::dji_sdk::MissionWaypointAction_<ContainerAllocator>  _waypoint_action_type;
  _waypoint_action_type waypoint_action;





  typedef boost::shared_ptr< ::dji_sdk::MissionWaypoint_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dji_sdk::MissionWaypoint_<ContainerAllocator> const> ConstPtr;

}; // struct MissionWaypoint_

typedef ::dji_sdk::MissionWaypoint_<std::allocator<void> > MissionWaypoint;

typedef boost::shared_ptr< ::dji_sdk::MissionWaypoint > MissionWaypointPtr;
typedef boost::shared_ptr< ::dji_sdk::MissionWaypoint const> MissionWaypointConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dji_sdk::MissionWaypoint_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dji_sdk::MissionWaypoint_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace dji_sdk

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'nav_msgs': ['/opt/ros/kinetic/share/nav_msgs/cmake/../msg'], 'dji_sdk': ['/home/hp-dawn/\xe6\xa1\x8c\xe9\x9d\xa2/Onboard-SDK-ROS-3.8/src/dji_sdk/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg'], 'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::dji_sdk::MissionWaypoint_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dji_sdk::MissionWaypoint_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dji_sdk::MissionWaypoint_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dji_sdk::MissionWaypoint_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dji_sdk::MissionWaypoint_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dji_sdk::MissionWaypoint_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dji_sdk::MissionWaypoint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "22e006a33239b0dd80a9840e2fb0dd19";
  }

  static const char* value(const ::dji_sdk::MissionWaypoint_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x22e006a33239b0ddULL;
  static const uint64_t static_value2 = 0x80a9840e2fb0dd19ULL;
};

template<class ContainerAllocator>
struct DataType< ::dji_sdk::MissionWaypoint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dji_sdk/MissionWaypoint";
  }

  static const char* value(const ::dji_sdk::MissionWaypoint_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dji_sdk::MissionWaypoint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 latitude          # degree\n\
float64 longitude         # degree\n\
float32 altitude          # relative altitude from takeoff point\n\
float32 damping_distance  # Bend length (effective coordinated turn mode only)\n\
int16 target_yaw          # Yaw (degree)\n\
int16 target_gimbal_pitch # Gimbal pitch\n\
uint8 turn_mode           # 0: clockwise, 1: counter-clockwise\n\
uint8 has_action          # 0: no, 1: yes\n\
uint16 action_time_limit\n\
MissionWaypointAction waypoint_action\n\
\n\
================================================================================\n\
MSG: dji_sdk/MissionWaypointAction\n\
# action_repeat\n\
# lower 4 bit: Total number of actions\n\
# hight 4 bit: Total running times\n\
uint8 action_repeat\n\
uint8[16] command_list\n\
uint16[16] command_parameter\n\
";
  }

  static const char* value(const ::dji_sdk::MissionWaypoint_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dji_sdk::MissionWaypoint_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.latitude);
      stream.next(m.longitude);
      stream.next(m.altitude);
      stream.next(m.damping_distance);
      stream.next(m.target_yaw);
      stream.next(m.target_gimbal_pitch);
      stream.next(m.turn_mode);
      stream.next(m.has_action);
      stream.next(m.action_time_limit);
      stream.next(m.waypoint_action);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MissionWaypoint_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dji_sdk::MissionWaypoint_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dji_sdk::MissionWaypoint_<ContainerAllocator>& v)
  {
    s << indent << "latitude: ";
    Printer<double>::stream(s, indent + "  ", v.latitude);
    s << indent << "longitude: ";
    Printer<double>::stream(s, indent + "  ", v.longitude);
    s << indent << "altitude: ";
    Printer<float>::stream(s, indent + "  ", v.altitude);
    s << indent << "damping_distance: ";
    Printer<float>::stream(s, indent + "  ", v.damping_distance);
    s << indent << "target_yaw: ";
    Printer<int16_t>::stream(s, indent + "  ", v.target_yaw);
    s << indent << "target_gimbal_pitch: ";
    Printer<int16_t>::stream(s, indent + "  ", v.target_gimbal_pitch);
    s << indent << "turn_mode: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.turn_mode);
    s << indent << "has_action: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.has_action);
    s << indent << "action_time_limit: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.action_time_limit);
    s << indent << "waypoint_action: ";
    s << std::endl;
    Printer< ::dji_sdk::MissionWaypointAction_<ContainerAllocator> >::stream(s, indent + "  ", v.waypoint_action);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DJI_SDK_MESSAGE_MISSIONWAYPOINT_H
