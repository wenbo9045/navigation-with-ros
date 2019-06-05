// Generated by gencpp from file tri_local_planner/Position_float.msg
// DO NOT EDIT!


#ifndef TRI_LOCAL_PLANNER_MESSAGE_POSITION_FLOAT_H
#define TRI_LOCAL_PLANNER_MESSAGE_POSITION_FLOAT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace tri_local_planner
{
template <class ContainerAllocator>
struct Position_float_
{
  typedef Position_float_<ContainerAllocator> Type;

  Position_float_()
    : x(0.0)
    , y(0.0)  {
    }
  Position_float_(const ContainerAllocator& _alloc)
    : x(0.0)
    , y(0.0)  {
  (void)_alloc;
    }



   typedef float _x_type;
  _x_type x;

   typedef float _y_type;
  _y_type y;




  typedef boost::shared_ptr< ::tri_local_planner::Position_float_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::tri_local_planner::Position_float_<ContainerAllocator> const> ConstPtr;

}; // struct Position_float_

typedef ::tri_local_planner::Position_float_<std::allocator<void> > Position_float;

typedef boost::shared_ptr< ::tri_local_planner::Position_float > Position_floatPtr;
typedef boost::shared_ptr< ::tri_local_planner::Position_float const> Position_floatConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::tri_local_planner::Position_float_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::tri_local_planner::Position_float_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace tri_local_planner

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'tri_local_planner': ['/home/yuanrupeng/catkin_ws/src/tri_local_planner/msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::tri_local_planner::Position_float_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::tri_local_planner::Position_float_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::tri_local_planner::Position_float_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::tri_local_planner::Position_float_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::tri_local_planner::Position_float_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::tri_local_planner::Position_float_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::tri_local_planner::Position_float_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ff8d7d66dd3e4b731ef14a45d38888b6";
  }

  static const char* value(const ::tri_local_planner::Position_float_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xff8d7d66dd3e4b73ULL;
  static const uint64_t static_value2 = 0x1ef14a45d38888b6ULL;
};

template<class ContainerAllocator>
struct DataType< ::tri_local_planner::Position_float_<ContainerAllocator> >
{
  static const char* value()
  {
    return "tri_local_planner/Position_float";
  }

  static const char* value(const ::tri_local_planner::Position_float_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::tri_local_planner::Position_float_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 x\n\
float32 y\n\
";
  }

  static const char* value(const ::tri_local_planner::Position_float_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::tri_local_planner::Position_float_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.x);
      stream.next(m.y);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Position_float_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::tri_local_planner::Position_float_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::tri_local_planner::Position_float_<ContainerAllocator>& v)
  {
    s << indent << "x: ";
    Printer<float>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<float>::stream(s, indent + "  ", v.y);
  }
};

} // namespace message_operations
} // namespace ros

#endif // TRI_LOCAL_PLANNER_MESSAGE_POSITION_FLOAT_H
