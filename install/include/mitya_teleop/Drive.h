// Generated by gencpp from file mitya_teleop/Drive.msg
// DO NOT EDIT!


#ifndef MITYA_TELEOP_MESSAGE_DRIVE_H
#define MITYA_TELEOP_MESSAGE_DRIVE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace mitya_teleop
{
template <class ContainerAllocator>
struct Drive_
{
  typedef Drive_<ContainerAllocator> Type;

  Drive_()
    : left(0)
    , right(0)  {
    }
  Drive_(const ContainerAllocator& _alloc)
    : left(0)
    , right(0)  {
  (void)_alloc;
    }



   typedef int8_t _left_type;
  _left_type left;

   typedef int8_t _right_type;
  _right_type right;




  typedef boost::shared_ptr< ::mitya_teleop::Drive_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mitya_teleop::Drive_<ContainerAllocator> const> ConstPtr;

}; // struct Drive_

typedef ::mitya_teleop::Drive_<std::allocator<void> > Drive;

typedef boost::shared_ptr< ::mitya_teleop::Drive > DrivePtr;
typedef boost::shared_ptr< ::mitya_teleop::Drive const> DriveConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mitya_teleop::Drive_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mitya_teleop::Drive_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace mitya_teleop

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'mitya_teleop': ['/home/dmitrydzz/dev/mitya3/ros3/src/mitya_teleop/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::mitya_teleop::Drive_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mitya_teleop::Drive_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mitya_teleop::Drive_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mitya_teleop::Drive_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mitya_teleop::Drive_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mitya_teleop::Drive_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mitya_teleop::Drive_<ContainerAllocator> >
{
  static const char* value()
  {
    return "24825b8956c21f4c3dd28a5a4d09322c";
  }

  static const char* value(const ::mitya_teleop::Drive_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x24825b8956c21f4cULL;
  static const uint64_t static_value2 = 0x3dd28a5a4d09322cULL;
};

template<class ContainerAllocator>
struct DataType< ::mitya_teleop::Drive_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mitya_teleop/Drive";
  }

  static const char* value(const ::mitya_teleop::Drive_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mitya_teleop::Drive_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int8 left\n\
int8 right\n\
";
  }

  static const char* value(const ::mitya_teleop::Drive_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mitya_teleop::Drive_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.left);
      stream.next(m.right);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Drive_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mitya_teleop::Drive_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::mitya_teleop::Drive_<ContainerAllocator>& v)
  {
    s << indent << "left: ";
    Printer<int8_t>::stream(s, indent + "  ", v.left);
    s << indent << "right: ";
    Printer<int8_t>::stream(s, indent + "  ", v.right);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MITYA_TELEOP_MESSAGE_DRIVE_H
