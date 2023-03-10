// Generated by gencpp from file robotic_arm_moveit_interface/move_gripper_confirmation.msg
// DO NOT EDIT!


#ifndef ROBOTIC_ARM_MOVEIT_INTERFACE_MESSAGE_MOVE_GRIPPER_CONFIRMATION_H
#define ROBOTIC_ARM_MOVEIT_INTERFACE_MESSAGE_MOVE_GRIPPER_CONFIRMATION_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace robotic_arm_moveit_interface
{
template <class ContainerAllocator>
struct move_gripper_confirmation_
{
  typedef move_gripper_confirmation_<ContainerAllocator> Type;

  move_gripper_confirmation_()
    : header()
    , robot_name()
    , cmd()
    , opening_percentage(0.0)  {
    }
  move_gripper_confirmation_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , robot_name(_alloc)
    , cmd(_alloc)
    , opening_percentage(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _robot_name_type;
  _robot_name_type robot_name;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _cmd_type;
  _cmd_type cmd;

   typedef float _opening_percentage_type;
  _opening_percentage_type opening_percentage;





  typedef boost::shared_ptr< ::robotic_arm_moveit_interface::move_gripper_confirmation_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::robotic_arm_moveit_interface::move_gripper_confirmation_<ContainerAllocator> const> ConstPtr;

}; // struct move_gripper_confirmation_

typedef ::robotic_arm_moveit_interface::move_gripper_confirmation_<std::allocator<void> > move_gripper_confirmation;

typedef boost::shared_ptr< ::robotic_arm_moveit_interface::move_gripper_confirmation > move_gripper_confirmationPtr;
typedef boost::shared_ptr< ::robotic_arm_moveit_interface::move_gripper_confirmation const> move_gripper_confirmationConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::robotic_arm_moveit_interface::move_gripper_confirmation_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::robotic_arm_moveit_interface::move_gripper_confirmation_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::robotic_arm_moveit_interface::move_gripper_confirmation_<ContainerAllocator1> & lhs, const ::robotic_arm_moveit_interface::move_gripper_confirmation_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.robot_name == rhs.robot_name &&
    lhs.cmd == rhs.cmd &&
    lhs.opening_percentage == rhs.opening_percentage;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::robotic_arm_moveit_interface::move_gripper_confirmation_<ContainerAllocator1> & lhs, const ::robotic_arm_moveit_interface::move_gripper_confirmation_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace robotic_arm_moveit_interface

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::robotic_arm_moveit_interface::move_gripper_confirmation_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robotic_arm_moveit_interface::move_gripper_confirmation_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::robotic_arm_moveit_interface::move_gripper_confirmation_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::robotic_arm_moveit_interface::move_gripper_confirmation_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robotic_arm_moveit_interface::move_gripper_confirmation_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robotic_arm_moveit_interface::move_gripper_confirmation_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::robotic_arm_moveit_interface::move_gripper_confirmation_<ContainerAllocator> >
{
  static const char* value()
  {
    return "88a8a98da02f94ae31efa1de1c3281a7";
  }

  static const char* value(const ::robotic_arm_moveit_interface::move_gripper_confirmation_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x88a8a98da02f94aeULL;
  static const uint64_t static_value2 = 0x31efa1de1c3281a7ULL;
};

template<class ContainerAllocator>
struct DataType< ::robotic_arm_moveit_interface::move_gripper_confirmation_<ContainerAllocator> >
{
  static const char* value()
  {
    return "robotic_arm_moveit_interface/move_gripper_confirmation";
  }

  static const char* value(const ::robotic_arm_moveit_interface::move_gripper_confirmation_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::robotic_arm_moveit_interface::move_gripper_confirmation_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"string robot_name\n"
"string cmd\n"
"float32 opening_percentage\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
;
  }

  static const char* value(const ::robotic_arm_moveit_interface::move_gripper_confirmation_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::robotic_arm_moveit_interface::move_gripper_confirmation_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.robot_name);
      stream.next(m.cmd);
      stream.next(m.opening_percentage);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct move_gripper_confirmation_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::robotic_arm_moveit_interface::move_gripper_confirmation_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::robotic_arm_moveit_interface::move_gripper_confirmation_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "robot_name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.robot_name);
    s << indent << "cmd: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.cmd);
    s << indent << "opening_percentage: ";
    Printer<float>::stream(s, indent + "  ", v.opening_percentage);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROBOTIC_ARM_MOVEIT_INTERFACE_MESSAGE_MOVE_GRIPPER_CONFIRMATION_H
