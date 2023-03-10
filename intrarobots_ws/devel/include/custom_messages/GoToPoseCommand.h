// Generated by gencpp from file custom_messages/GoToPoseCommand.msg
// DO NOT EDIT!


#ifndef CUSTOM_MESSAGES_MESSAGE_GOTOPOSECOMMAND_H
#define CUSTOM_MESSAGES_MESSAGE_GOTOPOSECOMMAND_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/Pose.h>

namespace custom_messages
{
template <class ContainerAllocator>
struct GoToPoseCommand_
{
  typedef GoToPoseCommand_<ContainerAllocator> Type;

  GoToPoseCommand_()
    : header()
    , robot_name()
    , command_id()
    , target_pose()  {
    }
  GoToPoseCommand_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , robot_name(_alloc)
    , command_id(_alloc)
    , target_pose(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _robot_name_type;
  _robot_name_type robot_name;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _command_id_type;
  _command_id_type command_id;

   typedef  ::geometry_msgs::Pose_<ContainerAllocator>  _target_pose_type;
  _target_pose_type target_pose;





  typedef boost::shared_ptr< ::custom_messages::GoToPoseCommand_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::custom_messages::GoToPoseCommand_<ContainerAllocator> const> ConstPtr;

}; // struct GoToPoseCommand_

typedef ::custom_messages::GoToPoseCommand_<std::allocator<void> > GoToPoseCommand;

typedef boost::shared_ptr< ::custom_messages::GoToPoseCommand > GoToPoseCommandPtr;
typedef boost::shared_ptr< ::custom_messages::GoToPoseCommand const> GoToPoseCommandConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::custom_messages::GoToPoseCommand_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::custom_messages::GoToPoseCommand_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::custom_messages::GoToPoseCommand_<ContainerAllocator1> & lhs, const ::custom_messages::GoToPoseCommand_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.robot_name == rhs.robot_name &&
    lhs.command_id == rhs.command_id &&
    lhs.target_pose == rhs.target_pose;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::custom_messages::GoToPoseCommand_<ContainerAllocator1> & lhs, const ::custom_messages::GoToPoseCommand_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace custom_messages

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::custom_messages::GoToPoseCommand_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::custom_messages::GoToPoseCommand_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::custom_messages::GoToPoseCommand_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::custom_messages::GoToPoseCommand_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::custom_messages::GoToPoseCommand_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::custom_messages::GoToPoseCommand_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::custom_messages::GoToPoseCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "1b4f85f247f494474e3eb1a08aaaa2cb";
  }

  static const char* value(const ::custom_messages::GoToPoseCommand_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x1b4f85f247f49447ULL;
  static const uint64_t static_value2 = 0x4e3eb1a08aaaa2cbULL;
};

template<class ContainerAllocator>
struct DataType< ::custom_messages::GoToPoseCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "custom_messages/GoToPoseCommand";
  }

  static const char* value(const ::custom_messages::GoToPoseCommand_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::custom_messages::GoToPoseCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"string robot_name\n"
"string command_id\n"
"geometry_msgs/Pose target_pose\n"
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
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Pose\n"
"# A representation of pose in free space, composed of position and orientation. \n"
"Point position\n"
"Quaternion orientation\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Quaternion\n"
"# This represents an orientation in free space in quaternion form.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 w\n"
;
  }

  static const char* value(const ::custom_messages::GoToPoseCommand_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::custom_messages::GoToPoseCommand_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.robot_name);
      stream.next(m.command_id);
      stream.next(m.target_pose);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GoToPoseCommand_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::custom_messages::GoToPoseCommand_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::custom_messages::GoToPoseCommand_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "robot_name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.robot_name);
    s << indent << "command_id: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.command_id);
    s << indent << "target_pose: ";
    s << std::endl;
    Printer< ::geometry_msgs::Pose_<ContainerAllocator> >::stream(s, indent + "  ", v.target_pose);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CUSTOM_MESSAGES_MESSAGE_GOTOPOSECOMMAND_H
