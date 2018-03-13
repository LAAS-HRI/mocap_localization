/* Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Auto-generated by genmsg_cpp from file /home/gmilliez/robotpkg/localization/optitrack-genom3/work.elea/templates/ros/client/ros/optitrack/msg/or_time_ts.msg
 *
 */


#ifndef OPTITRACK_MESSAGE_OR_TIME_TS_H
#define OPTITRACK_MESSAGE_OR_TIME_TS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace optitrack
{
template <class ContainerAllocator>
struct or_time_ts_
{
  typedef or_time_ts_<ContainerAllocator> Type;

  or_time_ts_()
    : sec(0)
    , nsec(0)  {
    }
  or_time_ts_(const ContainerAllocator& _alloc)
    : sec(0)
    , nsec(0)  {
    }



   typedef uint32_t _sec_type;
  _sec_type sec;

   typedef uint32_t _nsec_type;
  _nsec_type nsec;




  typedef boost::shared_ptr< ::optitrack::or_time_ts_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::optitrack::or_time_ts_<ContainerAllocator> const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;

}; // struct or_time_ts_

typedef ::optitrack::or_time_ts_<std::allocator<void> > or_time_ts;

typedef boost::shared_ptr< ::optitrack::or_time_ts > or_time_tsPtr;
typedef boost::shared_ptr< ::optitrack::or_time_ts const> or_time_tsConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::optitrack::or_time_ts_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::optitrack::or_time_ts_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace optitrack

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/hydro/share/std_msgs/msg'], 'actionlib_msgs': ['/opt/ros/hydro/share/actionlib_msgs/msg'], 'optitrack': ['optitrack/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::optitrack::or_time_ts_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::optitrack::or_time_ts_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::optitrack::or_time_ts_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::optitrack::or_time_ts_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::optitrack::or_time_ts_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::optitrack::or_time_ts_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::optitrack::or_time_ts_<ContainerAllocator> >
{
  static const char* value()
  {
    return "4771ad66fef816d2e4bead2f45a1cde6";
  }

  static const char* value(const ::optitrack::or_time_ts_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x4771ad66fef816d2ULL;
  static const uint64_t static_value2 = 0xe4bead2f45a1cde6ULL;
};

template<class ContainerAllocator>
struct DataType< ::optitrack::or_time_ts_<ContainerAllocator> >
{
  static const char* value()
  {
    return "optitrack/or_time_ts";
  }

  static const char* value(const ::optitrack::or_time_ts_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::optitrack::or_time_ts_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# IDL struct ::or::time::ts\n\
uint32 sec\n\
uint32 nsec\n\
";
  }

  static const char* value(const ::optitrack::or_time_ts_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::optitrack::or_time_ts_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.sec);
      stream.next(m.nsec);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct or_time_ts_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::optitrack::or_time_ts_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::optitrack::or_time_ts_<ContainerAllocator>& v)
  {
    s << indent << "sec: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.sec);
    s << indent << "nsec: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.nsec);
  }
};

} // namespace message_operations
} // namespace ros

#endif // OPTITRACK_MESSAGE_OR_TIME_TS_H
