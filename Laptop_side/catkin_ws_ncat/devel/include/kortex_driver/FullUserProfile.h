// Generated by gencpp from file kortex_driver/FullUserProfile.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_FULLUSERPROFILE_H
#define KORTEX_DRIVER_MESSAGE_FULLUSERPROFILE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <kortex_driver/UserProfile.h>

namespace kortex_driver
{
template <class ContainerAllocator>
struct FullUserProfile_
{
  typedef FullUserProfile_<ContainerAllocator> Type;

  FullUserProfile_()
    : user_profile()
    , password()  {
    }
  FullUserProfile_(const ContainerAllocator& _alloc)
    : user_profile(_alloc)
    , password(_alloc)  {
  (void)_alloc;
    }



   typedef  ::kortex_driver::UserProfile_<ContainerAllocator>  _user_profile_type;
  _user_profile_type user_profile;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _password_type;
  _password_type password;





  typedef boost::shared_ptr< ::kortex_driver::FullUserProfile_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::FullUserProfile_<ContainerAllocator> const> ConstPtr;

}; // struct FullUserProfile_

typedef ::kortex_driver::FullUserProfile_<std::allocator<void> > FullUserProfile;

typedef boost::shared_ptr< ::kortex_driver::FullUserProfile > FullUserProfilePtr;
typedef boost::shared_ptr< ::kortex_driver::FullUserProfile const> FullUserProfileConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::FullUserProfile_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::FullUserProfile_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::FullUserProfile_<ContainerAllocator1> & lhs, const ::kortex_driver::FullUserProfile_<ContainerAllocator2> & rhs)
{
  return lhs.user_profile == rhs.user_profile &&
    lhs.password == rhs.password;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::FullUserProfile_<ContainerAllocator1> & lhs, const ::kortex_driver::FullUserProfile_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::FullUserProfile_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::FullUserProfile_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::FullUserProfile_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::FullUserProfile_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::FullUserProfile_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::FullUserProfile_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::FullUserProfile_<ContainerAllocator> >
{
  static const char* value()
  {
    return "5832f6aa4b0c784ce33a85ff505da582";
  }

  static const char* value(const ::kortex_driver::FullUserProfile_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x5832f6aa4b0c784cULL;
  static const uint64_t static_value2 = 0xe33a85ff505da582ULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::FullUserProfile_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/FullUserProfile";
  }

  static const char* value(const ::kortex_driver::FullUserProfile_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::FullUserProfile_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"UserProfile user_profile\n"
"string password\n"
"================================================================================\n"
"MSG: kortex_driver/UserProfile\n"
"\n"
"UserProfileHandle handle\n"
"string username\n"
"string firstname\n"
"string lastname\n"
"string application_data\n"
"================================================================================\n"
"MSG: kortex_driver/UserProfileHandle\n"
"\n"
"uint32 identifier\n"
"uint32 permission\n"
;
  }

  static const char* value(const ::kortex_driver::FullUserProfile_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::FullUserProfile_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.user_profile);
      stream.next(m.password);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct FullUserProfile_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::FullUserProfile_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::FullUserProfile_<ContainerAllocator>& v)
  {
    s << indent << "user_profile: ";
    s << std::endl;
    Printer< ::kortex_driver::UserProfile_<ContainerAllocator> >::stream(s, indent + "  ", v.user_profile);
    s << indent << "password: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.password);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_FULLUSERPROFILE_H