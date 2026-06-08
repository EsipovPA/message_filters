// Copyright 2009, Willow Garage, Inc. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Willow Garage nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// File imported from
// https://github.com/ros/roscpp_core/blob/38b9663/roscpp_traits/include/ros/message_traits.h

#ifndef MESSAGE_FILTERS__MESSAGE_TRAITS_HPP_
#define MESSAGE_FILTERS__MESSAGE_TRAITS_HPP_

#include <string>
#include <stdexcept>
#include <type_traits>

#include <rclcpp/time.hpp>
#include <std_msgs/msg/header.hpp>

namespace message_filters
{
namespace message_traits
{

/**
 * False if the message does not have a header
 * @tparam MessageType
 */
template<typename MessageType, typename = void>
struct HasHeader : public std::false_type {};

/**
 * True if the message has a field named 'header' with a type of std_msgs::msg::Header
 * @tparam MessageType
 */
template<typename MessageType>
struct HasHeader<MessageType, typename std::enable_if<std::is_same<std_msgs::msg::Header,
  decltype(MessageType().header)>::value>::type>: public std::true_type {};

/**
 * \brief FrameId trait.  In the default implementation pointer()
 * returns &m.header.frame_id if HasHeader<MessageType>::value is true, otherwise returns NULL.  value()
 * does not exist, and causes a compile error
 */
template<typename MessageType, typename Enable = void>
struct FrameId
{
  static std::string * pointer(MessageType & m) {(void)m; return nullptr;}
  static std::string const * pointer(const MessageType & m) {(void)m; return nullptr;}
};
template<typename MessageType>
struct FrameId<MessageType, typename std::enable_if<HasHeader<MessageType>::value>::type>
{
  static std::string * pointer(MessageType & m) {return &m.header.frame_id;}
  static std::string const * pointer(const MessageType & m) {return &m.header.frame_id;}
  static std::string value(const MessageType & m) {return m.header.frame_id;}
};

/**
 * \brief TimeGetterBase is a virtual abstract class
 * The actual class should implement the getTime method
 * that should retrieve rclcpp::Time from a message
 */
template<typename MessageType>
class TimeGetterBase
{
public:
  static rclcpp::Time getTime(const MessageType & message)
  {
    (void)message;
    throw std::logic_error("getTime not implemented for base TimeGetterBase class");
  }
};

template<typename MessageType>
class HeaderTime : public TimeGetterBase<MessageType>
{
public:
  static rclcpp::Time getTime(const MessageType & message)
  {
    return rclcpp::Time(message.header.stamp, RCL_ROS_TIME);
  }
};


template<typename MessageType>
class NullTime : public TimeGetterBase<MessageType>
{
public:
  static rclcpp::Time getTime(const MessageType & message)
  {
    (void)message;
    return rclcpp::Time(0, 0, RCL_ROS_TIME);
  }
};


template<typename MessageType, template<typename GetterMessageType> typename TimeGetter>
class TimeStampBase
{
public:
  static rclcpp::Time value(const MessageType & message)
  {
    return TimeGetter<MessageType>::getTime(message);
  }
};

/**
 * \brief TimeStamp trait.  In the default implementation pointer()
 * returns &m.header.stamp if HasHeader<MessageType>::value is true, otherwise returns NULL.  value()
 * does not exist, and causes a compile error
 */
template<typename MessageType, typename Enable = void>
struct TimeStamp;

template<typename MessageType>
struct TimeStamp<MessageType, typename std::enable_if_t<!HasHeader<MessageType>::value>>
  : public TimeStampBase<MessageType, NullTime>
{};

template<typename MessageType>
struct TimeStamp<MessageType, typename std::enable_if_t<HasHeader<MessageType>::value>>
  : public TimeStampBase<MessageType, HeaderTime>
{};

template<
  typename MessageType,
  template<typename GetterMessageType> typename TimeGetter,
  typename Enable = void
>
struct TimeStampCustom;

template<
  typename MessageType,
  template<typename GetterMessageType> typename TimeGetter
>
struct TimeStampCustom<
  MessageType,
  TimeGetter,
  typename std::enable_if_t<
    !HasHeader<MessageType>::value &&
    std::is_same_v<TimeGetter<MessageType>, TimeGetterBase<MessageType>>
  >
>: public TimeStampBase<MessageType, NullTime>
{};

template<
  typename MessageType,
  template<typename GetterMessageType> typename TimeGetter
>
struct TimeStampCustom<
  MessageType,
  TimeGetter,
  typename std::enable_if_t<HasHeader<MessageType>::value && std::is_same_v<TimeGetter<MessageType>,
  TimeGetterBase<MessageType>>>
>: public TimeStampBase<MessageType, HeaderTime>
{};


template<
  typename MessageType,
  template<typename GetterMessageType> typename TimeGetter
>
struct TimeStampCustom<
  MessageType,
  TimeGetter,
  typename std::enable_if_t<!std::is_same_v<TimeGetter<MessageType>, TimeGetterBase<MessageType>>>
>: public TimeStampBase<MessageType, TimeGetter>
{};

}  // namespace message_traits
}  // namespace message_filters

#endif  // MESSAGE_FILTERS__MESSAGE_TRAITS_HPP_
