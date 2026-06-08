// Copyright 2022, Kenji Brameld All rights reserved.
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

#include <gtest/gtest.h>

#include "message_filters/message_traits.hpp"
#include "rclcpp/time.hpp"
#include "std_msgs/msg/header.hpp"

namespace
{

struct Msg
{
  std_msgs::msg::Header header;
};

struct HeaderlessMsg
{
  int32_t seconds;
  int32_t nanoseconds;
};

template<typename MessageType>
struct TimeGetterNoHeaderCustom
  : public message_filters::message_traits::TimeGetterBase<MessageType>
{
  static rclcpp::Time getTime(const MessageType & message)
  {
    return rclcpp::Time(message.seconds, message.nanoseconds, RCL_ROS_TIME);
  }
};

template<typename MessageType>
struct TimeGetterZeroTimestampCustom
  : public message_filters::message_traits::TimeGetterBase<MessageType>
{
  static rclcpp::Time getTime(const MessageType & message)
  {
    (void)message;
    return rclcpp::Time(0, 0, RCL_ROS_TIME);
  }
};

// Test that message_filters::message_traits::TimeStamp<Msg>::value returns RCL_ROS_TIME.
TEST(MessageTraits, timeSource)
{
  Msg msg;
  rclcpp::Time time = message_filters::message_traits::TimeStamp<Msg>::value(msg);

  EXPECT_EQ(time.get_clock_type(), RCL_ROS_TIME);

  // Ensure an exception isn't thrown when compared with a RCL_ROS_TIME time.
  bool unused;
  EXPECT_NO_THROW(unused = (time == rclcpp::Time{msg.header.stamp, RCL_ROS_TIME}));
  (void)unused;
}

TEST(MessageTraits, testTimeStampCustomHasHeader)
{
  Msg msg;
  msg.header.stamp.sec = uint32_t(1);
  msg.header.stamp.nanosec = uint32_t(2);
  rclcpp::Time time = message_filters::message_traits::TimeStampCustom<
    Msg,
    message_filters::message_traits::TimeGetterBase
    >::value(msg);

  EXPECT_EQ(time.get_clock_type(), RCL_ROS_TIME);

  EXPECT_EQ(time.nanoseconds(), uint32_t(1000000002));

  msg.header.stamp.nanosec = uint32_t(3);
  time = message_filters::message_traits::TimeStampCustom<
    Msg,
    message_filters::message_traits::TimeGetterBase
    >::value(msg);

  EXPECT_EQ(time.nanoseconds(), uint32_t(1000000003));
}

TEST(MessageTraits, testTimeStampCustomNoHeader)
{
  HeaderlessMsg msg;
  msg.seconds = uint32_t(1);
  msg.nanoseconds = uint32_t(2);

  rclcpp::Time time = message_filters::message_traits::TimeStampCustom<
    HeaderlessMsg,
    message_filters::message_traits::TimeGetterBase
    >::value(msg);

  EXPECT_EQ(time.get_clock_type(), RCL_ROS_TIME);

  EXPECT_EQ(time.nanoseconds(), uint32_t(0));
}

TEST(MessageTraits, testTimeStampCustomHasHeaderCustom)
{
  Msg msg;
  msg.header.stamp.sec = uint32_t(1);
  msg.header.stamp.nanosec = uint32_t(2);

  rclcpp::Time time = message_filters::message_traits::TimeStampCustom<
    Msg,
    TimeGetterZeroTimestampCustom
    >::value(msg);

  EXPECT_EQ(time.get_clock_type(), RCL_ROS_TIME);

  EXPECT_EQ(time.nanoseconds(), uint32_t(0));
}

TEST(MessageTraits, testTimeStampCustomHeaderlessCustom)
{
  HeaderlessMsg msg;
  msg.seconds = uint32_t(1);
  msg.nanoseconds = uint32_t(2);

  rclcpp::Time time = message_filters::message_traits::TimeStampCustom<
    HeaderlessMsg,
    TimeGetterNoHeaderCustom
    >::value(msg);

  EXPECT_EQ(time.get_clock_type(), RCL_ROS_TIME);

  EXPECT_EQ(time.nanoseconds(), uint32_t(1000000002));

  msg.nanoseconds = uint32_t(3);
  time = message_filters::message_traits::TimeStampCustom<
    HeaderlessMsg,
    TimeGetterNoHeaderCustom
    >::value(msg);

  EXPECT_EQ(time.nanoseconds(), uint32_t(1000000003));
}
}  // namespace
