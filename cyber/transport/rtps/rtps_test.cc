/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include <cstdint>
#include <string>
#include <utility>

#include "gtest/gtest.h"

#include "cyber/common/log.h"
#include "cyber/transport/qos/qos_filler.h"
#include "cyber/transport/qos/qos_profile_conf.h"
#include "cyber/transport/rtps/attributes_filler.h"
#include "cyber/transport/rtps/underlay_message.h"

namespace apollo {
namespace cyber {
namespace transport {

TEST(QosFillerTest, fill_in_pub_qos_test) {
  QosProfile qos;
  QosFiller filler;
  eprosima::fastdds::dds::PublisherQos pub_qos;
  eprosima::fastdds::dds::DataWriterQos writer_qos;
  qos.set_history(QosHistoryPolicy::HISTORY_KEEP_LAST);
  qos.set_durability(QosDurabilityPolicy::DURABILITY_TRANSIENT_LOCAL);
  qos.set_reliability(QosReliabilityPolicy::RELIABILITY_BEST_EFFORT);
  qos.set_mps(32);
  filler.FillInPubQos("channel", qos, &pub_qos);
  filler.FillInWriterQos("channel", qos, &writer_qos);
  EXPECT_EQ(eprosima::fastdds::dds::KEEP_LAST_HISTORY_QOS,
            writer_qos.history().kind);
  EXPECT_EQ(eprosima::fastdds::dds::TRANSIENT_LOCAL_DURABILITY_QOS,
            writer_qos.durability().kind);
  EXPECT_EQ(eprosima::fastdds::dds::BEST_EFFORT_RELIABILITY_QOS,
            writer_qos.reliability().kind);
  AINFO << "heartbeat period: "
        << writer_qos.reliable_writer_qos().times.heartbeatPeriod.seconds
        << ", "
        << writer_qos.reliable_writer_qos().times.heartbeatPeriod.nanosec;
  qos.set_depth(1024);
  filler.FillInPubQos("channel", qos, &pub_qos);
  filler.FillInWriterQos("channel", qos, &writer_qos);
  AINFO << qos.depth() << ", "
        << QosProfileConf::QOS_HISTORY_DEPTH_SYSTEM_DEFAULT << ", "
        << writer_qos.history().depth;
  EXPECT_EQ(qos.depth(), writer_qos.history().depth);

  qos.set_history(QosHistoryPolicy::HISTORY_KEEP_ALL);
  qos.set_durability(QosDurabilityPolicy::DURABILITY_VOLATILE);
  qos.set_reliability(QosReliabilityPolicy::RELIABILITY_RELIABLE);
  qos.set_mps(65);
  filler.FillInPubQos("channel", qos, &pub_qos);
  filler.FillInWriterQos("channel", qos, &writer_qos);
  EXPECT_EQ(eprosima::fastdds::dds::KEEP_ALL_HISTORY_QOS,
            writer_qos.history().kind);
  EXPECT_EQ(eprosima::fastdds::dds::VOLATILE_DURABILITY_QOS,
            writer_qos.durability().kind);
  EXPECT_EQ(eprosima::fastdds::dds::RELIABLE_RELIABILITY_QOS,
            writer_qos.reliability().kind);
  AINFO << "heartbeat period: "
        << writer_qos.reliable_writer_qos().times.heartbeatPeriod.seconds
        << ", "
        << writer_qos.reliable_writer_qos().times.heartbeatPeriod.nanosec;

  qos.set_history(QosHistoryPolicy::HISTORY_SYSTEM_DEFAULT);
  qos.set_durability(QosDurabilityPolicy::DURABILITY_SYSTEM_DEFAULT);
  qos.set_reliability(QosReliabilityPolicy::RELIABILITY_SYSTEM_DEFAULT);
  qos.set_mps(1025);
  filler.FillInPubQos("channel", qos, &pub_qos);
  filler.FillInWriterQos("channel", qos, &writer_qos);
  EXPECT_EQ(eprosima::fastdds::dds::KEEP_LAST_HISTORY_QOS,
            writer_qos.history().kind);
  EXPECT_EQ(eprosima::fastdds::dds::TRANSIENT_LOCAL_DURABILITY_QOS,
            writer_qos.durability().kind);
  EXPECT_EQ(eprosima::fastdds::dds::RELIABLE_RELIABILITY_QOS,
            writer_qos.reliability().kind);
  AINFO << "heartbeat period: "
        << writer_qos.reliable_writer_qos().times.heartbeatPeriod.seconds
        << ", "
        << writer_qos.reliable_writer_qos().times.heartbeatPeriod.nanosec;
  qos.set_mps(0);
  filler.FillInPubQos("channel", qos, &pub_qos);
  filler.FillInWriterQos("channel", qos, &writer_qos);
  AINFO << "heartbeat period: "
        << writer_qos.reliable_writer_qos().times.heartbeatPeriod.seconds
        << ", "
        << writer_qos.reliable_writer_qos().times.heartbeatPeriod.nanosec;
  qos.set_depth(-1);
  EXPECT_FALSE(filler.FillInPubQos("channel", qos, &pub_qos));
  EXPECT_FALSE(filler.FillInWriterQos("channel", qos, &writer_qos));
}

TEST(AttributesFillerTest, fill_in_sub_attr_test) {
  QosProfile qos;
  QosFiller filler;
  eprosima::fastdds::dds::SubscriberQos sub_qos;
  eprosima::fastdds::dds::DataReaderQos reader_qos;
  qos.set_history(QosHistoryPolicy::HISTORY_KEEP_LAST);
  qos.set_durability(QosDurabilityPolicy::DURABILITY_TRANSIENT_LOCAL);
  qos.set_reliability(QosReliabilityPolicy::RELIABILITY_BEST_EFFORT);
  qos.set_mps(32);
  filler.FillInSubQos("channel", qos, &sub_qos);
  filler.FillInReaderQos("channel", qos, &reader_qos);
  EXPECT_EQ(eprosima::fastdds::dds::KEEP_LAST_HISTORY_QOS,
            reader_qos.history().kind);
  EXPECT_EQ(eprosima::fastdds::dds::TRANSIENT_LOCAL_DURABILITY_QOS,
            reader_qos.durability().kind);
  EXPECT_EQ(eprosima::fastdds::dds::BEST_EFFORT_RELIABILITY_QOS,
            reader_qos.reliability().kind);
  qos.set_depth(1024);
  // attrs.topic.historyQos.depth = 512;
  filler.FillInSubQos("channel", qos, &sub_qos);
  filler.FillInReaderQos("channel", qos, &reader_qos);
  EXPECT_EQ(qos.depth(), reader_qos.history().depth);

  qos.set_history(QosHistoryPolicy::HISTORY_KEEP_ALL);
  qos.set_durability(QosDurabilityPolicy::DURABILITY_VOLATILE);
  qos.set_reliability(QosReliabilityPolicy::RELIABILITY_RELIABLE);
  qos.set_mps(65);
  filler.FillInSubQos("channel", qos, &sub_qos);
  filler.FillInReaderQos("channel", qos, &reader_qos);
  EXPECT_EQ(eprosima::fastdds::dds::KEEP_ALL_HISTORY_QOS,
            reader_qos.history().kind);
  EXPECT_EQ(eprosima::fastdds::dds::VOLATILE_DURABILITY_QOS,
            reader_qos.durability().kind);
  EXPECT_EQ(eprosima::fastdds::dds::RELIABLE_RELIABILITY_QOS,
            reader_qos.reliability().kind);

  qos.set_history(QosHistoryPolicy::HISTORY_SYSTEM_DEFAULT);
  qos.set_durability(QosDurabilityPolicy::DURABILITY_SYSTEM_DEFAULT);
  qos.set_reliability(QosReliabilityPolicy::RELIABILITY_SYSTEM_DEFAULT);
  qos.set_mps(1025);
  filler.FillInSubQos("channel", qos, &sub_qos);
  filler.FillInReaderQos("channel", qos, &reader_qos);
  EXPECT_EQ(eprosima::fastdds::dds::KEEP_LAST_HISTORY_QOS,
            reader_qos.history().kind);
  EXPECT_EQ(eprosima::fastdds::dds::VOLATILE_DURABILITY_QOS,
            reader_qos.durability().kind);
  EXPECT_EQ(eprosima::fastdds::dds::BEST_EFFORT_RELIABILITY_QOS,
            reader_qos.reliability().kind);

  qos.set_depth(-1);
  EXPECT_FALSE(filler.FillInSubQos("channel", qos, &sub_qos));
  EXPECT_FALSE(filler.FillInReaderQos("channel", qos, &reader_qos));
}

TEST(UnderlayMessageTest, underlay_message_test) {
  UnderlayMessage message;
  message.timestamp(1024);
  uint64_t& t = message.timestamp();
  t = 256;
  EXPECT_EQ(256, message.timestamp());

  message.seq(1024);
  uint64_t& seq = message.seq();
  seq = 256;
  EXPECT_EQ(256, message.seq());

  message.data("data");
  std::string& data = message.data();
  data = "data string";
  EXPECT_EQ(data, message.data());
  message.data(std::forward<std::string>("data forward"));
  EXPECT_EQ("data forward", message.data());

  const UnderlayMessage const_message(message);
  std::string data1 = const_message.data();
  EXPECT_EQ(256, const_message.timestamp());
  EXPECT_EQ(256, const_message.seq());
  EXPECT_EQ("data forward", const_message.data());

  UnderlayMessage message2;
  message2 = message;
  EXPECT_EQ(256, message2.timestamp());
  EXPECT_EQ(256, message2.seq());
  EXPECT_EQ("data forward", message2.data());

  UnderlayMessage message3;
  message3 = std::forward<UnderlayMessage>(message2);
  EXPECT_EQ(256, message3.timestamp());
  EXPECT_EQ(256, message3.seq());
  EXPECT_EQ("data forward", message3.data());

  UnderlayMessage message4(message3);
  EXPECT_EQ(256, message4.timestamp());
  EXPECT_EQ(256, message4.seq());
  EXPECT_EQ("data forward", message4.data());

  UnderlayMessage message5(std::forward<UnderlayMessage>(message4));
  EXPECT_EQ(256, message5.timestamp());
  EXPECT_EQ(256, message5.seq());
  EXPECT_EQ("data forward", message5.data());

  EXPECT_EQ("", message4.data());
}

}  // namespace transport
}  // namespace cyber
}  // namespace apollo
