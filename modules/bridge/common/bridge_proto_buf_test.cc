/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "gtest/gtest.h"

#include "modules/planning/proto/planning.pb.h"

#include "cyber/init.h"
#include "modules/bridge/common/bridge_proto_diserialized_buf.h"
#include "modules/bridge/common/bridge_proto_serialized_buf.h"

namespace apollo {
namespace bridge {

TEST(BridgeProtoBufTest, Simple) {
  cyber::Init("bridge_proto_buf_test");
  BridgeProtoSerializedBuf<planning::ADCTrajectory> proto_buf;

  auto adc_trajectory = std::make_shared<planning::ADCTrajectory>();
  double x = 1.0;
  double y = 1.0;
  for (size_t i = 0; i < 100; ++i) {
    auto *point = adc_trajectory->add_trajectory_point();
    double offset = 0.1 * static_cast<double>(i);
    point->mutable_path_point()->set_x(x + offset);
    point->mutable_path_point()->set_y(y + offset);
  }
  adc_trajectory->mutable_header()->set_sequence_num(123);
  proto_buf.Serialize(adc_trajectory, "planning::ADCTrajectory");

  BridgeProtoDiserializedBuf<planning::ADCTrajectory> proto_recv_buf;

  size_t frame_count = proto_buf.GetSerializedBufCount();
  for (size_t i = 0; i < frame_count; i++) {
    char header_flag[sizeof(BRIDGE_HEADER_FLAG) + 1] = {0};
    bsize offset = 0;
    memcpy(header_flag, proto_buf.GetSerializedBuf(i), HEADER_FLAG_SIZE);
    EXPECT_STREQ(header_flag, BRIDGE_HEADER_FLAG);
    offset += static_cast<bsize>(sizeof(BRIDGE_HEADER_FLAG) + 1);

    char header_size_buf[sizeof(hsize) + 1] = {0};
    const char *cursor = proto_buf.GetSerializedBuf(i) + offset;
    memcpy(header_size_buf, cursor, sizeof(hsize));
    hsize header_size = *(reinterpret_cast<hsize *>(header_size_buf));
    EXPECT_EQ(header_size, 184);
    offset += static_cast<bsize>(sizeof(hsize) + 1);

    BridgeHeader header;
    bsize buf_size = header_size - offset;
    cursor = proto_buf.GetSerializedBuf(i) + offset;
    EXPECT_TRUE(header.Diserialize(cursor, buf_size));
    EXPECT_STREQ(header.GetMsgName().c_str(), "planning::ADCTrajectory");
    EXPECT_EQ(header.GetMsgID(), 123);

    proto_recv_buf.Initialize(header);
    char *buf = proto_recv_buf.GetBuf(header.GetFramePos());
    cursor = proto_buf.GetSerializedBuf(i) + header_size;
    memcpy(buf, cursor, header.GetFrameSize());
    proto_recv_buf.UpdateStatus(header.GetIndex());
    if (i < frame_count - 1) {
      EXPECT_FALSE(proto_recv_buf.IsReadyDiserialize());
    } else {
      EXPECT_TRUE(proto_recv_buf.IsReadyDiserialize());
    }
  }
  auto pb_msg = std::make_shared<planning::ADCTrajectory>();
  proto_recv_buf.Diserialized(pb_msg);
  EXPECT_EQ(pb_msg->header().sequence_num(),
            adc_trajectory->header().sequence_num());
  EXPECT_EQ(pb_msg->trajectory_point_size(),
            adc_trajectory->trajectory_point_size());

  int traj_size = adc_trajectory->trajectory_point_size();
  EXPECT_EQ(traj_size, 100);
  for (int i = 0; i < traj_size; ++i) {
    EXPECT_EQ(adc_trajectory->trajectory_point(i).path_point().x(),
              pb_msg->trajectory_point(i).path_point().x());
    EXPECT_EQ(adc_trajectory->trajectory_point(i).path_point().y(),
              pb_msg->trajectory_point(i).path_point().x());
  }
}

}  // namespace bridge
}  // namespace apollo
