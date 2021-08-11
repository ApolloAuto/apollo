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

#include "cyber/blocker/blocker_manager.h"

#include "gtest/gtest.h"

#include "cyber/proto/unit_test.pb.h"

#include "cyber/blocker/intra_reader.h"
#include "cyber/blocker/intra_writer.h"

namespace apollo {
namespace cyber {
namespace blocker {

using apollo::cyber::proto::UnitTest;

void cb(const std::shared_ptr<UnitTest>& msg_ptr) { UNUSED(msg_ptr); }

TEST(BlockerTest, blocker_manager_test) {
  auto block_mgr = BlockerManager::Instance();
  Blocker<UnitTest>::MessageType msgtype;

  block_mgr->Publish<UnitTest>("ch1", msgtype);
  block_mgr->Subscribe<UnitTest>("ch1", 10, "cb1", cb);
  auto blocker = block_mgr->GetOrCreateBlocker<UnitTest>(BlockerAttr("ch1"));
  EXPECT_NE(blocker, nullptr);
  block_mgr->Unsubscribe<UnitTest>("ch1", "cb1");
  block_mgr->Subscribe<UnitTest>("ch_null", 10, "cb1", cb);
  block_mgr->Observe();
  block_mgr->Reset();
}

TEST(BlockerTest, blocker_intra_writer) {
  proto::RoleAttributes role_attr;
  auto msg_ptr = std::make_shared<UnitTest>();
  UnitTest msg;
  IntraWriter<UnitTest> writer(role_attr);
  EXPECT_FALSE(writer.Write(msg_ptr));

  EXPECT_TRUE(writer.Init());
  EXPECT_TRUE(writer.Init());

  EXPECT_TRUE(writer.Write(msg_ptr));

  writer.Shutdown();
  writer.Shutdown();
}

TEST(BlockerTest, blocker_intra_reader) {
  auto block_mgr = BlockerManager::Instance();
  Blocker<UnitTest>::MessageType msgtype;
  block_mgr->Publish<UnitTest>("ch1", msgtype);

  proto::RoleAttributes role_attr;
  auto msg = std::make_shared<UnitTest>();

  IntraWriter<UnitTest> writer(role_attr);
  writer.Init();
  writer.Write(msg);

  IntraReader<UnitTest> reader(role_attr, cb);
  reader.Init();
  reader.Init();
  reader.SetHistoryDepth(10);
  EXPECT_EQ(10, reader.GetHistoryDepth());
  reader.Observe();
  reader.Begin();
  reader.End();
  EXPECT_TRUE(reader.HasReceived());
  block_mgr->GetOrCreateBlocker<UnitTest>(BlockerAttr("ch1"));
  reader.Observe();
  EXPECT_FALSE(reader.Empty());
  reader.ClearData();
  EXPECT_TRUE(reader.Empty());
  block_mgr->Reset();
  EXPECT_TRUE(reader.Empty());
  EXPECT_FALSE(reader.HasReceived());
  reader.GetLatestObserved();
  reader.GetOldestObserved();
  reader.SetHistoryDepth(10);
  EXPECT_EQ(reader.GetHistoryDepth(), 0);
  reader.ClearData();
  reader.Shutdown();
}

}  // namespace blocker
}  // namespace cyber
}  // namespace apollo
