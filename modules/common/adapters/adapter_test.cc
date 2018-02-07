/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/common/adapters/adapter.h"

#include <cmath>
#include <string>
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/localization/proto/localization.pb.h"

namespace apollo {
namespace common {
namespace adapter {

using IntegerAdapter = Adapter<int>;

TEST(AdapterTest, Empty) {
  IntegerAdapter adapter("Integer", "integer_topic", 10);
  EXPECT_TRUE(adapter.Empty());
}

TEST(AdapterTest, Observe) {
  IntegerAdapter adapter("Integer", "integer_topic", 10);
  adapter.OnReceive(173);

  // Before calling Observe.
  EXPECT_TRUE(adapter.Empty());

  // After calling Observe.
  adapter.Observe();
  EXPECT_FALSE(adapter.Empty());
}

TEST(AdapterTest, GetLatestObserved) {
  IntegerAdapter adapter("Integer", "integer_topic", 10);
  adapter.OnReceive(173);

  adapter.Observe();
  EXPECT_FALSE(adapter.Empty());
  EXPECT_EQ(173, adapter.GetLatestObserved());

  adapter.OnReceive(5);
  adapter.OnReceive(7);
  // Before calling Observe() again.
  EXPECT_FALSE(adapter.Empty());
  EXPECT_EQ(173, adapter.GetLatestObserved());
  adapter.Observe();
  // After calling Observe() again.
  EXPECT_FALSE(adapter.Empty());
  EXPECT_EQ(7, adapter.GetLatestObserved());
}

TEST(AdapterTest, History) {
  IntegerAdapter adapter("Integer", "integer_topic", 3);
  adapter.OnReceive(1);
  adapter.OnReceive(2);

  adapter.Observe();
  {
    // Currently the history contains [2, 1].
    std::vector<IntegerAdapter::DataPtr> history(adapter.begin(),
                                                 adapter.end());
    EXPECT_EQ(2, history.size());
    EXPECT_EQ(2, *history[0]);
    EXPECT_EQ(1, *history[1]);
  }

  adapter.OnReceive(0);
  adapter.OnReceive(0);
  adapter.OnReceive(0);
  adapter.OnReceive(0);
  adapter.OnReceive(0);
  adapter.OnReceive(0);
  adapter.OnReceive(0);
  adapter.OnReceive(3);
  adapter.OnReceive(4);
  adapter.OnReceive(5);

  {
    // Although there are more messages, without calling Observe,
    // the history is still [2, 1].
    std::vector<IntegerAdapter::DataPtr> history(adapter.begin(),
                                                 adapter.end());
    EXPECT_EQ(2, history.size());
    EXPECT_EQ(2, *history[0]);
    EXPECT_EQ(1, *history[1]);
  }

  adapter.Observe();
  {
    // After calling Observe(), the history starts from 5. Since we only
    // maintain 3 elements in this adapter, 1 and 2 will be thrown out.
    //
    // History should be 5, 4, 3.
    std::vector<IntegerAdapter::DataPtr> history(adapter.begin(),
                                                 adapter.end());
    EXPECT_EQ(3, history.size());
    EXPECT_EQ(5, *history[0]);
    EXPECT_EQ(4, *history[1]);
    EXPECT_EQ(3, *history[2]);
  }
}

TEST(AdapterTest, Callback) {
  IntegerAdapter adapter("Integer", "integer_topic", 3);

  // Set the callback to act as a counter of messages.
  int count = 0;
  adapter.AddCallback([&count](int x) { count += x; });

  adapter.OnReceive(11);
  adapter.OnReceive(41);
  adapter.OnReceive(31);
  EXPECT_EQ(11 + 41 + 31, count);
}

using MyLocalizationAdapter = Adapter<localization::LocalizationEstimate>;

TEST(AdapterTest, Dump) {
  FLAGS_enable_adapter_dump = true;
  std::string temp_dir = std::getenv("TEST_TMPDIR");

  MyLocalizationAdapter adapter("local", "local_topic", 3, temp_dir);
  localization::LocalizationEstimate loaded;

  localization::LocalizationEstimate msg;

  msg.mutable_header()->set_sequence_num(17);
  adapter.OnReceive(msg);
  apollo::common::util::GetProtoFromASCIIFile(temp_dir + "/local/17.pb.txt",
                                              &loaded);
  EXPECT_EQ(17, loaded.header().sequence_num());

  msg.mutable_header()->set_sequence_num(23);
  adapter.OnReceive(msg);
  apollo::common::util::GetProtoFromASCIIFile(temp_dir + "/local/23.pb.txt",
                                              &loaded);
  EXPECT_EQ(23, loaded.header().sequence_num());
}

}  // namespace adapter
}  // namespace common
}  // namespace apollo
