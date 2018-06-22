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

#include "modules/perception/onboard/subnode.h"

#include "google/protobuf/text_format.h"
#include "gtest/gtest.h"

#include "modules/perception/onboard/proto/dag_config.pb.h"

#include "modules/common/log.h"
#include "modules/common/util/file.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/onboard/event_manager.h"
#include "modules/perception/onboard/shared_data.h"
#include "modules/perception/onboard/shared_data_manager.h"

namespace apollo {
namespace perception {

using apollo::common::Status;
using google::protobuf::TextFormat;

class MySubnode : public Subnode {
 public:
  MySubnode() : Subnode() {}
  virtual ~MySubnode() {}

  Status ProcEvents() override {
    AINFO << "MySubnode proc event.";
    return Status::OK();
  }
};

TEST(SubnodeTest, test_init) {
  FLAGS_work_root = "modules/perception";
  FLAGS_config_manager_path = "./conf/config_manager.config";
  std::string dag_config_path =
      FLAGS_work_root + "/data/onboard_test/dag_streaming.config";
  std::string content;
  DAGConfig dag_config;
  ASSERT_TRUE(apollo::common::util::GetContent(dag_config_path, &content));
  ASSERT_TRUE(TextFormat::ParseFromString(content, &dag_config));
  EventManager event_manager;
  ASSERT_TRUE(event_manager.Init(dag_config.edge_config()));

  MySubnode my_subnode;

  std::vector<EventID> sub_events;
  sub_events.push_back(1008);
  sub_events.push_back(1009);

  std::vector<EventID> pub_events;
  pub_events.push_back(1002);
  pub_events.push_back(1004);

  SharedDataManager shared_data_manager;

  EXPECT_TRUE(my_subnode.Init(dag_config.subnode_config().subnodes(0),
                              sub_events, pub_events, &event_manager,
                              &shared_data_manager));

  AINFO << my_subnode.DebugString();

  EXPECT_EQ(my_subnode.id(), 1);
  EXPECT_EQ(my_subnode.name(), "Lidar64InputNode");
  EXPECT_EQ(my_subnode.reserve(), "topic_64");
}

}  // namespace perception
}  // namespace apollo
