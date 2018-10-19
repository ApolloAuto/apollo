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
#include "modules/dreamview/backend/hmi/hmi_worker.h"

#include "google/protobuf/util/message_differencer.h"
#include "gtest/gtest.h"

#include "cyber/cyber.h"
#include "modules/common/util/file.h"

DECLARE_string(modes_config_path);

namespace apollo {
namespace dreamview {

TEST(HMIWorkerTest, Init) {
  apollo::cyber::Init();
  std::shared_ptr<apollo::cyber::Node> node(
      apollo::cyber::CreateNode("hmi_worker_tester"));
  HMIWorker worker(node);
  const auto& hmi_config = worker.GetConfig();
  EXPECT_GT(hmi_config.available_vehicles().size(), 0);
}

TEST(HMIWorkerTest, LoadModesConfig) {
  const std::string modes_config_path =
      "/apollo/modules/dreamview/backend/hmi/testdata/modes";
  const std::string expected_config_pb_file =
      "/apollo/modules/dreamview/backend/hmi/testdata/modes/"
      "expected_modes.pb.txt";

  HMIConfig expected_config;
  ASSERT_TRUE(apollo::common::util::GetProtoFromASCIIFile(
      expected_config_pb_file, &expected_config));

  HMIConfig loaded_config;
  EXPECT_TRUE(HMIWorker::LoadModesConfig(modes_config_path, &loaded_config));

  EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equivalent(
      expected_config, loaded_config));
}

}  // namespace dreamview
}  // namespace apollo
