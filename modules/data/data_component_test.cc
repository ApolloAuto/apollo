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

#include "gflags/gflags.h"
#include "gtest/gtest.h"

#include "cyber/cyber.h"
#include "cyber/init.h"

#include "modules/data/proto/data.pb.h"
#include "modules/data/proto/data_conf.pb.h"
#include "modules/data/data_component.h"

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/time/time.h"
#include "modules/common/util/file.h"
#include "modules/common/util/message_util.h"

namespace apollo {
namespace data {

using apollo::common::time::Clock;
using apollo::cyber::Writer;

class DataComponentTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    SetupCyber();
  }

 protected:
  void SetupCyber();
  bool RunDataTest();

 protected:
  bool is_cyber_initialized_ = false;
  cyber::ComponentConfig component_config_;
  std::shared_ptr<Writer<DataInputCommand>> data_writer_;
  std::shared_ptr<DataComponent> data_component_;
};

void DataComponentTest::SetupCyber() {
  if (is_cyber_initialized_) {
    return;
  }

  apollo::cyber::Init("data_test");
  Clock::SetMode(Clock::CYBER);

  component_config_.set_name("data_test");
  component_config_.add_readers();

  std::shared_ptr<apollo::cyber::Node> node(
      apollo::cyber::CreateNode("data_test"));
  data_writer_ = node->CreateWriter<DataInputCommand>(FLAGS_data_topic);

  is_cyber_initialized_ = true;
}

bool DataComponentTest::RunDataTest() {
  data_component_.reset(new DataComponent());
  data_component_->Initialize(component_config_);
  data_component_->CreateReader();
  usleep(1000);

  DataInputCommand data_input_cmd;
  RecordRequest record_request;
  data_input_cmd.set_command_type(DataInputCommand::RECORD);
  record_request.set_record_switch(RecordRequest::START);
  record_request.set_record_all(true);
  data_input_cmd.mutable_record_request()->CopyFrom(record_request);
  data_writer_->Write(std::make_shared<DataInputCommand>(data_input_cmd));

  usleep(20000);

  return true;
}

TEST_F(DataComponentTest, data_input_command) {
  bool run_data_success = RunDataTest();
  EXPECT_TRUE(run_data_success);
}

}  // namespace data
}  // namespace apollo
