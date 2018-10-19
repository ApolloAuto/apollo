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
#include "modules/data/data_component.h"

#include "cyber/common/log.h"

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/message_util.h"
#include "modules/data/common/data_gflags.h"
#include "modules/data/recorder/record_service.h"

namespace apollo {
namespace data {

bool DataComponent::Init() {
  CHECK(apollo::common::util::GetProtoFromFile(FLAGS_data_conf_file,
                                               &data_conf_))
      << "Unable to load data conf file: " << FLAGS_data_conf_file;

  if (!data_conf_.data_enabled()) {
    AINFO << "data module is not enabled";
    return true;
  }

  if (!RecordService::Init(node_)) {
    AERROR << "Unable to initialize data recorder service";
    return false;
  }

  return true;
}

bool DataComponent::Proc(
    const std::shared_ptr<DataInputCommand> &request) {
  ADEBUG << "Received data input command: "
         << request->DebugString();

  OnDataInputCommand(request);

  return true;
}

// probably do not need with Proc function,
// but just leave it for testing for now
void DataComponent::CreateReader() {
  data_input_cmd_reader_ = node_->CreateReader<DataInputCommand>(
      FLAGS_data_topic,
      std::bind(
          &DataComponent::OnDataInputCommand,
          this,
          std::placeholders::_1));
  CHECK(data_input_cmd_reader_ != nullptr);
}

void DataComponent::OnDataInputCommand(
    const std::shared_ptr<DataInputCommand> &data_input_cmd) {
  ADEBUG << "Received data input command: executing command.";
  if (!data_conf_.data_enabled()) {
    ADEBUG << "Data module is not enabled, quit";
    return;
  }
  if (data_input_cmd->command_type() == DataInputCommand::RECORD) {
    auto client = node_->CreateClient<RecordRequest, RecordResponse>(
        FLAGS_data_record_service_name);
    auto response = client->SendRequest(
        std::make_shared<RecordRequest>(data_input_cmd->record_request()));
    if (!response) {
      AERROR << "call data record service failed";
      return;
    }
    ADEBUG << "call service succeeded, response: " << response->DebugString();
  }
}

}  // namespace data
}  // namespace apollo
