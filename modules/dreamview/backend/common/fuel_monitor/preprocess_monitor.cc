/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#include "modules/dreamview/backend/common/fuel_monitor/preprocess_monitor.h"

#include "absl/strings/str_cat.h"
#include "gflags/gflags.h"
#include "google/protobuf/descriptor.h"
#include "google/protobuf/message.h"
#include "google/protobuf/util/json_util.h"

#include "cyber/common/file.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/dreamview/backend/common/dreamview_gflags.h"
#include "modules/dreamview/backend/common/fuel_monitor/fuel_monitor_gflags.h"
#include "modules/dreamview/backend/common/vehicle_manager/vehicle_manager.h"

namespace apollo {
namespace dreamview {

using apollo::common::VehicleConfigHelper;
using cyber::common::PathExists;
using google::protobuf::util::MessageToJsonString;

using Json = nlohmann::json;

PreprocessMonitor::PreprocessMonitor()
    : FuelMonitor(FLAGS_preprocess_monitor_name),
      node_(cyber::CreateNode("progress_monitor")) {
  InitReaders();
  LoadConfiguration();
}

PreprocessMonitor::PreprocessMonitor(const std::string& task_name)
    : FuelMonitor(FLAGS_preprocess_monitor_name),
      task_name_(task_name),
      node_(cyber::CreateNode(task_name + "_progress_monitor")) {
  InitReaders();
  LoadConfiguration();
}

PreprocessMonitor::~PreprocessMonitor() { Stop(); }

void PreprocessMonitor::InitReaders() {
  node_->CreateReader<Progress>(
      FLAGS_progress_topic, [this](const std::shared_ptr<Progress>& progress) {
        this->OnProgress(progress);
      });
}

void PreprocessMonitor::LoadConfiguration() {
  if (!task_name_.empty()) {
    const std::string& vehicle_dir =
        VehicleManager::Instance()->GetVehicleDataPath();
    std::string config_path = absl::StrCat(
        vehicle_dir, "dreamview_conf/", task_name_, "_preprocess_table.pb.txt");
    if (!PathExists(config_path)) {
      AWARN << "No corresponding preprocess table file found in " << vehicle_dir
            << ". Using default one instead.";
      config_path = absl::StrCat("/apollo/modules/dreamview/conf/", task_name_,
                                 "_preprocess_table.pb.txt");
    }

    ACHECK(cyber::common::GetProtoFromFile(config_path, &preprocess_table_))
        << "Unable to parse preprocess configuration from file " << config_path;
  }
  auto* progress = preprocess_table_.mutable_progress();
  progress->set_percentage(0.0);
  progress->set_log_string("Click on the button to start preprocessing");
  progress->set_status(Status::UNKNOWN);

  std::string json_string;
  MessageToJsonString(preprocess_table_, &json_string);
  current_status_json_ = Json::parse(json_string);

  ADEBUG << "Configuration loaded.";
}

void PreprocessMonitor::Start() {
  if (!enabled_) {
    current_status_json_.clear();
    LoadConfiguration();
  }
  enabled_ = true;
}

void PreprocessMonitor::Stop() { enabled_ = false; }

void PreprocessMonitor::OnProgress(const std::shared_ptr<Progress>& progress) {
  if (!enabled_) {
    return;
  }

  std::string json_string;
  MessageToJsonString(*progress, &json_string);
  {
    boost::unique_lock<boost::shared_mutex> writer_lock(mutex_);
    current_status_json_["progress"] = Json::parse(json_string);
  }
}

nlohmann::json PreprocessMonitor::GetProgressAsJson() {
  boost::unique_lock<boost::shared_mutex> reader_lock(mutex_);
  return current_status_json_;
}

}  // namespace dreamview
}  // namespace apollo
