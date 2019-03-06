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

#include "modules/dreamview/backend/data_collection_monitor/data_collection_monitor.h"

#include <google/protobuf/descriptor.h>
#include <google/protobuf/message.h>

#include "cyber/common/file.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/dreamview/backend/common/dreamview_gflags.h"

namespace apollo {
namespace dreamview {

using apollo::canbus::Chassis;
using Json = nlohmann::json;

namespace {

bool GetProtobufFloatByFieldName(const google::protobuf::Message& message,
                                 const google::protobuf::Descriptor* descriptor,
                                 const google::protobuf::Reflection* reflection,
                                 const std::string& field_name, float* value) {
  if (!descriptor) {
    AERROR << "Protobuf descriptor not found";
    return false;
  }

  const auto* field_descriptor = descriptor->FindFieldByName(field_name);
  if (!field_descriptor) {
    AERROR << field_name << " not found in " << descriptor->name();
    return false;
  }

  if (field_descriptor->is_repeated() ||
      field_descriptor->cpp_type() !=
          google::protobuf::FieldDescriptor::CppType::CPPTYPE_FLOAT) {
    AWARN << field_name << " has unsupported conversion type: "
          << field_descriptor->cpp_type();
    return false;
  }

  *value = reflection->GetFloat(message, field_descriptor);

  return true;
}

bool IsCompliedWithCriterion(float actual_value,
                             const std::string& comparison_operator,
                             float target_value) {
  if (comparison_operator == "==") {
    return (actual_value == target_value);
  } else if (comparison_operator == ">") {
    return (actual_value > target_value);
  } else if (comparison_operator == ">=") {
    return (actual_value >= target_value);
  } else if (comparison_operator == "<") {
    return (actual_value < target_value);
  } else if (comparison_operator == "<=") {
    return (actual_value <= target_value);
  } else if (comparison_operator == "!=") {
    return (actual_value != target_value);
  } else {
    AERROR << " Unsupported comparision operator defined in "
           << comparison_operator;
    return false;
  }
}

}  // namespace

DataCollectionMonitor::DataCollectionMonitor()
    : node_(cyber::CreateNode("data_collection_monitor")) {
  InitReaders();
  LoadConfiguration(FLAGS_data_collection_config_path);
}

DataCollectionMonitor::~DataCollectionMonitor() { Stop(); }

void DataCollectionMonitor::InitReaders() {
  node_->CreateReader<Chassis>(FLAGS_chassis_topic,
                               [this](const std::shared_ptr<Chassis>& chassis) {
                                 this->OnChassis(chassis);
                               });
}

void DataCollectionMonitor::LoadConfiguration(
    const std::string& data_collection_config_path) {
  CHECK(cyber::common::GetProtoFromFile(data_collection_config_path,
                                        &data_collection_table_))
      << "Unable to parse calibration configuration from file "
      << data_collection_config_path;

  for (const auto& iter : data_collection_table_.category()) {
    const std::string& category_name = iter.first;
    category_frame_count_.insert({category_name, 0});
  }

  ADEBUG << "Configuration loaded.";
}

void DataCollectionMonitor::Start() {
  if (!enabled_) {
    current_frame_count_ = 0;
    category_frame_count_.clear();
    current_progress_json_.clear();
    LoadConfiguration(FLAGS_data_collection_config_path);
  }
  enabled_ = true;
}

void DataCollectionMonitor::Stop() { enabled_ = false; }

void DataCollectionMonitor::OnChassis(const std::shared_ptr<Chassis>& chassis) {
  if (!enabled_) {
    return;
  }

  const auto* descriptor = chassis->GetDescriptor();
  const auto* reflection = chassis->GetReflection();
  bool should_increment_total_frame_count = false;
  for (const auto& iter : data_collection_table_.category()) {
    const std::string& category_name = iter.first;
    const Category& category = iter.second;

    // This category is done, skip
    if (category_frame_count_[category_name] >= category.total_frames()) {
      continue;
    }

    bool is_complied_with_criteria = true;
    for (const auto& criterion : category.criterion()) {
      float actual_value;
      if (!GetProtobufFloatByFieldName(*chassis, descriptor, reflection,
                                       criterion.field(), &actual_value)) {
        continue;
      }

      if (!IsCompliedWithCriterion(actual_value,
                                   criterion.comparison_operator(),
                                   criterion.value())) {
        is_complied_with_criteria = false;
        break;
      }
    }

    if (is_complied_with_criteria) {
      category_frame_count_[category_name]++;
      should_increment_total_frame_count = true;
    }
  }

  if (should_increment_total_frame_count) {
    current_frame_count_++;

    UpdateProgressInJson();
  }
}

void DataCollectionMonitor::UpdateProgressInJson() {
  for (const auto& iter : data_collection_table_.category()) {
    const std::string& category_name = iter.first;
    const Category& category = iter.second;
    const double progress_percentage =
        100.0 * static_cast<double>(category_frame_count_[category_name]) /
        static_cast<double>(category.total_frames());
    {
      boost::unique_lock<boost::shared_mutex> writer_lock(mutex_);
      current_progress_json_[category.description()] = progress_percentage;
    }
  }

  const double overall_percentage =
      100.0 * static_cast<double>(current_frame_count_) /
      static_cast<double>(data_collection_table_.total_frames());
  {
    boost::unique_lock<boost::shared_mutex> writer_lock(mutex_);
    current_progress_json_["overall"] = overall_percentage;
  }
}

nlohmann::json DataCollectionMonitor::GetProgressString() {
  boost::unique_lock<boost::shared_mutex> reader_lock(mutex_);
  return current_progress_json_;
}

}  // namespace dreamview
}  // namespace apollo
