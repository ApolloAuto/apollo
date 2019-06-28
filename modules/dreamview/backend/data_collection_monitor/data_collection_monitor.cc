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
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/dreamview/backend/common/dreamview_gflags.h"

namespace apollo {
namespace dreamview {

using apollo::canbus::Chassis;
using apollo::common::VehicleConfigHelper;
using google::protobuf::FieldDescriptor;
using Json = nlohmann::json;
namespace {

/*
 * For simplicity, we are converting all supporting cpp types to float.
 * It is okay to lost precision since this conversion is used
 * for criteria comparison.
 */
bool GetProtobufFloatByFieldName(const google::protobuf::Message& message,
                                 const google::protobuf::Descriptor* descriptor,
                                 const google::protobuf::Reflection* reflection,
                                 const std::string& field_name, float* value) {
  if (!descriptor) {
    AERROR << "Protobuf descriptor not found";
    return false;
  }

  const auto* field_descriptor = descriptor->FindFieldByName(field_name);
  const auto cpp_type = field_descriptor->cpp_type();
  if (cpp_type == FieldDescriptor::CppType::CPPTYPE_FLOAT) {
    *value = reflection->GetFloat(message, field_descriptor);
  } else if (cpp_type == FieldDescriptor::CppType::CPPTYPE_DOUBLE) {
    double value_in_double = reflection->GetDouble(message, field_descriptor);
    *value = static_cast<float>(value_in_double);
  } else if (cpp_type == FieldDescriptor::CppType::CPPTYPE_ENUM) {
    int value_in_int = reflection->GetEnumValue(message, field_descriptor);
    *value = static_cast<float>(value_in_int);
  } else {
    AERROR << field_name << " has unsupported conversion type: "
           << field_descriptor->cpp_type();
    return false;
  }

  return true;
}

bool IsCompliedWithCriterion(float actual_value,
                             const ComparisonOperator& comparison_operator,
                             float target_value) {
  switch (comparison_operator) {
    case ComparisonOperator::EQUAL:
      return std::fabs(actual_value - target_value) <
             common::math::kMathEpsilon;
    case ComparisonOperator::GREATER_THAN:
      return (actual_value > target_value);
    case ComparisonOperator::GREATER_THAN_OR_EQUAL:
      return (actual_value >= target_value);
    case ComparisonOperator::LESS_THAN:
      return (actual_value < target_value);
    case ComparisonOperator::LESS_THAN_OR_EQUAL:
      return (actual_value <= target_value);
    case ComparisonOperator::NOT_EQUAL:
      return (actual_value != target_value);
    default:
      AERROR << "Unsupported comparison operator found:" << comparison_operator;
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
      << "Unable to parse data collection configuration from file "
      << data_collection_config_path;

  for (const auto& scenario_iter : data_collection_table_.scenario()) {
    const std::string& scenario_name = scenario_iter.first;
    const Scenario& scenario = scenario_iter.second;

    for (const auto& category_iter : scenario.category()) {
      const std::string& category_name = category_iter.first;
      const Category& category = category_iter.second;

      category_consecutive_frame_count_[scenario_name][category_name] = 0.0;
      category_frame_count_[scenario_name][category_name] = 0.0;
      current_progress_json_[scenario_name][category.description()] = 0.0;
    }
  }

  ADEBUG << "Configuration loaded.";
}

void DataCollectionMonitor::Start() {
  if (!enabled_) {
    category_consecutive_frame_count_.clear();
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

  const size_t frame_threshold = data_collection_table_.frame_threshold();
  for (const auto& scenario_iter : data_collection_table_.scenario()) {
    const std::string& scenario_name = scenario_iter.first;
    const Scenario& scenario = scenario_iter.second;

    for (const auto& category_iter : scenario.category()) {
      const std::string& category_name = category_iter.first;
      const Category& category = category_iter.second;

      // This category is done, skip
      if (category_frame_count_[scenario_name][category_name] >=
          category.total_frames()) {
        continue;
      }

      if (!IsCompliedWithCriteria(chassis, category)) {
        category_consecutive_frame_count_[scenario_name][category_name] = 0;
        continue;
      }

      // Increment frame count only if consecutive count exceeds the threshold
      const size_t consecutive_count =
          ++category_consecutive_frame_count_[scenario_name][category_name];
      if (consecutive_count == frame_threshold) {
        category_frame_count_[scenario_name][category_name] +=
            consecutive_count;
      } else if (consecutive_count >= frame_threshold) {
        category_frame_count_[scenario_name][category_name] += 1;
      }

      // Update category progress
      const double progress_percentage =
          100.0 *
          static_cast<double>(
              category_frame_count_[scenario_name][category_name]) /
          static_cast<double>(category.total_frames());
      {
        boost::unique_lock<boost::shared_mutex> writer_lock(mutex_);
        current_progress_json_[scenario_name][category.description()] =
            progress_percentage;
      }
    }
  }
}

bool DataCollectionMonitor::IsCompliedWithCriteria(
    const std::shared_ptr<Chassis>& chassis, const Category& category) {
  const auto& vehicle_param = VehicleConfigHelper::GetConfig().vehicle_param();
  const auto* vehicle_param_descriptor = vehicle_param.GetDescriptor();
  const auto* vehicle_param_reflection = vehicle_param.GetReflection();

  const auto* chassis_descriptor = chassis->GetDescriptor();
  const auto* chassis_reflection = chassis->GetReflection();

  for (const auto& criterion : category.criterion()) {
    float target_value;
    if (criterion.has_value()) {
      target_value = criterion.value();
    } else if (!GetProtobufFloatByFieldName(
                   vehicle_param, vehicle_param_descriptor,
                   vehicle_param_reflection, criterion.vehicle_config(),
                   &target_value)) {
      return false;
    }

    float actual_value;
    if (!GetProtobufFloatByFieldName(*chassis, chassis_descriptor,
                                     chassis_reflection, criterion.field(),
                                     &actual_value)) {
      return false;
    }

    if (!IsCompliedWithCriterion(actual_value, criterion.comparison_operator(),
                                 target_value)) {
      return false;
    }
  }

  return true;
}

nlohmann::json DataCollectionMonitor::GetProgressAsJson() {
  boost::unique_lock<boost::shared_mutex> reader_lock(mutex_);
  return current_progress_json_;
}

}  // namespace dreamview
}  // namespace apollo
