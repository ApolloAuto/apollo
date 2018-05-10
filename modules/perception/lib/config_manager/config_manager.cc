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

#include "modules/perception/lib/config_manager/config_manager.h"

#include <memory>
#include <mutex>
#include <utility>
#include <vector>

#include "google/protobuf/text_format.h"
#include "modules/common/log.h"
#include "modules/common/util/file.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/lib/config_manager/proto/config_schema.pb.h"

namespace apollo {
namespace perception {

using apollo::common::util::GetAbsolutePath;
using apollo::common::util::GetContent;
using google::protobuf::TextFormat;

ConfigManager::ConfigManager() { work_root_ = FLAGS_work_root; }

bool ConfigManager::Init() {
  std::unique_lock<std::mutex> lock(mutex_);
  return InitInternal();
}

bool ConfigManager::InitInternal() {
  if (inited_) {
    return true;
  }
  ModelConfigMapIterator iter = model_config_map_.begin();
  for (; iter != model_config_map_.end(); ++iter) {
    delete iter->second;
  }
  model_config_map_.clear();

  const std::string path =
      GetAbsolutePath(work_root_, FLAGS_config_manager_path);

  AINFO << "WORK_ROOT: " << work_root_ << " config_manager_path: " << path;
  ModelConfigFileListProto file_list_proto;
  if (!apollo::common::util::GetProtoFromASCIIFile(path, &file_list_proto)) {
    AERROR << "failed to parse ConfigManager config: " << path;
    return false;
  }

  for (const std::string& model_config_file :
       file_list_proto.model_config_path()) {
    const std::string abs_path = GetAbsolutePath(work_root_, model_config_file);

    std::string config_content;
    if (!GetContent(abs_path, &config_content)) {
      AERROR << "failed to get_file_content: " << abs_path;
      return false;
    }

    MultiModelConfigProto multi_model_config_proto;

    if (!TextFormat::ParseFromString(config_content,
                                     &multi_model_config_proto)) {
      AERROR << "invalid MultiModelConfigProto file: " << abs_path;
      return false;
    }

    for (const ModelConfigProto& model_config_proto :
         multi_model_config_proto.model_configs()) {
      ModelConfig* model_config = new ModelConfig();
      if (!model_config->Reset(model_config_proto)) {
        delete model_config;
        return false;
      }

      AINFO << "load ModelConfig succ. name: " << model_config->name();

      std::pair<ModelConfigMapIterator, bool> result =
          model_config_map_.emplace(model_config->name(), model_config);
      if (!result.second) {
        AWARN << "duplicate ModelConfig, name: " << model_config->name();
        return false;
      }
    }
  }

  AINFO << "finish to load ModelConfigs. num_models: "
        << model_config_map_.size();

  inited_ = true;

  return true;
}

bool ConfigManager::Reset() {
  std::unique_lock<std::mutex> lock(mutex_);
  inited_ = false;
  return InitInternal();
}

const ModelConfig* ConfigManager::GetModelConfig(
    const std::string& model_name) {
  if (!inited_ && !Init()) {
    AERROR << "ConfigManager is not initiated.";
    return nullptr;
  }

  ModelConfigMapConstIterator citer = model_config_map_.find(model_name);

  if (citer == model_config_map_.end()) {
    return nullptr;
  }
  return citer->second;
}

ConfigManager::~ConfigManager() {
  ModelConfigMapIterator iter = model_config_map_.begin();
  for (; iter != model_config_map_.end(); ++iter) {
    delete iter->second;
  }
}

bool ModelConfig::Reset(const ModelConfigProto& proto) {
  name_ = proto.name();
  version_ = proto.version();

  integer_param_map_.clear();
  string_param_map_.clear();
  double_param_map_.clear();
  float_param_map_.clear();
  bool_param_map_.clear();
  array_integer_param_map_.clear();
  array_string_param_map_.clear();
  array_double_param_map_.clear();
  array_float_param_map_.clear();
  array_bool_param_map_.clear();

  for (const KeyValueInt& pair : proto.integer_params()) {
    integer_param_map_.emplace(pair.name(), pair.value());
  }

  for (const KeyValueString& pair : proto.string_params()) {
    string_param_map_.emplace(pair.name(), pair.value());
  }

  for (const KeyValueDouble& pair : proto.double_params()) {
    double_param_map_.emplace(pair.name(), pair.value());
  }

  for (const KeyValueFloat& pair : proto.float_params()) {
    float_param_map_.emplace(pair.name(), pair.value());
  }

  for (const KeyValueBool& pair : proto.bool_params()) {
    bool_param_map_.emplace(pair.name(), pair.value());
  }

  for (const KeyValueArrayInt& pair : proto.array_integer_params()) {
    std::vector<int> values;
    RepeatedToVector(pair.values(), &values);
    array_integer_param_map_.emplace(pair.name(), values);
  }

  for (const KeyValueArrayString& pair : proto.array_string_params()) {
    std::vector<std::string> values;
    values.reserve(pair.values_size());
    for (const std::string& value : pair.values()) {
      values.push_back(value);
    }
    array_string_param_map_.emplace(pair.name(), values);
  }

  for (const KeyValueArrayDouble& pair : proto.array_double_params()) {
    std::vector<double> values;
    RepeatedToVector(pair.values(), &values);
    array_double_param_map_.emplace(pair.name(), values);
  }

  for (const KeyValueArrayFloat& pair : proto.array_float_params()) {
    std::vector<float> values;
    RepeatedToVector(pair.values(), &values);
    array_float_param_map_.emplace(pair.name(), values);
  }

  for (const KeyValueArrayBool& pair : proto.array_bool_params()) {
    std::vector<bool> values;
    RepeatedToVector(pair.values(), &values);
    array_bool_param_map_.emplace(pair.name(), values);
  }

  AINFO << "reset ModelConfig. model_name: " << name_
        << " integer_param_map's size: " << integer_param_map_.size()
        << " string_param_map's size: " << string_param_map_.size()
        << " double_param_map's size: " << double_param_map_.size()
        << " float_param_map's size: " << float_param_map_.size()
        << " bool_param_map's size: " << bool_param_map_.size()
        << " array_integer_param_map's size: "
        << array_integer_param_map_.size()
        << " array_string_param_map's size: " << array_string_param_map_.size()
        << " array_double_param_map's size: " << array_double_param_map_.size()
        << " array_float_param_map's size: " << array_float_param_map_.size()
        << " array_bool_param_map's size: " << array_bool_param_map_.size();

  return true;
}

}  // namespace perception
}  // namespace apollo
