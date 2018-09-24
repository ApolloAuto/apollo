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
#include "modules/perception/lib/config_manager/config_manager.h"

#include <gflags/gflags.h>
#include <google/protobuf/text_format.h>

#include <utility>

#include "modules/perception/base/log.h"
#include "modules/perception/lib/config_manager/proto/perception_config_schema.pb.h"
#include "modules/perception/lib/io/file_util.h"

namespace apollo {
namespace perception {
namespace lib {

DEFINE_string(config_manager_path, "./conf", "The ModelConfig config paths.");
DEFINE_string(work_root, "", "Project work root direcotry.");
DEFINE_string(adu_data, "/home/caros/adu_data",
              "ADU shared data path, including maps, routings...");

using google::protobuf::TextFormat;
using std::map;
using std::move;
using std::pair;
using std::string;
using std::stringstream;
using std::vector;

ConfigManager::ConfigManager() {
  work_root_ = FLAGS_work_root;
  adu_data_ = FLAGS_adu_data;

  // For start at arbitrary path
  if (work_root_.empty()) {
    work_root_ = get_env("MODULE_PATH");
    if (work_root_.empty()) {
      work_root_ = get_env("CYBERTRON_PATH");
    }
  }
}

bool ConfigManager::Init() {
  MutexLock lock(&mutex_);
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

  string config_module_path =
      FileUtil::GetAbsolutePath(work_root_, FLAGS_config_manager_path);
  LOG_INFO << "WORK_ROOT: " << work_root_
           << " config_root_path: " << config_module_path
           << " ADU_DATA: " << adu_data_;

  std::vector<std::string> model_config_files;
  if (!FileUtil::GetFileList(config_module_path, "config_manager.config",
                             &model_config_files)) {
    LOG_ERROR << "config_root_path : " << config_module_path
              << " get file list error.";
    return false;
  }

  for (auto &model_config_file : model_config_files) {
    std::string content;
    if (!FileUtil::GetFileContent(model_config_file, &content)) {
      LOG_ERROR << "failed to get ConfigManager config path: "
                << model_config_file;
      return false;
    }

    ModelConfigFileListProto file_list_proto;

    if (!TextFormat::ParseFromString(content, &file_list_proto)) {
      LOG_ERROR << "invalid ModelConfigFileListProto file: "
                << FLAGS_config_manager_path;
      return false;
    }

    for (const string &model_config_file :
         file_list_proto.model_config_path()) {
      string abs_path =
          FileUtil::GetAbsolutePath(work_root_, model_config_file);
      string config_content;
      if (!FileUtil::GetFileContent(abs_path, &config_content)) {
        LOG_ERROR << "failed to get file content: " << abs_path;
        return false;
      }

      MultiModelConfigProto multi_model_config_proto;

      if (!TextFormat::ParseFromString(config_content,
                                       &multi_model_config_proto)) {
        LOG_ERROR << "invalid MultiModelConfigProto file: " << abs_path;
        return false;
      }

      for (const ModelConfigProto &model_config_proto :
           multi_model_config_proto.model_configs()) {
        ModelConfig *model_config = new ModelConfig();
        if (!model_config->Reset(model_config_proto)) {
          return false;
        }

        LOG_INFO << "load ModelConfig succ. name: " << model_config->name();

        pair<ModelConfigMapIterator, bool> result =
            model_config_map_.emplace(model_config->name(), model_config);
        if (!result.second) {
          LOG_WARN << "duplicate ModelConfig, name: " << model_config->name();
          return false;
        }
      }
    }
  }

  LOG_INFO << "finish to load ModelConfigs. NumModels: "
           << model_config_map_.size();

  inited_ = true;

  return true;
}

bool ConfigManager::Reset() {
  MutexLock lock(&mutex_);
  inited_ = false;
  return InitInternal();
}

std::string ConfigManager::get_env(const std::string &var_name) {
  char *var = nullptr;
  var = getenv(var_name.c_str());
  if (var == nullptr) {
    return std::string("");
  }
  return std::string(var);
}

bool ConfigManager::GetModelConfig(const string &model_name,
                                   const ModelConfig **model_config) {
  if (!inited_ && !Init()) {
    return false;
  }

  ModelConfigMapConstIterator citer = model_config_map_.find(model_name);

  if (citer == model_config_map_.end()) {
    return false;
  }
  *model_config = citer->second;
  return true;
}

ConfigManager::~ConfigManager() {
  ModelConfigMapIterator iter = model_config_map_.begin();
  for (; iter != model_config_map_.end(); ++iter) {
    delete iter->second;
  }
}

bool ModelConfig::Reset(const ModelConfigProto &proto) {
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

  for (const KeyValueInt &pair : proto.integer_params()) {
    integer_param_map_.emplace(pair.name(), pair.value());
  }

  for (const KeyValueString &pair : proto.string_params()) {
    string_param_map_.emplace(pair.name(), pair.value());
  }

  for (const KeyValueDouble &pair : proto.double_params()) {
    double_param_map_.emplace(pair.name(), pair.value());
  }

  for (const KeyValueFloat &pair : proto.float_params()) {
    float_param_map_.emplace(pair.name(), pair.value());
  }

  for (const KeyValueBool &pair : proto.bool_params()) {
    bool_param_map_.emplace(pair.name(), pair.value());
  }

  for (const KeyValueArrayInt &pair : proto.array_integer_params()) {
    vector<int> values;
    RepeatedToVector(pair.values(), &values);
    array_integer_param_map_.emplace(pair.name(), values);
  }

  for (const KeyValueArrayString &pair : proto.array_string_params()) {
    vector<string> values;
    values.reserve(pair.values_size());
    for (const string &value : pair.values()) {
      values.push_back(value);
    }
    array_string_param_map_.emplace(pair.name(), values);
  }

  for (const KeyValueArrayDouble &pair : proto.array_double_params()) {
    vector<double> values;
    RepeatedToVector(pair.values(), &values);
    array_double_param_map_.emplace(pair.name(), values);
  }

  for (const KeyValueArrayFloat &pair : proto.array_float_params()) {
    vector<float> values;
    RepeatedToVector(pair.values(), &values);
    array_float_param_map_.emplace(pair.name(), values);
  }

  for (const KeyValueArrayBool &pair : proto.array_bool_params()) {
    vector<bool> values;
    RepeatedToVector(pair.values(), &values);
    array_bool_param_map_.emplace(pair.name(), values);
  }

  LOG_INFO << "reset ModelConfig. model_name: " << name_
           << " integer_param_map's size: " << integer_param_map_.size()
           << " string_param_map's size: " << string_param_map_.size()
           << " double_param_map's size: " << double_param_map_.size()
           << " float_param_map's size: " << float_param_map_.size()
           << " bool_param_map's size: " << bool_param_map_.size()
           << " array_integer_param_map's size: "
           << array_integer_param_map_.size()
           << " array_string_param_map's size: "
           << array_string_param_map_.size()
           << " array_double_param_map's size: "
           << array_double_param_map_.size()
           << " array_float_param_map's size: " << array_float_param_map_.size()
           << " array_bool_param_map's size: " << array_bool_param_map_.size();

  return true;
}

}  // namespace lib
}  // namespace perception
}  // namespace apollo
