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

// ConfigManager use protobuf text format to manage all your model configs.
//
// CODE SAMPLE:
// you can use such code to access your parameters:
//
//         #include "lib/config_manager/config_manager.h"
//
//         ConfigManager* config_manager = ConfigManager::instance();
//
//         string model_name = "FrameClassifier";
//         const ModelConfig* model_config = NULL;
//         if (!config_manager->GetModelConfig(model_name, &model_config)) {
//            AERROR << "not found model: " << model_name;
//            return false;
//         }
//
//         int int_value = 0;
//         if (!model_config->GetValue("my_param_name", &int_value)) {
//             AERROR << "my_param_name not found."
//             return false;
//         }
//         using int_value....
//
//
// CONFIG FORMAT
//
// First you should define file: conf/config_manager.config,
// you can set the path by gflags
//    --config_manager_path=conf/config_manager.config
//
// file content like as:
// define all model config paths.
// ############################################
//
// model_config_path: "./conf/frame_classifier.config"
// model_config_path: "./conf/track_classifier.config"
//
// ############################################
//
// one line identify one model parameter config path.
// ModelConfig config file like as:
// file: ./conf/frame_classifier.config
// #############################################
//
// model_configs {
//     # FrameClassifier model.
//     name: "FrameClassifier"
//     version: "1.0.0"
//     integer_params {
//         name: "threshold1"
//         value: 1
//     }
//
//     integer_params {
//         name: "threshold2"
//         value: 2
//     }
//
//     string_params {
//         name: "threshold3"
//         value: "str3"
//     }
//
//     double_params {
//         name: "threshold4"
//         value: 4.0
//     }
//
//     array_integer_params {
//         name: "array_p1"
//         values: 1
//         values: 2
//         values: 3
//     }
//
//     array_string_params {
//         name: "array_p2"
//         values: "str1"
//         values: "str2"
//         values: "str3"
//         values: "str4"
//     }
//
//     array_string_params {
//         name: "array_p3"
//         values: "str1"
//         values: "str2"
//         values: "str3"
//         values: "str4"
//     }
//
//     array_double_params {
//         name: "array_p4"
//         values: 1.1
//         values: 1.2
//         values: 1.3
//         values: 1.4
//     }
// }

#ifndef MODULES_PERCEPTION_LIB_CONFIG_MANAGER_H_
#define MODULES_PERCEPTION_LIB_CONFIG_MANAGER_H_

#include <mutex>
#include <sstream>
#include <string>
#include <typeinfo>
#include <unordered_map>
#include <vector>

#include "google/protobuf/message.h"
#include "modules/common/macro.h"

namespace apollo {
namespace perception {

class ModelConfig;
class ModelConfigProto;

class ConfigManager {
 public:
  // thread-safe interface.
  bool Init();

  // thread-safe interface.
  bool Reset();

  const ModelConfig* GetModelConfig(const std::string& model_name);

  size_t NumModels() const { return model_config_map_.size(); }

  const std::string& WorkRoot() const { return work_root_; }

  void SetWorkRoot(const std::string& work_root) { work_root_ = work_root; }

 private:
  ~ConfigManager();

  bool InitInternal();

  typedef std::unordered_map<std::string, ModelConfig*> ModelConfigMap;
  typedef ModelConfigMap::iterator ModelConfigMapIterator;
  typedef ModelConfigMap::const_iterator ModelConfigMapConstIterator;

  // key: model_name
  ModelConfigMap model_config_map_;
  std::mutex mutex_;  // multi-thread init safe.
  bool inited_ = false;
  std::string work_root_;  // ConfigManager work root dir.

  DECLARE_SINGLETON(ConfigManager);
};

class ModelConfig {
 public:
  ModelConfig() {}
  ~ModelConfig() {}

  bool Reset(const ModelConfigProto& proto);

  std::string name() const { return name_; }

  bool GetValue(const std::string& name, int* value) const {
    return GetValueFromMap<int>(name, integer_param_map_, value);
  }

  bool GetValue(const std::string& name, std::string* value) const {
    return GetValueFromMap<std::string>(name, string_param_map_, value);
  }

  bool GetValue(const std::string& name, double* value) const {
    return GetValueFromMap<double>(name, double_param_map_, value);
  }

  bool GetValue(const std::string& name, float* value) const {
    return GetValueFromMap<float>(name, float_param_map_, value);
  }

  bool GetValue(const std::string& name, bool* value) const {
    return GetValueFromMap<bool>(name, bool_param_map_, value);
  }

  bool GetValue(const std::string& name, std::vector<int>* values) const {
    return GetValueFromMap<std::vector<int>>(name, array_integer_param_map_,
                                             values);
  }

  bool GetValue(const std::string& name, std::vector<double>* values) const {
    return GetValueFromMap<std::vector<double>>(name, array_double_param_map_,
                                                values);
  }

  bool GetValue(const std::string& name, std::vector<float>* values) const {
    return GetValueFromMap<std::vector<float>>(name, array_float_param_map_,
                                               values);
  }

  bool GetValue(const std::string& name,
                std::vector<std::string>* values) const {
    return GetValueFromMap<std::vector<std::string>>(
        name, array_string_param_map_, values);
  }

  bool GetValue(const std::string& name, std::vector<bool>* values) const {
    return GetValueFromMap<std::vector<bool>>(name, array_bool_param_map_,
                                              values);
  }

 private:
  template <typename T>
  bool GetValueFromMap(const std::string& name,
                       const std::unordered_map<std::string, T>& container,
                       T* value) const;

  template <typename T>
  void RepeatedToVector(
      const google::protobuf::RepeatedField<T>& repeated_values,
      std::vector<T>* vec_values);

  std::string name_;
  std::string version_;

  typedef std::unordered_map<std::string, int> IntegerParamMap;
  typedef std::unordered_map<std::string, std::string> StringParamMap;
  typedef std::unordered_map<std::string, double> DoubleParamMap;
  typedef std::unordered_map<std::string, float> FloatParamMap;
  typedef std::unordered_map<std::string, bool> BoolParamMap;
  typedef std::unordered_map<std::string, std::vector<int>>
      ArrayIntegerParamMap;
  typedef std::unordered_map<std::string, std::vector<std::string>>
      ArrayStringParamMap;
  typedef std::unordered_map<std::string, std::vector<double>>
      ArrayDoubleParamMap;
  typedef std::unordered_map<std::string, std::vector<float>>
      ArrayFloatParamMap;
  typedef std::unordered_map<std::string, std::vector<bool>> ArrayBoolParamMap;

  IntegerParamMap integer_param_map_;
  StringParamMap string_param_map_;
  DoubleParamMap double_param_map_;
  FloatParamMap float_param_map_;
  BoolParamMap bool_param_map_;
  ArrayIntegerParamMap array_integer_param_map_;
  ArrayStringParamMap array_string_param_map_;
  ArrayDoubleParamMap array_double_param_map_;
  ArrayFloatParamMap array_float_param_map_;
  ArrayBoolParamMap array_bool_param_map_;

  DISALLOW_COPY_AND_ASSIGN(ModelConfig);
};

template <typename T>
bool ModelConfig::GetValueFromMap(
    const std::string& name,
    const std::unordered_map<std::string, T>& container, T* value) const {
  typename std::unordered_map<std::string, T>::const_iterator citer =
      container.find(name);

  if (citer == container.end()) {
    return false;
  }

  *value = citer->second;
  return true;
}

template <typename T>
void ModelConfig::RepeatedToVector(
    const google::protobuf::RepeatedField<T>& repeated_values,
    std::vector<T>* vec_list) {
  vec_list->reserve(repeated_values.size());
  for (T value : repeated_values) {
    vec_list->push_back(value);
  }
}

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_LIB_CONFIG_MANAGER_H_
