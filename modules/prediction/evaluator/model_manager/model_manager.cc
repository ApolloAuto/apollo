/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#include "modules/prediction/evaluator/model_manager/model_manager.h"

#include <algorithm>
#include <sstream>

#include "cyber/common/file.h"
#include "modules/prediction/evaluator/warm_up/warm_up.h"
#include "modules/prediction/common/prediction_system_gflags.h"

namespace apollo {
namespace prediction {

ModelManager::ModelManager() {
  PredictionConf prediciton_conf;
  if (!cyber::common::GetProtoFromFile(
      FLAGS_prediction_conf_file, &prediciton_conf)) {
    AERROR << "Unable to load prediction conf file: "
            << FLAGS_prediction_conf_file;
    ACHECK(false);
  }
  evaluator_model_conf_ = prediciton_conf.evaluator_model_conf();
  models_ = std::map<std::string, std::vector<std::shared_ptr<ModelBase>>>{};
}

bool ModelManager::Init() {
  static const std::string class_namespace = "apollo::prediction::";
  std::map<std::string, std::vector<Model>> m;
  for (auto& model_conf : evaluator_model_conf_.model()) {
    auto key = to_string(model_conf);
    if (model_conf.priority() == 0) {
      // priority = 0 means this plugin only applicable to arm arch
      #ifndef __aarch64__
      continue;
      #endif
    }
    if (m.find(key) != m.end()) {
      m[key].push_back(model_conf);
    } else {
      m[key] = std::vector<Model>{model_conf};
    }
  }

  // sort the model based on priority
  for (auto& p : m) {
    sort(m[p.first].begin(), m[p.first].end(),
      [](Model& a, Model& b){return a.priority() < b.priority();});
  }

  for (auto& p : m) {
    auto key = p.first;
    auto& model_conf_vec = p.second;
    models_[key] = std::vector<std::shared_ptr<ModelBase>>{};
    for (auto model_conf : model_conf_vec) {
      std::string class_name = model_conf.type();
      if (class_name.find("::") == std::string::npos) {
        class_name = class_namespace + class_name;
      }
      auto model_ptr =
        apollo::cyber::plugin_manager::PluginManager::Instance()
          ->CreateInstance<ModelBase>(class_name);
      models_[key].push_back(model_ptr);
    }
  }

  return true;
}

std::string ModelManager::to_string(const Model& model_config) {
  std::ostringstream key;
  key << model_config.evaluator_type() << model_config.obstacle_type();
  key << model_config.backend();
  return key.str();
}

std::string ModelManager::to_string(const Model::Backend& backend,
                const ObstacleConf::EvaluatorType& evaluator_type,
                const apollo::perception::PerceptionObstacle::Type&
                  obstacle_type) {
  std::ostringstream key;
  key << evaluator_type << obstacle_type << backend;
  return key.str();
}

std::shared_ptr<ModelBase> ModelManager::SelectModel(
                const Model::Backend& backend,
                const ObstacleConf::EvaluatorType& evaluator_type,
                const apollo::perception::PerceptionObstacle::Type&
                  obstacle_type) {
  auto key = to_string(backend, evaluator_type, obstacle_type);
  if (models_.find(key) == models_.end()) {
    AERROR << "Can't find model with attribute: evaluator_type("
      << evaluator_type << ") " << "obstacle_type(" << obstacle_type
      << ") " << "backend(" << backend << ")";
    ACHECK(false);
  }
  return models_[key][0];
}

}  // namespace prediction
}  // namespace apollo
