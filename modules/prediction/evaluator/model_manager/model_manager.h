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

#pragma once

#include <string>
#include <vector>
#include <memory>
#include <mutex>
#include <map>
#include <utility>

#include "cyber/cyber.h"
#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/prediction/proto/prediction_conf.pb.h"
#include "modules/prediction/evaluator/model_manager/model/model_base.h"


namespace apollo {
namespace prediction {

class ModelManager {
 public:
  ModelManager();
  ~ModelManager() {}

  /**
   * @brief init model manager add load all defined plugin
   *
   * @return init result, true for success
   */ 
  bool Init();

  /**
   * @brief select the best model
   *
   * @param backend inference backend of model
   * @param evaluator_type the type of evaluator
   * @param obstacle_type the type of obstacle 
   * @return shared ptr of the model
   */ 
  std::shared_ptr<ModelBase> SelectModel(const Model::Backend& backend,
                const ObstacleConf::EvaluatorType& evaluator_type,
                const apollo::perception::PerceptionObstacle::Type&
                  obstacle_type);

 private:
  std::map<std::string, std::vector<std::shared_ptr<ModelBase>>> models_;
  std::string to_string(const Model& model_config);
  std::string to_string(const Model::Backend& backend,
                const ObstacleConf::EvaluatorType& evaluator_type,
                const apollo::perception::PerceptionObstacle::Type&
                  obstacle_type);
  EvaluatorModelConf evaluator_model_conf_;
};

}  // namespace prediction
}  // namespace apollo
