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

/**
 * @file
 * @brief Defines the predictor factory
 */

#ifndef MODULES_PREDICTION_PREDICTOR_PREDICTOR_FACTORY_H_
#define MODULES_PREDICTION_PREDICTOR_PREDICTOR_FACTORY_H_

#include <memory>

#include "modules/prediction/predictor/predictor.h"
#include "modules/prediction/proto/prediction_conf.pb.h"
#include "modules/common/util/factory.h"
#include "modules/common/macro.h"

/**
 * @namespace apollo::prediction
 * @brief apollo::prediction
 */

namespace apollo {
namespace prediction {

class PredictorFactory
    : public apollo::common::util::Factory<ObstacleConf::Pred,
                                           Predictor> {
 public:
  /**
   * @brief Register predictor
   */
  void RegisterPredictor();

  /**
   * @brief Create a pointer to a predictor
   * @param Predictor name
   * @return A pointer to the given predictor
   */
  std::unique_ptr<Predictor> CreatePredictor(const ObstacleConf::Pred& pred);

 private:
  DECLARE_SINGLETON(PredictorFactory);
};

} // namespace prediction
} // namespace apollo

#endif // MODULES_PREDICTION_PREDICTOR_PREDICTOR_FACTORY_H_