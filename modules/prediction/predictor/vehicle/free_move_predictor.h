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
 * @brief Define lane sequence predictor
 */

#ifndef MODULES_PREDICTION_PREDICTOR_VEHICLE_FREE_MOVE_PREDICTOR_H_
#define MODULES_PREDICTION_PREDICTOR_VEHICLE_FREE_MOVE_PREDICTOR_H_

#include "modules/prediction/predictor/predictor.h"

namespace apollo {
namespace prediction {

class FreeMovePredictor : public Predictor {
 public:
  /**
   * @brief Constructor
   */
  FreeMovePredictor() = default;

  /**
   * @brief Destructor
   */
  virtual ~FreeMovePredictor() = default;
<<<<<<< HEAD

  /**
   * @brief Make prediction
   */
  void Predict() const override;
=======
>>>>>>> Added lane sequence and free move predictor framework (#127)
};

}  // prediction
}  // apollo

#endif  // MODULES_PREDICTION_PREDICTOR_VEHICLE_FREE_MOVE_PREDICTOR_H_