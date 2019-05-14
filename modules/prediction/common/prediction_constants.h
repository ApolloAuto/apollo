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

/**
 * @file
 * @brief Holds all global constants for the prediction module.
 */

#pragma once

namespace apollo {
namespace prediction {

class PredictionConstants {
 public:
  static const int kOnlineMode = 0;
  static const int kDumpFeatureProto = 1;
  static const int kDumpDataForLearning = 2;
  static const int kDumpPredictionResult = 3;
  static const int kDumpFrameEnv = 4;
  static const int kDumpDataForTuning = 5;
};

}  // namespace prediction
}  // namespace apollo
