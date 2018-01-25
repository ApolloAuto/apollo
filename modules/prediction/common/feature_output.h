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
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
 *implied. See the License for the specific language governing
 *permissions and limitations under the License.
 *****************************************************************************/

#ifndef MODULES_PREDICTION_COMMON_FEATURE_OUTPUT_H_
#define MODULES_PREDICTION_COMMON_FEATURE_OUTPUT_H_

#include "modules/common/macro.h"
#include "modules/prediction/proto/feature.pb.h"
#include "modules/prediction/proto/offline_features.pb.h"

namespace apollo {
namespace prediction {

class FeatureOutput {
 public:
  /**
   * @brief Destructor
   */
  ~FeatureOutput();

  /**
   * @brief Open the output stream
   * @return True if the output stream is open
   */
  bool Open();

  /**
   * @brief Close the output stream
   */
  void Close();

  /**
   * @brief Check if output is ready
   * @return True if output is ready
   */
  bool Ready();

  /**
   * @brief Insert a feature
   * @param A feature in proto
   */
  void Insert(const Feature& feature);

  /**
   * @brief Write features to a file
   */
  void Write();

 private:
  Features features_;
  size_t count_ = 0;
  DECLARE_SINGLETON(FeatureOutput);
};

}  // namespace prediction
}  // namespace apollo

#endif  // MODULES_PREDICTION_COMMON_FEATURE_OUTPUT_H_
