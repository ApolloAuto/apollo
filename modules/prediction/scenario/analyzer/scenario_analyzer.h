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

#ifndef MODULES_PREDICTION_SCENARIO_ANALYZER_SCENARIO_ANALYZER_H_
#define MODULES_PREDICTION_SCENARIO_ANALYZER_SCENARIO_ANALYZER_H_

#include "modules/common/proto/scenario.pb.h"
#include "modules/prediction/proto/scenario_feature.pb.h"

namespace apollo {
namespace prediction {

class ScenarioAnalyzer {
 public:
  ScenarioAnalyzer() = default;

  virtual ~ScenarioAnalyzer() = default;

  void Analyze(const ScenarioFeature& scenario_feature);

  const apollo::common::Scenario& scenario() const;

 private:
  apollo::common::Scenario scenario_;
};

}  // namespace prediction
}  // namespace apollo

#endif  // MODULES_PREDICTION_SCENARIO_ANALYZER_SCENARIO_ANALYZER_H_
