/******************************************************************************
 * Copyright 2022 The Apollo Authors. All Rights Reserved.
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
#include <memory>
#include <vector>

#include "modules/perception/fusion/lib/interface/base_gatekeeper.h"
#include "modules/perception/pipeline/stage.h"

namespace apollo {
namespace perception {
namespace fusion {

class CollectFusedObject : public pipeline::Stage {
 public:
  using StageConfig = pipeline::StageConfig;
  using DataFrame = pipeline::DataFrame;
  using PluginType = pipeline::PluginType;

 public:
  CollectFusedObject() = default;
  ~CollectFusedObject() = default;

  bool Init(const StageConfig& stage_config) override;

  bool Process(DataFrame* data_frame) override;

  bool IsEnabled() const override { return enable_; }

  std::string Name() const override { return name_; }

 private:
  void Process(double timestamp, std::vector<base::ObjectPtr>* fused_objects);

  void CollectObjectsByTrack(
      double timestamp,
      const TrackPtr& track,
      std::vector<base::ObjectPtr>* fused_objects);

  void CollectSensorMeasurementFromObject(
      const SensorObjectConstPtr& object,
      base::SensorObjectMeasurement* measurement);

 private:
  std::unique_ptr<BaseGatekeeper> gate_keeper_;
  ScenePtr scenes_;
};

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
