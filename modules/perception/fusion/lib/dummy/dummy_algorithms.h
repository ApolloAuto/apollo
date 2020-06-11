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
#pragma once

#include <string>
#include <vector>

#include "modules/perception/fusion/lib/interface/base_data_association.h"
#include "modules/perception/fusion/lib/interface/base_existance_fusion.h"
#include "modules/perception/fusion/lib/interface/base_fusion_system.h"
#include "modules/perception/fusion/lib/interface/base_motion_fusion.h"
#include "modules/perception/fusion/lib/interface/base_shape_fusion.h"
#include "modules/perception/fusion/lib/interface/base_tracker.h"
#include "modules/perception/fusion/lib/interface/base_type_fusion.h"

namespace apollo {
namespace perception {
namespace fusion {

class DummyFusionSystem : public BaseFusionSystem {
 public:
  DummyFusionSystem() = default;
  ~DummyFusionSystem() = default;
  DummyFusionSystem(const DummyFusionSystem&) = delete;
  DummyFusionSystem& operator=(const DummyFusionSystem&) = delete;

  bool Init(const FusionInitOptions& options) override;
  std::string Name() const override { return "DummyFusionSystem"; }

  bool Fuse(const FusionOptions& options,
            const base::FrameConstPtr& sensor_frame,
            std::vector<base::ObjectPtr>* fused_objects) override;
};

class DummyDataAssociation : public BaseDataAssociation {
 public:
  DummyDataAssociation() = default;
  ~DummyDataAssociation() = default;
  DummyDataAssociation(const DummyDataAssociation&) = delete;
  DummyDataAssociation& operator=(const DummyDataAssociation&) = delete;

  bool Init() override;
  bool Associate(const AssociationOptions& options,
                 SensorFramePtr sensor_measurements, ScenePtr scene,
                 AssociationResult* association_result) override;

  std::string Name() const override { return "DummyDataAssociation"; }
};

class DummyTracker : public BaseTracker {
 public:
  DummyTracker() = default;
  ~DummyTracker() = default;
  DummyTracker(const DummyTracker&) = delete;
  DummyTracker& operator=(const DummyTracker&) = delete;

  bool Init(TrackPtr track, SensorObjectPtr measurement) override;

  void UpdateWithMeasurement(const TrackerOptions& options,
                             const SensorObjectPtr measurement,
                             double target_timestamp) override;

  void UpdateWithoutMeasurement(const TrackerOptions& options,
                                const std::string& sensor_id,
                                double measurement_timestamp,
                                double target_timestamp) override;

  std::string Name() const override { return "DummyTracker"; }
};

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
