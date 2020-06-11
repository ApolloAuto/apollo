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

#include <memory>
#include <string>
#include <vector>

#include "modules/perception/fusion/base/sensor_data_manager.h"
#include "modules/perception/fusion/lib/interface/base_data_association.h"
#include "modules/perception/fusion/lib/interface/base_fusion_system.h"
#include "modules/perception/fusion/lib/interface/base_gatekeeper.h"
#include "modules/perception/fusion/lib/interface/base_tracker.h"

namespace apollo {
namespace perception {
namespace fusion {

struct FusionParams {
  bool use_lidar = true;
  bool use_radar = true;
  bool use_camera = true;
  std::string tracker_method;
  std::string data_association_method;
  std::string gate_keeper_method;
  std::vector<std::string> prohibition_sensors;
};

class ProbabilisticFusion : public BaseFusionSystem {
 public:
  ProbabilisticFusion();
  ~ProbabilisticFusion();

  ProbabilisticFusion(const ProbabilisticFusion&) = delete;
  ProbabilisticFusion& operator=(const ProbabilisticFusion&) = delete;

  bool Init(const FusionInitOptions& init_options) override;

  bool Fuse(const FusionOptions& options,
            const base::FrameConstPtr& sensor_frame,
            std::vector<base::ObjectPtr>* fused_objects) override;

  std::string Name() const override;

 private:
  bool IsPublishSensor(const base::FrameConstPtr& sensor_frame) const;

  void FuseFrame(const SensorFramePtr& frame);

  void CollectFusedObjects(double timestamp,
                           std::vector<base::ObjectPtr>* fused_objects);

  void FuseForegroundTrack(const SensorFramePtr& frame);
  void FusebackgroundTrack(const SensorFramePtr& frame);

  void RemoveLostTrack();

  void UpdateAssignedTracks(
      const SensorFramePtr& frame,
      const std::vector<TrackMeasurmentPair>& assignments);

  void UpdateUnassignedTracks(const SensorFramePtr& frame,
                              const std::vector<size_t>& unassigned_track_inds);

  void CreateNewTracks(const SensorFramePtr& frame,
                       const std::vector<size_t>& unassigned_obj_inds);

  void CollectObjectsByTrack(double timestamp, const TrackPtr& track,
                             std::vector<base::ObjectPtr>* fused_objects);

  void CollectSensorMeasurementFromObject(
      const SensorObjectConstPtr& object,
      base::SensorObjectMeasurement* measurement);

 private:
  std::mutex data_mutex_;
  std::mutex fuse_mutex_;

  bool started_ = false;

  ScenePtr scenes_ = nullptr;
  std::vector<std::shared_ptr<BaseTracker>> trackers_;  // for foreground

  std::unique_ptr<BaseDataAssociation> matcher_;
  std::unique_ptr<BaseGatekeeper> gate_keeper_;

  FusionParams params_;
};

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
