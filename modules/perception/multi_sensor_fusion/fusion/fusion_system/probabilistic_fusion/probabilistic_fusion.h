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

#include "cyber/common/macros.h"
#include "modules/perception/multi_sensor_fusion/base/sensor_data_manager.h"
#include "modules/perception/multi_sensor_fusion/interface/base_data_association.h"
#include "modules/perception/multi_sensor_fusion/interface/base_fusion_system.h"
#include "modules/perception/multi_sensor_fusion/interface/base_gatekeeper.h"
#include "modules/perception/multi_sensor_fusion/interface/base_tracker.h"

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
  ProbabilisticFusion() = default;
  ~ProbabilisticFusion() = default;

  /**
   * @brief Initialization
   *
   * @param options
   * @return true
   * @return false
   */
  bool Init(const FusionInitOptions& options) override;

  /**
   * @brief Probabilistic fusion of sensor data, the input is the detection
   * results of different sensors, and the output is the result after fusion
   *
   * @param sensor_frame sensor data
   * @param fused_objects objects after fusion
   * @return true
   * @return false
   */
  bool Fuse(const base::FrameConstPtr& sensor_frame,
            std::vector<base::ObjectPtr>* fused_objects) override;

  std::string Name() const override { return "ProbabilisticFusion"; }

 private:
  /**
   * @brief Determine whether to send message, if it's the message from
   * the main sensor, it will be publish
   *
   * @param sensor_frame
   * @return true
   * @return false
   */
  bool IsPublishSensor(const base::FrameConstPtr& sensor_frame) const;

  /**
   * @brief Fusion SensorFrame
   *
   * @param frame
   */
  void FuseFrame(const SensorFramePtr& frame);

  /**
   * @brief Collect the fused objects
   *
   * @param timestamp
   * @param fused_objects
   */
  void CollectFusedObjects(double timestamp,
                           std::vector<base::ObjectPtr>* fused_objects);

  /**
   * @brief Fuse foreground track
   *
   * @param frame
   */
  void FuseForegroundTrack(const SensorFramePtr& frame);

  /**
   * @brief Fuse background track
   *
   * @param frame
   */
  void FusebackgroundTrack(const SensorFramePtr& frame);

  /// @brief delete lost tracker
  void RemoveLostTrack();

  /**
   * @brief Update assigned tracks
   *
   * @param frame
   * @param assignments
   */
  void UpdateAssignedTracks(
      const SensorFramePtr& frame,
      const std::vector<TrackMeasurmentPair>& assignments);

  /**
   * @brief Update unassigned tracks
   *
   * @param frame
   * @param unassigned_track_inds
   */
  void UpdateUnassignedTracks(const SensorFramePtr& frame,
                              const std::vector<size_t>& unassigned_track_inds);

  /**
   * @brief Create new tracks
   *
   * @param frame
   * @param unassigned_obj_inds
   */
  void CreateNewTracks(const SensorFramePtr& frame,
                       const std::vector<size_t>& unassigned_obj_inds);

  /**
   * @brief Collect objects by track
   *
   * @param timestamp
   * @param track
   * @param fused_objects
   */
  void CollectObjectsByTrack(double timestamp, const TrackPtr& track,
                             std::vector<base::ObjectPtr>* fused_objects);

  /**
   * @brief Collect sensor measurement from object
   *
   * @param object
   * @param measurement
   */
  void CollectSensorMeasurementFromObject(
      const SensorObjectConstPtr& object,
      base::SensorObjectMeasurement* measurement);

 private:
  std::mutex data_mutex_;
  std::mutex fuse_mutex_;

  ScenePtr scenes_;
  std::vector<std::shared_ptr<BaseTracker>> trackers_;  // for foreground

  std::unique_ptr<BaseDataAssociation> matcher_;
  std::unique_ptr<BaseGatekeeper> gate_keeper_;

  FusionParams params_;

  DISALLOW_COPY_AND_ASSIGN(ProbabilisticFusion);
};

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
