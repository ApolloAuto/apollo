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

#ifndef MODULES_PERCEPTION_OBSTACLE_FUSION_PROBABILISTIC_FUSION_PROBABILISTIC_FUSION_H_  // NOLINT
#define MODULES_PERCEPTION_OBSTACLE_FUSION_PROBABILISTIC_FUSION_PROBABILISTIC_FUSION_H_  // NOLINT

#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "modules/perception/proto/probabilistic_fusion_config.pb.h"

#include "modules/perception/obstacle/base/object.h"
#include "modules/perception/obstacle/fusion/interface/base_fusion.h"
#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_base_track_object_matcher.h"
#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_sensor_manager.h"
#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_sensor_object.h"
#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_track.h"
#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_track_manager.h"

namespace apollo {
namespace perception {

class ProbabilisticFusion : public BaseFusion {
 public:
  ProbabilisticFusion() = default;
  ~ProbabilisticFusion();

  virtual bool Init();

  // @brief: fuse objects from multi sensors(64-lidar, 16-lidar, radar...)
  // @param [in]: multi sensor objects.
  // @param [out]: fused objects.
  // @return true if fuse successfully, otherwise return false
  virtual bool Fuse(const std::vector<SensorObjects> &multi_sensor_objects,
                    std::vector<std::shared_ptr<Object>> *fused_objects);

  virtual std::string name() const;

 protected:
  void FuseFrame(const PbfSensorFramePtr &frame);

  /**@brief create new tracks for objects not assigned to current tracks*/
  void CreateNewTracks(
      const std::vector<std::shared_ptr<PbfSensorObject>> &sensor_objects,
      const std::vector<int> &unassigned_ids);

  /**@brief update current tracks with matched objects*/
  void UpdateAssignedTracks(
      std::vector<PbfTrackPtr> *tracks,
      const std::vector<std::shared_ptr<PbfSensorObject>> &sensor_objects,
      const std::vector<std::pair<int, int>> &assignments,
      const std::vector<double> &track_objects_dist);

  /**@brief update current tracks which cannot find matched objects*/
  void UpdateUnassignedTracks(std::vector<PbfTrackPtr> *tracks,
                              const std::vector<int> &unassigned_tracks,
                              const std::vector<double> &track_object_dist,
                              const SensorType &sensor_type,
                              const std::string &sensor_id, double timestamp);

  void CollectFusedObjects(double timestamp,
                           std::vector<std::shared_ptr<Object>> *fused_objects);

  void DecomposeFrameObjects(
      const std::vector<std::shared_ptr<PbfSensorObject>> &frame_objects,
      std::vector<std::shared_ptr<PbfSensorObject>> *foreground_objects,
      std::vector<std::shared_ptr<PbfSensorObject>> *background_objects);

  void FuseForegroundObjects(
      std::vector<std::shared_ptr<PbfSensorObject>> *foreground_objects,
      Eigen::Vector3d ref_point, const SensorType &sensor_type,
      const std::string &sensor_id, double timestamp);

 protected:
  /**@brief produce fusion result for PNC only when fusing sensor with
   * publish_sensor_id_*/
  std::string publish_sensor_id_ = "velodyne_64";
  bool started_ = false;
  PbfBaseTrackObjectMatcher *matcher_ = nullptr;
  PbfSensorManager *sensor_manager_ = nullptr;
  PbfTrackManager *track_manager_ = nullptr;
  std::mutex sensor_data_rw_mutex_;
  std::mutex fusion_mutex_;
  bool use_radar_ = true;
  bool use_lidar_ = true;
  bool use_camera_ = true;

  probabilistic_fusion_config::ModelConfigs config_;

 private:
  DISALLOW_COPY_AND_ASSIGN(ProbabilisticFusion);
};

// Register plugin.
REGISTER_FUSION(ProbabilisticFusion);

}  // namespace perception
}  // namespace apollo

// clang-format off
#endif  // MODULES_PERCEPTION_OBSTACLE_FUSION_PROBABILISTIC_FUSION_PROBABILISTIC_FUSION_H_ // NOLINT
// clang-format on
