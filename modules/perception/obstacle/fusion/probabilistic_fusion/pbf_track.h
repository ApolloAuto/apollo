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

#ifndef MODULES_PERCEPTION_OBSTACLE_FUSION_PROBABILISTIC_FUSION_PBF_TRACK_H_
#define MODULES_PERCEPTION_OBSTACLE_FUSION_PROBABILISTIC_FUSION_PBF_TRACK_H_

#include <map>
#include <memory>
#include <string>

#include "gtest/gtest.h"
#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_base_motion_fusion.h"
#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_sensor_object.h"

namespace apollo {
namespace perception {

class PbfTrack {
 public:
  explicit PbfTrack(std::shared_ptr<PbfSensorObject> obj);

  ~PbfTrack();

  /**@brief Update track with sensor object */
  void UpdateWithSensorObject(std::shared_ptr<PbfSensorObject> obj,
                              double match_dist);

  void UpdateWithoutSensorObject(const SensorType &sensor_type,
                                 const std::string &sensor_id,
                                 double min_match_dist, double timestamp);

  std::shared_ptr<PbfSensorObject> GetFusedObject();

  double GetFusedTimestamp() const;

  std::shared_ptr<PbfSensorObject> GetLidarObject(const std::string &sensor_id);

  std::shared_ptr<PbfSensorObject> GetRadarObject(const std::string &sensor_id);

  std::shared_ptr<PbfSensorObject> GetCameraObject(
      const std::string &sensor_id);

  std::shared_ptr<PbfSensorObject> GetSensorObject(
      const SensorType &sensor_type, const std::string &sensor_id);

  /**@brief get latest lidar measurement for multi lidar sensors*/
  std::shared_ptr<PbfSensorObject> GetLatestLidarObject();
  /**@brief get latest lidar measurement for multi radar sensors*/
  std::shared_ptr<PbfSensorObject> GetLatestRadarObject();
  /**@brief get latest camera measurement for multi camera sensors*/
  std::shared_ptr<PbfSensorObject> GetLatestCameraObject();

  int GetTrackId() const;

  double GetTrackingPeriod() const { return tracking_period_; }

  inline bool IsDead() const { return is_dead_; }
  bool AbleToPublish();

  static int GetNextTrackId();

  static void SetMaxLidarInvisiblePeriod(double period) {
    s_max_lidar_invisible_period_ = period;
  }

  static double GetMaxLidarInvisiblePeriod() {
    return s_max_lidar_invisible_period_;
  }

  static void SetMaxCameraInvisiblePeriod(double period) {
    s_max_camera_invisible_period_ = period;
  }

  static double GetMaxCameraInvisiblePeriod() {
    return s_max_camera_invisible_period_;
  }

  static void SetMaxRadarInvisiblePeriod(double period) {
    s_max_radar_invisible_period_ = period;
  }

  static double GetMaxRadarInvisiblePeriod() {
    return s_max_radar_confident_angle_;
  }

  static void SetMaxRadarConfidentAngle(double angle) {
    s_max_radar_confident_angle_ = angle;
  }

  static double GetMaxRadarConfidentAngle() {
    return s_max_radar_confident_angle_;
  }

  static void SetMinRadarConfidentDistance(double dist) {
    s_min_radar_confident_distance_ = dist;
  }

  static void SetPublishIfHasLidar(bool enabled) {
    s_publish_if_has_lidar_ = enabled;
  }
  static void SetPublishIfHasRadar(bool enabled) {
    s_publish_if_has_radar_ = enabled;
  }

  static void SetMotionFusionMethod(const std::string &motion_fusion_method);

 protected:
  /**@brief use obj's velocity to update obj's location to input timestamp*/
  void PerformMotionCompensation(std::shared_ptr<PbfSensorObject> obj,
                                 double timestamp);

  void PerformMotionFusion(std::shared_ptr<PbfSensorObject> obj);

  void PerformMotionFusionAsync(std::shared_ptr<PbfSensorObject> obj);

  void UpdateMeasurementsLifeWithMeasurement(
      std::map<std::string, std::shared_ptr<PbfSensorObject>> *objects,
      const std::string &sensor_id, double timestamp,
      double max_invisible_time);

  void UpdateMeasurementsLifeWithoutMeasurement(
      std::map<std::string, std::shared_ptr<PbfSensorObject>> *objects,
      const std::string &sensor_id, double timestamp, double max_invisible_time,
      bool *invisible_state);

 protected:
  std::shared_ptr<PbfSensorObject> fused_object_;

  /**@brief time stamp of the track*/
  double fused_timestamp_;

  int age_;
  double tracking_period_;

  /**@brief global track id*/
  int idx_;
  double invisible_period_;
  bool invisible_in_lidar_;
  bool invisible_in_radar_;
  bool invisible_in_camera_;

  /**@brief motion fusion*/
  std::shared_ptr<PbfBaseMotionFusion> motion_fusion_;

  /**@brief one object instance per sensor, might be more later*/
  std::map<std::string, std::shared_ptr<PbfSensorObject>> lidar_objects_;
  std::map<std::string, std::shared_ptr<PbfSensorObject>> radar_objects_;
  std::map<std::string, std::shared_ptr<PbfSensorObject>> camera_objects_;

  bool is_dead_;

 private:
  PbfTrack();

 private:
  static int s_track_idx_;
  // invisible period for different sensors
  static double s_max_lidar_invisible_period_;
  static double s_max_radar_invisible_period_;
  static double s_max_camera_invisible_period_;

  // radar confidant regions
  static double s_max_radar_confident_angle_;
  static double s_min_radar_confident_distance_;

  enum class MotionFusionMethod {
    PBF_KALMAN = 1,
    PBF_IMF = 2,
    UNKNOWN = 3,
  };
  static MotionFusionMethod s_motion_fusion_method_;

  // publish conditions
  static bool s_publish_if_has_lidar_;
  static bool s_publish_if_has_radar_;
  FRIEND_TEST(PbfTrackTest, test_pbf_track_constructor);
  FRIEND_TEST(PbfTrackTest, test_pbf_get_object);
  FRIEND_TEST(PbfTrackTest, test_pbf_update_measurements_life);
};

typedef std::shared_ptr<PbfTrack> PbfTrackPtr;

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_FUSION_PROBABILISTIC_FUSION_PBF_TRACK_H_
