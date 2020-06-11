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

#include <map>
#include <memory>
#include <string>

#include "gtest/gtest_prod.h"

#include "modules/perception/fusion/base/sensor_object.h"

namespace apollo {
namespace perception {
namespace fusion {

typedef std::map<std::string, SensorObjectPtr> SensorId2ObjectMap;

class Track {
 public:
  Track();
  virtual ~Track() = default;

  Track(const Track&) = delete;
  Track& operator=(const Track&) = delete;

  // static members initialization
  inline static void SetMaxLidarInvisiblePeriod(double period) {
    s_max_lidar_invisible_period_ = period;
  }
  inline static void SetMaxRadarInvisiblePeriod(double period) {
    s_max_radar_invisible_period_ = period;
  }
  inline static void SetMaxCameraInvisiblePeriod(double period) {
    s_max_camera_invisible_period_ = period;
  }

  bool Initialize(SensorObjectPtr obj, bool is_background = false);

  void Reset();

  SensorObjectConstPtr GetSensorObject(const std::string& sensor_id) const;
  SensorObjectConstPtr GetLatestLidarObject() const;
  SensorObjectConstPtr GetLatestRadarObject() const;
  SensorObjectConstPtr GetLatestCameraObject() const;

  inline FusedObjectPtr GetFusedObject() { return fused_object_; }
  inline SensorId2ObjectMap& GetLidarObjects() { return lidar_objects_; }

  inline const SensorId2ObjectMap& GetLidarObjects() const {
    return lidar_objects_;
  }

  inline SensorId2ObjectMap& GetRadarObjects() { return radar_objects_; }

  inline const SensorId2ObjectMap& GetRadarObjects() const {
    return radar_objects_;
  }

  inline SensorId2ObjectMap& GetCameraObjects() { return camera_objects_; }

  inline const SensorId2ObjectMap& GetCameraObjects() const {
    return camera_objects_;
  }

  inline int GetTrackId() const {
    return fused_object_->GetBaseObject()->track_id;
  }

  inline double GetTrackingPeriod() const { return tracking_period_; }

  inline size_t GetTrackedTimes() const { return tracked_times_; }

  inline void AddTrackedTimes() { ++tracked_times_; }

  inline double GetExistanceProb() const { return existance_prob_; }

  inline void SetExistanceProb(double prob) { existance_prob_ = prob; }

  inline double GetToicProb() const { return toic_prob_; }

  inline void SetToicProb(double prob) { toic_prob_ = prob; }
  inline bool IsBackground() const { return is_background_; }

  inline bool IsAlive() const { return is_alive_; }

  bool IsVisible(const std::string& sensor_id) const;
  bool IsLidarVisible() const;
  bool IsRadarVisible() const;
  bool IsCameraVisible() const;

  static size_t GenerateNewTrackId();

  void UpdateWithSensorObject(const SensorObjectPtr& obj);

  void UpdateWithoutSensorObject(const std::string& sensor_id,
                                 double measurement_timestamp);

  std::string DebugString() const;

 protected:
  // update state
  void UpdateSupplementState(const SensorObjectPtr& src_object = nullptr);
  void UpdateUnfusedState(const SensorObjectPtr& src_object);

  SensorObjectConstPtr GetLatestSensorObject(
      const SensorId2ObjectMap& objects) const;
  void UpdateSensorObject(SensorId2ObjectMap* objects,
                          const SensorObjectPtr& obj);
  void UpdateSensorObjectWithoutMeasurement(SensorId2ObjectMap* objects,
                                            const std::string& sensor_id,
                                            double measurement_timestamp,
                                            double max_invisible_period);
  void UpdateSensorObjectWithMeasurement(SensorId2ObjectMap* objects,
                                         const std::string& sensor_id,
                                         double measurement_timestamp,
                                         double max_invisible_period);
  void UpdateWithSensorObjectForBackground(const SensorObjectPtr& obj);
  void UpdateWithoutSensorObjectForBackground(const std::string& sensor_id,
                                              double measurement_timestamp);

 protected:
  SensorId2ObjectMap lidar_objects_;
  SensorId2ObjectMap radar_objects_;
  SensorId2ObjectMap camera_objects_;

  FusedObjectPtr fused_object_ = nullptr;
  double tracking_period_ = 0.0;
  double existance_prob_ = 0.0;
  double toic_prob_ = 0.0;

  bool is_background_ = false;
  bool is_alive_ = true;

  size_t tracked_times_ = 0;

 private:
  FRIEND_TEST(TrackTest, test);

  static size_t s_track_idx_;
  static double s_max_lidar_invisible_period_;
  static double s_max_radar_invisible_period_;
  static double s_max_camera_invisible_period_;
};

typedef std::shared_ptr<Track> TrackPtr;
typedef std::shared_ptr<const Track> TrackConstPtr;

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
