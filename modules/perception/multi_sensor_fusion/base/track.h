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

#include "cyber/common/macros.h"
#include "modules/perception/multi_sensor_fusion/base/sensor_object.h"

namespace apollo {
namespace perception {
namespace fusion {

typedef std::map<std::string, SensorObjectPtr> SensorId2ObjectMap;

class Track {
 public:
  Track();
  virtual ~Track() = default;

  /**
   * @brief Set the max lidar invisible period
   *
   * @param period
   */
  inline static void SetMaxLidarInvisiblePeriod(double period) {
    s_max_lidar_invisible_period_ = period;
  }

  /**
   * @brief Set the max radar invisible period
   *
   * @param period
   */
  inline static void SetMaxRadarInvisiblePeriod(double period) {
    s_max_radar_invisible_period_ = period;
  }

  /**
   * @brief Set the max camera invisible period
   *
   * @param period
   */
  inline static void SetMaxCameraInvisiblePeriod(double period) {
    s_max_camera_invisible_period_ = period;
  }

  /**
   * @brief Initialize
   *
   * @param obj
   * @param is_background
   * @return true
   * @return false
   */
  bool Initialize(SensorObjectPtr obj, bool is_background = false);

  void Reset();

  /**
   * @brief Get the sensor object object
   *
   * @param sensor_id
   * @return SensorObjectConstPtr
   */
  SensorObjectConstPtr GetSensorObject(const std::string& sensor_id) const;

  /**
   * @brief Get the latest lidar object object
   *
   * @return SensorObjectConstPtr
   */
  SensorObjectConstPtr GetLatestLidarObject() const;

  /**
   * @brief Get the latest radar object object
   *
   * @return SensorObjectConstPtr
   */
  SensorObjectConstPtr GetLatestRadarObject() const;

  /**
   * @brief Get the latest camera object object
   *
   * @return SensorObjectConstPtr
   */
  SensorObjectConstPtr GetLatestCameraObject() const;

  /**
   * @brief Get the fused object object
   *
   * @return FusedObjectPtr
   */
  inline FusedObjectPtr GetFusedObject() { return fused_object_; }

  /**
   * @brief Get the lidar objects object
   *
   * @return SensorId2ObjectMap&
   */
  inline SensorId2ObjectMap& GetLidarObjects() { return lidar_objects_; }

  /**
   * @brief Get the lidar objects object
   *
   * @return const SensorId2ObjectMap&
   */
  inline const SensorId2ObjectMap& GetLidarObjects() const {
    return lidar_objects_;
  }

  /**
   * @brief Get the radar objects object
   *
   * @return SensorId2ObjectMap&
   */
  inline SensorId2ObjectMap& GetRadarObjects() { return radar_objects_; }

  /**
   * @brief Get the radar objects object
   *
   * @return const SensorId2ObjectMap&
   */
  inline const SensorId2ObjectMap& GetRadarObjects() const {
    return radar_objects_;
  }

  /**
   * @brief Get the camera objects object
   *
   * @return SensorId2ObjectMap&
   */
  inline SensorId2ObjectMap& GetCameraObjects() { return camera_objects_; }

  /**
   * @brief Get the camera objects object
   *
   * @return const SensorId2ObjectMap&
   */
  inline const SensorId2ObjectMap& GetCameraObjects() const {
    return camera_objects_;
  }

  /**
   * @brief Get the track id
   *
   * @return int
   */
  inline int GetTrackId() const {
    return fused_object_->GetBaseObject()->track_id;
  }

  /**
   * @brief Get the tracking period
   *
   * @return double
   */
  inline double GetTrackingPeriod() const { return tracking_period_; }

  /**
   * @brief Get the tracked times
   *
   * @return size_t
   */
  inline size_t GetTrackedTimes() const { return tracked_times_; }

  /**
   * @brief Add the tracked times
   *
   */
  inline void AddTrackedTimes() { ++tracked_times_; }

  /**
   * @brief Get the Existence Prob object
   *
   * @return double
   */
  inline double GetExistenceProb() const { return existence_prob_; }

  /**
   * @brief Set the Existence Prob object
   *
   * @param prob
   */
  inline void SetExistenceProb(double prob) { existence_prob_ = prob; }

  /**
   * @brief Get the Toic Prob object
   *
   * @return double
   */
  inline double GetToicProb() const { return toic_prob_; }

  /**
   * @brief Set the Toic Prob object
   *
   * @param prob
   */
  inline void SetToicProb(double prob) { toic_prob_ = prob; }

  /**
   * @brief Is background
   *
   * @return true
   * @return false
   */
  inline bool IsBackground() const { return is_background_; }

  /**
   * @brief Is tracking lost
   *
   * @return true
   * @return false
   */
  inline bool IsAlive() const { return is_alive_; }

  /**
   * @brief Is visible
   *
   * @param sensor_id
   * @return true
   * @return false
   */
  bool IsVisible(const std::string& sensor_id) const;

  /**
   * @brief Is lidar visible
   *
   * @return true
   * @return false
   */
  bool IsLidarVisible() const;

  /**
   * @brief Is radar visible
   *
   * @return true
   * @return false
   */
  bool IsRadarVisible() const;

  /**
   * @brief Is camera visible
   *
   * @return true
   * @return false
   */
  bool IsCameraVisible() const;

  /**
   * @brief Generate new track id
   *
   * @return size_t
   */
  static size_t GenerateNewTrackId();

  /**
   * @brief Update by SensorObject
   *
   * @param obj
   */
  void UpdateWithSensorObject(const SensorObjectPtr& obj);

  /**
   * @brief Update without SensorObject
   *
   * @param sensor_id
   * @param measurement_timestamp
   */
  void UpdateWithoutSensorObject(const std::string& sensor_id,
                                 double measurement_timestamp);

  /**
   * @brief Debug information
   *
   * @return std::string
   */
  std::string DebugString() const;

 protected:
  /**
   * @brief Update supplement state
   *
   * @param src_object
   */
  void UpdateSupplementState(const SensorObjectPtr& src_object = nullptr);

  /**
   * @brief update no fusion state
   *
   * @param src_object
   */
  void UpdateUnfusedState(const SensorObjectPtr& src_object);

  /**
   * @brief Get the Latest Sensor Object object
   *
   * @param objects
   * @return SensorObjectConstPtr
   */
  SensorObjectConstPtr GetLatestSensorObject(
      const SensorId2ObjectMap& objects) const;

  /**
   * @brief
   *
   * @param objects
   * @param obj
   */
  void UpdateSensorObject(SensorId2ObjectMap* objects,
                          const SensorObjectPtr& obj);

  /**
   * @brief
   *
   * @param objects
   * @param sensor_id
   * @param measurement_timestamp
   * @param max_invisible_period
   */
  void UpdateSensorObjectWithoutMeasurement(SensorId2ObjectMap* objects,
                                            const std::string& sensor_id,
                                            double measurement_timestamp,
                                            double max_invisible_period);

  /**
   * @brief
   *
   * @param objects
   * @param sensor_id
   * @param measurement_timestamp
   * @param max_invisible_period
   */
  void UpdateSensorObjectWithMeasurement(SensorId2ObjectMap* objects,
                                         const std::string& sensor_id,
                                         double measurement_timestamp,
                                         double max_invisible_period);

  /**
   * @brief
   *
   * @param obj
   */
  void UpdateWithSensorObjectForBackground(const SensorObjectPtr& obj);

  /**
   * @brief
   *
   * @param sensor_id
   * @param measurement_timestamp
   */
  void UpdateWithoutSensorObjectForBackground(const std::string& sensor_id,
                                              double measurement_timestamp);

 protected:
  SensorId2ObjectMap lidar_objects_;
  SensorId2ObjectMap radar_objects_;
  SensorId2ObjectMap camera_objects_;

  FusedObjectPtr fused_object_ = nullptr;
  double tracking_period_ = 0.0;
  double existence_prob_ = 0.0;
  double toic_prob_ = 0.0;

  bool is_background_ = false;
  bool is_alive_ = true;

  size_t tracked_times_ = 0;

 private:
  // FRIEND_TEST(TrackTest, test);

  static size_t s_track_idx_;
  static double s_max_lidar_invisible_period_;
  static double s_max_radar_invisible_period_;
  static double s_max_camera_invisible_period_;

  DISALLOW_COPY_AND_ASSIGN(Track);
};

typedef std::shared_ptr<Track> TrackPtr;
typedef std::shared_ptr<const Track> TrackConstPtr;

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
