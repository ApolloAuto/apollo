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

#ifndef MODULES_PERCEPTION_OBSTACLE_TRACK_OBJECT_DISTANCE_H_
#define MODULES_PERCEPTION_OBSTACLE_TRACK_OBJECT_DISTANCE_H_

#include <Eigen/Core>
#include <string>
#include "modules/common/macro.h"
#include "modules/perception/obstacle/lidar/tracker/hm_tracker/object_track.h"
#include "modules/perception/obstacle/lidar/tracker/hm_tracker/tracked_object.h"

namespace apollo {
namespace perception {

class TrackObjectDistance{
 public:
  // @brief set weight of location dist for all the track object distance
  // objects
  // @params[IN] weight_location_dist: weight of location dist
  // @return true if set successfully, otherwise return false
  static bool SetWeightLocationDist(float weight_location_dist);

  // @brief set weight of direction dist for all the track object distance
  // objects
  // @params[IN] weight_direction_dist: weight of direction dist
  // @return true if set successfully, otherwise return false
  static bool SetWeightDirectionDist(float weight_direction_dist);

  // @brief set weight of bbox size dist for all the track object distance
  // objects
  // @params[IN] weight_bbox_size_dist: weight of bbox size dist
  // @return true if set successfully, otherwise return false
  static bool SetWeightBboxSizeDist(float weight_bbox_size_dist);

  // @brief set weight of point num dist for all the track object distance
  // objects
  // @params[IN] weight_point_num_dist: weight of point num dist
  // @return true if set successfully, otherwise return false
  static bool SetWeightPointNumDist(float weight_point_num_dist);

  // @brief set weight of histogram dist for all the track object distance
  // objects
  // @params[IN] weight_histogram_dist: weight of histogram dist
  // @return true if set successfully, otherwise return false
  static bool SetWeightHistogramDist(float weight_histogram_dist);

  // @brief compute distance for given track & object
  // @params[IN] track: track for computing distance
  // @params[IN] track_predict: predicted state of track for computing distance
  // @params[IN] new_object: new detected object for computing distance
  // @params[IN] time_diff: time interval from last matching
  // @return nothing
  static float ComputeDistance(const ObjectTrackPtr& track,
    const Eigen::VectorXf& track_predict,
    const TrackedObjectPtr& new_object,
    const double time_diff);

  std::string Name() const {
    return "TrackObjectDistance";
  }

 private:
  // @brief compute location distance for given track & object
  // @params[IN] track: track for computing distance
  // @params[IN] track_predict: predicted state of track for computing distance
  // @params[IN] new_object: new detected object for computing distance
  // @params[IN] time_diff: time interval from last matching
  // @return nothing
  static float ComputeLocationDistance(const ObjectTrackPtr& track,
    const Eigen::VectorXf& track_predict,
    const TrackedObjectPtr& new_object,
    const double time_diff);

  // @brief compute direction distance for given track & object
  // @params[IN] track: track for computing distance
  // @params[IN] track_predict: predicted state of track for computing distance
  // @params[IN] new_object: new detected object for computing distance
  // @params[IN] time_diff: time interval from last matching
  // @return nothing
  static float ComputeDirectionDistance(const ObjectTrackPtr& track,
    const Eigen::VectorXf& track_predict,
    const TrackedObjectPtr& new_object,
    const double time_diff);

  // @brief compute bbox size distance for given track & object
  // @params[IN] track: track for computing distance
  // @params[IN] track_predict: predicted state of track for computing distance
  // @params[IN] new_object: new detected object for computing distance
  // @params[IN] time_diff: time interval from last matching
  // @return nothing
  static float ComputeBboxSizeDistance(const ObjectTrackPtr& track,
    const Eigen::VectorXf& track_predict,
    const TrackedObjectPtr& new_object,
    const double time_diff);

  // @brief compute point num distance for given track & object
  // @params[IN] track: track for computing distance
  // @params[IN] track_predict: predicted state of track for computing distance
  // @params[IN] new_object: new detected object for computing distance
  // @params[IN] time_diff: time interval from last matching
  // @return nothing
  static float ComputePointNumDistance(const ObjectTrackPtr& track,
    const Eigen::VectorXf& track_predict,
    const TrackedObjectPtr& new_object,
    const double time_diff);

  // @brief compute histogram distance for given track & object
  // @params[IN] track: track for computing distance
  // @params[IN] track_predict: predicted state of track for computing distance
  // @params[IN] new_object: new detected object for computing distance
  // @params[IN] time_diff: time interval from last matching
  // @return nothing
  static float ComputeHistogramDistance(const ObjectTrackPtr& track,
    const Eigen::VectorXf& track_predict,
    const TrackedObjectPtr& new_object,
    const double time_diff);

 protected:
  // distance weights
  static double   s_weight_location_dist_;
  static double   s_weight_direction_dist_;
  static double   s_weight_bbox_size_dist_;
  static double   s_weight_point_num_dist_;
  static double   s_weight_histogram_dist_;

 private:
  DISALLOW_COPY_AND_ASSIGN(TrackObjectDistance);
};  // class TrackObjectDistance

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_TRACK_OBJECT_DISTANCE_H_
