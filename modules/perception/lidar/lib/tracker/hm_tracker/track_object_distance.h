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
#ifndef PERCEPTION_LIDAR_LIB_TRACKER_HM_TRACKER_TRACK_OBJECT_DISTANCE_H_
#define PERCEPTION_LIDAR_LIB_TRACKER_HM_TRACKER_TRACK_OBJECT_DISTANCE_H_

#include <Eigen/Core>
#include <string>
#include "modules/perception/lidar/lib/tracker/common/track_data.h"
#include "modules/perception/lidar/lib/tracker/common/tracked_object.h"

namespace apollo {
namespace perception {
namespace lidar {

class TrackObjectDistance {
 public:
  TrackObjectDistance() = default;
  ~TrackObjectDistance() = default;
  // @brief set weight of location dist for all the track object distance
  // @params[IN] weight_location_dist: weight of location dist
  // @return true if set successfully, otherwise return false
  bool set_weight_location_dist(float weight_location_dist);

  // @brief set weight of direction dist for all the track object distance
  // @params[IN] weight_direction_dist: weight of direction dist
  // @return true if set successfully, otherwise return false
  bool set_weight_direction_dist(float weight_direction_dist);

  // @brief set weight of bbox size dist for all the track object distance
  // @params[IN] weight_bbox_size_dist: weight of bbox size dist
  // @return true if set successfully, otherwise return false
  bool set_weight_bbox_size_dist(float weight_bbox_size_dist);

  // @brief set weight of point num dist for all the track object distance
  // @params[IN] weight_point_num_dist: weight of point num dist
  // @return true if set successfully, otherwise return false
  bool set_weight_point_num_dist(float weight_point_num_dist);

  // @brief set weight of histogram dist for all the track object distance
  // @params[IN] weight_histogram_dist: weight of histogram dist
  // @return true if set successfully, otherwise return false
  bool set_weight_histogram_dist(float weight_histogram_dist);

  // @brief set weight of centroid shift dist for all the track object distance
  // @params[IN] weight_centroid_shift_dist: weight of histogram dist
  // @return true if set successfully, otherwise return false
  bool set_weight_centroid_shift_dist(float weight_centroid_shift_dist);

  // @brief set weight of bbox iou dist for all the track object distance
  // @params[IN] weight_bbox_iou_dist: weight of histogram dist
  // @return true if set successfully, otherwise return false
  bool set_weight_bbox_iou_dist(float weight_bbox_iou_dist);

  // @brief set backfround and boject match thresh
  // @param[IN] background_object_match_threshold
  // @return true if successfully, otherwise return false
  bool set_background_object_match_threshold(
      float background_object_match_threshold);

  // @brief compute distance for given track & object
  // @params[IN] track: track for computing distance
  // @params[IN] track_predict: predicted state of track for computing distance
  // @params[IN] new_object: new detected object for computing distance
  // @params[IN] time_diff: time interval from last matching
  // @return nothing
  float ComputeDistance(const TrackDataPtr &track,
                        const Eigen::VectorXf &track_predict,
                        const TrackedObjectPtr &new_object,
                        const double time_diff);

  std::string Name() const { return "TrackObjectDistance"; }

 private:
  // @brief compute location distance for given track & object
  // @params[IN] track: track for computing distance
  // @params[IN] track_predict: predicted state of track for computing distance
  // @params[IN] new_object: new detected object for computing distance
  // @params[IN] time_diff: time interval from last matching
  // @return nothing
  float ComputeLocationDistance(const TrackDataPtr &track,
                                const Eigen::VectorXf &track_predict,
                                const TrackedObjectPtr &new_object,
                                const double time_diff);

  // @brief compute direction distance for given track & object
  // @params[IN] track: track for computing distance
  // @params[IN] track_predict: predicted state of track for computing distance
  // @params[IN] new_object: new detected object for computing distance
  // @params[IN] time_diff: time interval from last matching
  // @return nothing
  float ComputeDirectionDistance(const TrackDataPtr &track,
                                 const Eigen::VectorXf &track_predict,
                                 const TrackedObjectPtr &new_object,
                                 const double time_diff);

  // @brief compute bbox size distance for given track & object
  // @params[IN] track: track for computing distance
  // @params[IN] track_predict: predicted state of track for computing distance
  // @params[IN] new_object: new detected object for computing distance
  // @params[IN] time_diff: time interval from last matching
  // @return nothing
  float ComputeBboxSizeDistance(const TrackDataPtr &track,
                                const Eigen::VectorXf &track_predict,
                                const TrackedObjectPtr &new_object,
                                const double time_diff);

  // @brief compute point num distance for given track & object
  // @params[IN] track: track for computing distance
  // @params[IN] track_predict: predicted state of track for computing distance
  // @params[IN] new_object: new detected object for computing distance
  // @params[IN] time_diff: time interval from last matching
  // @return nothing
  float ComputePointNumDistance(const TrackDataPtr &track,
                                const Eigen::VectorXf &track_predict,
                                const TrackedObjectPtr &new_object,
                                const double time_diff);

  // @brief compute histogram distance for given track & object
  // @params[IN] track: track for computing distance
  // @params[IN] track_predict: predicted state of track for computing distance
  // @params[IN] new_object: new detected object for computing distance
  // @params[IN] time_diff: time interval from last matching
  // @return nothing
  float ComputeHistogramDistance(const TrackDataPtr &track,
                                 const Eigen::VectorXf &track_predict,
                                 const TrackedObjectPtr &new_object,
                                 const double time_diff);

  // @brief compute centroid shift distance for object and background match
  // @params[IN] track: track for computing distance
  // @params[IN] track_predict: unused
  // @params[IN] new_object: new detected object for computing distance
  // @params[IN] time_diff: unused
  // @return nothing
  float ComputeCentroidShiftDistance(const TrackDataPtr &track,
                                     const Eigen::VectorXf &track_predict,
                                     const TrackedObjectPtr &cur_obj,
                                     const double time_diff);

  // @brief compute bbox iou distance for object and background match
  // @params[IN] track: track for computing distance
  // @params[IN] track_predict: unused
  // @params[IN] new_object: new detected object for computing distance
  // @params[IN] time_diff: unused
  // @return nothing
  float ComputeBboxIouDistance(const TrackDataPtr &track,
                               const Eigen::VectorXf &track_predict,
                               const TrackedObjectPtr &cur_obj,
                               const double time_diff);

  TrackObjectDistance(const TrackObjectDistance &) = delete;
  TrackObjectDistance &operator=(const TrackObjectDistance &) = delete;

 protected:
  // distance weights
  double weight_location_dist_ = 0;
  double weight_direction_dist_ = 0;
  double weight_bbox_size_dist_ = 0;
  double weight_point_num_dist_ = 0;
  double weight_histogram_dist_ = 0;
  double weight_centroid_shift_dist_ = 0;
  double weight_bbox_iou_dist_ = 0;

  // if track is foreground
  double background_object_match_threshold_ = 0;
};  // class TrackObjectDistance

}  // namespace lidar
}  // namespace perception
}  // namespace apollo

#endif  // PERCEPTION_LIDAR_LIB_TRACKER_HM_TRACKER_TRACK_OBJECT_DISTANCE_H_
