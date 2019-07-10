/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include <array>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "Eigen/Dense"
#include "Eigen/Eigen"
#include "Eigen/Geometry"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/perception/base/lane_struct.h"
#include "modules/perception/base/object.h"
#include "modules/perception/camera/common/lane_object.h"

namespace apollo {
namespace perception {

struct CipvOptions {
  float velocity = 5.0f;
  float yaw_rate = 0.0f;
  float yaw_angle = 0.0f;
};

constexpr float kMinVelocity = 10.0f;  // in m/s
constexpr float kMaxDistObjectToLaneInMeter = 70.0f;
constexpr float kMaxDistObjectToVirtualLaneInMeter = 10.0f;
constexpr float kMaxDistObjectToLaneInPixel = 10.0f;
const std::size_t kDropsHistorySize = 20;
const std::size_t kMaxObjectNum = 100;
const std::size_t kMaxAllowedSkipObject = 10;

static constexpr uint32_t kMaxNumVirtualLanePoint = 25;
// TODO(All) average image frame rate should come from other header file.
static constexpr float kAverageFrameRate = 0.05f;

class Cipv {
  // Member functions
 public:
  Cipv(void);
  virtual ~Cipv(void);

  virtual bool Init(
      const Eigen::Matrix3d &homography_im2car,
      const float min_laneline_length_for_cipv = kMinLaneLineLengthForCIPV,
      const float average_lane_width_in_meter = kAverageLaneWidthInMeter,
      const float max_vehicle_width_in_meter = kMaxVehicleWidthInMeter,
      const float average_frame_rate = kAverageFrameRate,
      const bool image_based_cipv = false, const int debug_level = 0);
  virtual std::string Name() const;

  // Determine CIPV among multiple objects
  bool DetermineCipv(const std::vector<base::LaneLine> &lane_objects,
                     const CipvOptions &options,
                     const Eigen::Affine3d &world2camera,
                     std::vector<std::shared_ptr<base::Object>> *objects);

  // Collect drops for tailgating
  bool CollectDrops(const base::MotionBufferPtr &motion_buffer,
                    const Eigen::Affine3d &world2camera,
                    std::vector<std::shared_ptr<base::Object>> *objects);

  static float VehicleDynamics(const uint32_t tick, const float yaw_rate,
                               const float velocity, const float time_unit,
                               float *x, float *y);
  static float VehicleDynamics(const uint32_t tick, const float yaw_rate,
                               const float velocity, const float time_unit,
                               const float vehicle_half_width, float *center_x,
                               float *ceneter_y, float *left_x, float *left_y,
                               float *right_x, float *right_y);
  // Make a virtual lane line using a yaw_rate
  static bool MakeVirtualEgoLaneFromYawRate(const float yaw_rate,
                                            const float velocity,
                                            const float offset_distance,
                                            LaneLineSimple *left_lane_line,
                                            LaneLineSimple *right_lane_line);

 private:
  // Distance from a point to a line segment
  bool DistanceFromPointToLineSegment(const Point2Df &point,
                                      const Point2Df &line_seg_start_point,
                                      const Point2Df &line_seg_end_point,
                                      float *distance);

  // Determine CIPV among multiple objects
  bool GetEgoLane(const std::vector<base::LaneLine> &lane_objects,
                  EgoLane *egolane_image, EgoLane *egolane_ground,
                  bool *b_left_valid, bool *b_right_valid);

  // Elongate lane line
  bool ElongateEgoLane(const std::vector<base::LaneLine> &lane_objects,
                       const bool b_left_valid, const bool b_right_valid,
                       const float yaw_rate, const float velocity,
                       EgoLane *egolane_image, EgoLane *egolane_ground);
  bool CreateVirtualEgoLane(const float yaw_rate, const float velocity,
                            EgoLane *egolane_ground);

  // Get closest edge of an object in image coordinate
  bool FindClosestObjectImage(const std::shared_ptr<base::Object> &object,
                              const EgoLane &egolane_image,
                              LineSegment2Df *closted_object_edge,
                              float *distance);

  // Get closest edge of an object in ground coordinate
  bool FindClosestObjectGround(const std::shared_ptr<base::Object> &object,
                               const EgoLane &egolane_ground,
                               const Eigen::Affine3d world2camera,
                               LineSegment2Df *closted_object_edge,
                               float *distance);

  // Check if the distance between lane and object are OK
  bool AreDistancesSane(const float distance_start_point_to_right_lane,
                        const float distance_start_point_to_left_lane,
                        const float distance_end_point_to_right_lane,
                        const float distance_end_point_to_left_lane);

  // Check if the object is in the lane in image space
  bool IsObjectInTheLaneImage(const std::shared_ptr<base::Object> &object,
                              const EgoLane &egolane_image, float *distance);
  // Check if the object is in the lane in ego-ground space
  //  |           |
  //  | *------*  |
  //  |         *-+-----*
  //  |           |  *--------* <- closest edge of object
  // *+------*    |
  //  |           |
  // l_lane     r_lane
  bool IsObjectInTheLaneGround(const std::shared_ptr<base::Object> &object,
                               const EgoLane &egolane_ground,
                               const Eigen::Affine3d world2camera,
                               const bool b_virtual, float *distance);

  // Check if the object is in the lane in ego-ground space
  bool IsObjectInTheLane(const std::shared_ptr<base::Object> &object,
                         const EgoLane &egolane_image,
                         const EgoLane &egolane_ground,
                         const Eigen::Affine3d world2camera,
                         const bool b_virtual, float *distance);

  // Check if a point is left of a line segment
  bool IsPointLeftOfLine(const Point2Df &point,
                         const Point2Df &line_seg_start_point,
                         const Point2Df &line_seg_end_point);

  // Make a virtual lane line using a reference lane line and its offset
  // distance
  bool MakeVirtualLane(const LaneLineSimple &ref_lane_line,
                       const float yaw_rate, const float offset_distance,
                       LaneLineSimple *virtual_lane_line);

  // transform point to another using motion
  bool TranformPoint(const Eigen::VectorXf &in,
                     const Eigen::Matrix4f &motion_matrix,
                     Eigen::Vector3d *out);

  bool image2ground(const float image_x, const float image_y, float *ground_x,
                    float *ground_y);
  bool ground2image(const float ground_x, const float ground_y, float *image_x,
                    float *image_y);

  // Member variables
  bool b_image_based_cipv_ = false;
  int32_t debug_level_ = 0;
  float time_unit_ = kAverageFrameRate;

  float min_laneline_length_for_cipv_ = kMinLaneLineLengthForCIPV;
  float average_lane_width_in_meter_ = kAverageLaneWidthInMeter;
  float max_vehicle_width_in_meter_ = kMaxVehicleWidthInMeter;
  float margin_vehicle_to_lane_ =
      (kAverageLaneWidthInMeter - kMaxVehicleWidthInMeter) * 0.5f;
  float single_virtual_egolane_width_in_meter_ =
      kMaxVehicleWidthInMeter + kMarginVehicleToLane;
  float half_virtual_egolane_width_in_meter_ =
      single_virtual_egolane_width_in_meter_ * 0.5f;
  float half_vehicle_width_in_meter_ = kMaxVehicleWidthInMeter * 0.5f;
  float max_dist_object_to_lane_in_meter_ = kMaxDistObjectToLaneInMeter;
  float max_dist_object_to_virtual_lane_in_meter_ =
      kMaxDistObjectToVirtualLaneInMeter;
  float max_dist_object_to_lane_in_pixel_ = kMaxDistObjectToLaneInPixel;
  float MAX_VEHICLE_WIDTH_METER = 5.0f;
  float EPSILON = 1.0e-6f;
  std::size_t kDropsHistorySize = 100;
  std::size_t kMaxObjectNum = 100;
  std::size_t kMaxAllowedSkipObject = 10;

  std::map<int, size_t> object_id_skip_count_;
  std::map<int, boost::circular_buffer<std::pair<float, float>>>
      object_trackjectories_;
  std::map<int, std::vector<double>> object_timestamps_;
  Eigen::Matrix3d homography_im2car_ = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d homography_car2im_ = Eigen::Matrix3d::Identity();
  int32_t old_cipv_track_id_ = -2;
};

}  // namespace perception
}  // namespace apollo
