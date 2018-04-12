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

#ifndef MODULES_PERCEPTION_OBSTACLE_CAMERA_CIPV_H_
#define MODULES_PERCEPTION_OBSTACLE_CAMERA_CIPV_H_

#include <array>
#include <memory>
#include <string>

#include "Eigen/Dense"
#include "Eigen/Eigen"
#include "Eigen/Geometry"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/perception/obstacle/base/object.h"
#include "modules/perception/obstacle/camera/common/lane_object.h"
#include "modules/perception/obstacle/camera/lane_post_process/common/type.h"

namespace apollo {
namespace perception {

struct CipvOptions {
  float velocity = 0.0f;
  float yaw_rate = 0.0f;
  float yaw_angle = 0.0f;
};

const float MAX_DIST_OBJECT_TO_LANE_METER = 20.0f;
const float MAX_VEHICLE_WIDTH_METER = 5.0f;

// TODO(All) averatge image frame rate should come from other header file.
const float AVERAGE_FRATE_RATE = 0.1f;

class Cipv {
  // Member functions
 public:
  //    friend class ::adu::perception::OnlineCalibrationService;
  Cipv(void);
  virtual ~Cipv(void);

  virtual bool Init();
  virtual std::string Name() const;

  // Determine CIPV among multiple objects
  bool DetermineCipv(std::shared_ptr<SensorObjects> sensor_objects,
                     CipvOptions *options);  // override;

 private:
  // Distance from a point to a line segment
  bool DistanceFromPointToLineSegment(const Point2Df &point,
                                      const Point2Df &line_seg_start_point,
                                      const Point2Df &line_seg_end_point,
                                      float *distance);

  // Determine CIPV among multiple objects
  bool GetEgoLane(const LaneObjectsPtr lane_objects, EgoLane *egolane_image,
                  EgoLane *egolane_ground, bool *b_left_valid,
                  bool *b_right_valid);

  // Elongate lane line
  bool ElongateEgoLane(const LaneObjectsPtr lane_objects,
                       const bool b_left_valid, const bool b_right_valid,
                       const float yaw_rate, const float velocity,
                       EgoLane *egolane_image, EgoLane *egolane_ground);

  // Get closest edge of an object in image cooridnate
  bool FindClosestEdgeOfObjectImage(const std::shared_ptr<Object> &object,
                                    const EgoLane &egolane_image,
                                    LineSegment2Df *closted_object_edge);

  // Get closest edge of an object in ground cooridnate
  bool FindClosestEdgeOfObjectGround(const std::shared_ptr<Object> &object,
                                     const EgoLane &egolane_ground,
                                     LineSegment2Df *closted_object_edge);

  // Check if the distance between lane and object are OK
  bool AreDistancesSane(const float distance_start_point_to_right_lane,
                        const float distance_start_point_to_left_lane,
                        const float distance_end_point_to_right_lane,
                        const float distance_end_point_to_left_lane);

  // Check if the object is in the lane in image space
  bool IsObjectInTheLaneImage(const std::shared_ptr<Object> &object,
                              const EgoLane &egolane_image);
  // Check if the object is in the lane in ego-ground space
  //  |           |
  //  | *------*  |
  //  |         *-+-----*
  //  |           |  *--------* <- closest edge of object
  // *+------*    |
  //  |           |
  // l_lane     r_lane
  bool IsObjectInTheLaneGround(const std::shared_ptr<Object> &object,
                               const EgoLane &egolane_ground);

  // Check if the object is in the lane in ego-ground space
  bool IsObjectInTheLane(const std::shared_ptr<Object> &object,
                         const EgoLane &egolane_image,
                         const EgoLane &egolane_ground);

  // Check if a point is left of a line segment
  bool IsPointLeftOfLine(const Point2Df &point,
                         const Point2Df &line_seg_start_point,
                         const Point2Df &line_seg_end_point);

  // Make a virtual lane line using a reference lane line and its offset
  // distance
  bool MakeVirtualLane(const LaneLine &ref_lane_line, const float yaw_rate,
                       const float offset_distance,
                       LaneLine *virtual_lane_line);

  float VehicleDynamics(const uint32_t tick, const float yaw_rate,
                        const float velocity, const float time_unit, float *x,
                        float *y);
  // Make a virtual lane line using a yaw_rate
  bool MakeVirtualEgoLaneFromYawRate(const float yaw_rate, const float velocity,
                                     const float offset_distance,
                                     LaneLine *left_lane_line,
                                     LaneLine *right_lane_line);
  // Member variables
  bool b_image_based_cipv_ = false;
  int32_t debug_level_ = 0;
  float time_unit_ = 0.0f;
  common::VehicleParam vehicle_param_;

  const float EGO_CAR_WIDTH_METER;
  const float EGO_CAR_LENGTH_METER;
  const float EGO_CAR_MARGIN_METER;
  const float EGO_CAR_VIRTUAL_LANE;
  const float EGO_CAR_HALF_VIRTUAL_LANE;
};

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_CAMERA_CIPV_H_
