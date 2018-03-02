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

#include "modules/perception/obstacle/base/object.h"
#include "modules/perception/obstacle/camera/common/lane_object.h"
#include "modules/perception/obstacle/camera/lane_post_process/common/type.h"

namespace apollo {
namespace perception {
namespace obstacle {

struct CipvOptions {
  float velocity = 0.0f;
  float yaw_rate = 0.0f;
  float yaw_angle = 0.0f;
};

const float MAX_DIST_OBJECT_TO_LANE_METER = 20.0f;
const float MAX_VEHICLE_WIDTH_METER = 5.0f;

// **** To do **** Should be from ego_vehicle_info header file
const float EGO_CAR_WIDTH_METER = 1.620f;   // 1.620 meter
const float EGO_CAR_LENGTH_METER = 3.564f;  // 3.564 meter
// *********************
const float EGO_CAR_MARGIN_METER = 0.50f;  // 0.5 meter

const float EGO_CAR_VIRTUAL_LANE = EGO_CAR_WIDTH_METER + EGO_CAR_MARGIN_METER;
const float EGO_CAR_HALF_VIRTUAL_LANE = EGO_CAR_VIRTUAL_LANE / 2.0f;

// **** TO DO ***** averatge image frame rate should come from other header
// file.
const float AVERAGE_FRATE_RATE = 0.1f;

class Cipv {
  // Member functions
 public:
  //    friend class ::adu::perception::obstacle::OnlineCalibrationService;
  Cipv(void);
  ~Cipv(void);

  bool init();               // override;
  std::string name() const;  //  override;

  // Determine CIPV among multiple objects
  bool determine_cipv(std::shared_ptr<SensorObjects> sensor_objects,
                      CipvOptions *options);  // override;

 private:
  // Distance from a point to a line segment
  bool distance_from_point_to_line_segment(const Point2Df &point,
                                           const Point2Df &line_seg_start_point,
                                           const Point2Df &line_seg_end_point,
                                           float *distance);

  // Determine CIPV among multiple objects
  bool get_egolane(const LaneObjectsPtr lane_objects, EgoLane *egolane_image,
                   EgoLane *egolane_ground, bool *b_left_valid,
                   bool *b_right_valid);

  // Elongate lane line
  bool elongate_egolane(const LaneObjectsPtr lane_objects,
                        const bool b_left_valid, const bool b_right_valid,
                        const float yaw_rate, const float velocity,
                        EgoLane *egolane_image, EgoLane *egolane_ground);

  // Get closest edge of an object in image cooridnate
  bool find_closest_edge_of_object_image(const ObjectPtr &object,
                                         const EgoLane &egolane_image,
                                         LineSegment2Df *closted_object_edge);

  // Get closest edge of an object in ground cooridnate
  bool find_closest_edge_of_object_ground(const ObjectPtr &object,
                                          const EgoLane &egolane_ground,
                                          LineSegment2Df *closted_object_edge);

  // Check if the distance between lane and object are OK
  bool are_distances_sane(const float distance_start_point_to_right_lane,
                          const float distance_start_point_to_left_lane,
                          const float distance_end_point_to_right_lane,
                          const float distance_end_point_to_left_lane);

  // Check if the object is in the lane in image space
  bool is_object_in_the_lane_image(const ObjectPtr &object,
                                   const EgoLane &egolane_image);
  // Check if the object is in the lane in ego-ground space
  //  |           |
  //  | *------*  |
  //  |         *-+-----*
  //  |           |  *--------* <- closest edge of object
  // *+------*    |
  //  |           |
  // l_lane     r_lane
  bool is_object_in_the_lane_ground(const ObjectPtr &object,
                                    const EgoLane &egolane_ground);

  // Check if the object is in the lane in ego-ground space
  bool is_object_in_the_lane(const ObjectPtr &object,
                             const EgoLane &egolane_image,
                             const EgoLane &egolane_ground);

  // Check if a point is left of a line segment
  bool is_point_left_of_line(const Point2Df &point,
                             const Point2Df &line_seg_start_point,
                             const Point2Df &line_seg_end_point);

  // Make a virtual lane line using a reference lane line and its offset
  // distance
  bool make_virtual_lane(const LaneLine &ref_lane_line, const float yaw_rate,
                         const float offset_distance,
                         LaneLine *virtual_lane_line);

  float vehicle_dynamics(const uint32_t tick, const float yaw_rate,
                         const float velocity, const float time_unit, float *x,
                         float *y);
  // Make a virtual lane line using a yaw_rate
  bool make_virtual_ego_lane_from_yaw_rate(const float yaw_rate,
                                           const float velocity,
                                           const float offset_distance,
                                           LaneLine *left_lane_line,
                                           LaneLine *right_lane_line);
  // Member variables
  bool _b_image_based_cipv = false;
  int32_t _debug_level = 0;
  float _time_unit = 0.0f;
};

}  // namespace obstacle
}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_CAMERA_CIPV_H_
