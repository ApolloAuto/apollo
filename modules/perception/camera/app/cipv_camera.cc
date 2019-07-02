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

#include "modules/perception/camera/app/cipv_camera.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "cyber/common/log.h"
#include "modules/common/math/line_segment2d.h"

namespace apollo {
namespace perception {

Cipv::Cipv() {}

Cipv::~Cipv() {}

bool Cipv::Init(const Eigen::Matrix3d &homography_im2car,
                const float min_laneline_length_for_cipv,
                const float average_lane_width_in_meter,
                const float max_vehicle_width_in_meter,
                const float average_frame_rate, const bool image_based_cipv,
                const int debug_devel) {
  b_image_based_cipv_ = image_based_cipv;
  debug_level_ =
      debug_devel;  // 0: no debug message
                    // 1: minimal output
                    // 2: some important output
                    // 3: verbose message
                    // 4: visualization
                    // 5: all
                    // -x: specific debugging, where x is the specific number
  time_unit_ = average_frame_rate;
  homography_im2car_ = homography_im2car;
  homography_car2im_ = homography_im2car.inverse();

  min_laneline_length_for_cipv_ = min_laneline_length_for_cipv;
  average_lane_width_in_meter_ = average_lane_width_in_meter;
  max_vehicle_width_in_meter_ = max_vehicle_width_in_meter;
  margin_vehicle_to_lane_ =
      (average_lane_width_in_meter - max_vehicle_width_in_meter) * 0.5f;
  single_virtual_egolane_width_in_meter_ = max_vehicle_width_in_meter;
  half_vehicle_width_in_meter_ = max_vehicle_width_in_meter * 0.5f;
  half_virtual_egolane_width_in_meter_ =
      single_virtual_egolane_width_in_meter_ * 0.5f;
  old_cipv_track_id_ = -2;

  return true;
}

// Distance from a point to a line segment
bool Cipv::DistanceFromPointToLineSegment(const Point2Df &point,
                                          const Point2Df &line_seg_start_point,
                                          const Point2Df &line_seg_end_point,
                                          float *distance) {
  common::math::Vec2d p = {point(0), point(1)};
  common::math::LineSegment2d line_seg(
      {line_seg_start_point(0), line_seg_start_point(1)},
      {line_seg_end_point(0), line_seg_end_point(1)});
  if (line_seg.length_sqr() <= kFloatEpsilon) {
    // line length = 0
    return false;
  }
  *distance = static_cast<float>(line_seg.DistanceTo(p));
  return true;
}

// Select CIPV among multiple objects
bool Cipv::GetEgoLane(const std::vector<base::LaneLine> &lane_objects,
                      EgoLane *egolane_image, EgoLane *egolane_ground,
                      bool *b_left_valid, bool *b_right_valid) {
  float x, y;
  for (size_t i = 0; i < lane_objects.size(); ++i) {
    const auto &lane_object = lane_objects[i];
    const size_t curve_image_point_size =
        lane_object.curve_image_point_set.size();
    const size_t curve_car_coord_point_set_size =
        lane_object.curve_car_coord_point_set.size();
    if (lane_object.pos_type == base::LaneLinePositionType::EGO_LEFT) {
      if (debug_level_ >= 2) {
        AINFO << "[GetEgoLane]LEFT_image_lane_objects[" << i
              << "].curve_image_point_set.size(): " << curve_image_point_size;
        AINFO << "[GetEgoLane]LEFT_ground_lane_objects[" << i
              << "].curve_car_coord_point_set_size: "
              << curve_car_coord_point_set_size;
      }
      if (curve_image_point_size < min_laneline_length_for_cipv_) {
        *b_left_valid = false;
      } else {
        *b_left_valid = true;

        for (size_t j = 0; j < curve_car_coord_point_set_size; ++j) {
          // ground_point
          x = lane_object.curve_car_coord_point_set[j].x;
          y = lane_object.curve_car_coord_point_set[j].y;
          egolane_ground->left_line.line_point.emplace_back(x, y);
        }
        for (size_t j = 0; j < curve_image_point_size; ++j) {
          // image_point
          x = lane_object.curve_image_point_set[j].x;
          y = lane_object.curve_image_point_set[j].y;
          egolane_image->left_line.line_point.emplace_back(x, y);
        }
      }
    } else if (lane_object.pos_type == base::LaneLinePositionType::EGO_RIGHT) {
      if (debug_level_ >= 2) {
        AINFO << "[GetEgoLane]RIGHT_image_lane_objects[" << i
              << "].curve_image_point_set.size(): " << curve_image_point_size;
        AINFO << "[GetEgoLane]RIGHT_ground_lane_objects[" << i
              << "].curve_car_coord_point_set_size: "
              << curve_car_coord_point_set_size;
      }
      if (curve_image_point_size < min_laneline_length_for_cipv_) {
        *b_right_valid = false;
      } else {
        *b_right_valid = true;
        for (size_t j = 0; j < curve_car_coord_point_set_size; ++j) {
          // ground_point
          x = lane_object.curve_car_coord_point_set[j].x;
          y = lane_object.curve_car_coord_point_set[j].y;
          egolane_ground->right_line.line_point.emplace_back(x, y);
        }
        for (size_t j = 0; j < curve_image_point_size; ++j) {
          // image_point
          x = lane_object.curve_image_point_set[j].x;
          y = lane_object.curve_image_point_set[j].y;
          egolane_image->right_line.line_point.emplace_back(x, y);
        }
      }
    }
  }
  return true;
}

// Make a virtual lane line using a reference lane line and its offset distance
bool Cipv::MakeVirtualLane(const LaneLineSimple &ref_lane_line,
                           const float yaw_rate, const float offset_distance,
                           LaneLineSimple *virtual_lane_line) {
  // TODO(techoe): Use union of lane line and yaw_rate path to define the
  // virtual lane
  virtual_lane_line->line_point.clear();
  for (uint32_t i = 0; i < ref_lane_line.line_point.size(); ++i) {
    virtual_lane_line->line_point.emplace_back(
        ref_lane_line.line_point[i](0),
        ref_lane_line.line_point[i](1) + offset_distance);
  }
  return true;
}

float Cipv::VehicleDynamics(const uint32_t tick, const float yaw_rate,
                            const float velocity, const float time_unit,
                            float *x, float *y) {
  // Option 1. Straight model;
  // *x = time_unit * velocity * static_cast<float>(tick);
  // *y = 0.0f;

  // Option 2. Sphere model
  float adjusted_velocity = std::max(kMinVelocity, velocity);
  float theta = static_cast<float>(tick) * time_unit * yaw_rate;
  float displacement = static_cast<float>(tick) * time_unit * adjusted_velocity;
  *x = displacement * static_cast<float>(cos(theta));
  *y = displacement * static_cast<float>(sin(theta));

  // Option 3. Bicycle model
  // TODO(techoe): Apply bicycle model for vehicle dynamics (need wheel base)

  return true;
}

// Make a virtual lane line using a yaw_rate
bool Cipv::MakeVirtualEgoLaneFromYawRate(const float yaw_rate,
                                         const float velocity,
                                         const float offset_distance,
                                         LaneLineSimple *left_lane_line,
                                         LaneLineSimple *right_lane_line) {
  float x = 0.0f;
  float y = 0.0f;
  left_lane_line->line_point.clear();
  right_lane_line->line_point.clear();

  for (uint32_t i = 1; i < kMaxNumVirtualLanePoint; ++i) {
    VehicleDynamics(i, yaw_rate, velocity, kAverageFrameRate, &x, &y);
    Point2Df left_point(x, y + offset_distance);
    left_lane_line->line_point.emplace_back(left_point);
    Point2Df right_point(x, y - offset_distance);
    right_lane_line->line_point.emplace_back(right_point);
  }
  return true;
}

// Elongate lane line
bool Cipv::ElongateEgoLane(const std::vector<base::LaneLine> &lane_objects,
                           const bool b_left_valid, const bool b_right_valid,
                           const float yaw_rate, const float velocity,
                           EgoLane *egolane_image, EgoLane *egolane_ground) {
  float offset_distance = half_vehicle_width_in_meter_;
  // When left lane line is available
  if (b_left_valid && b_right_valid) {
    // elongate both lanes or do nothing
    if (debug_level_ >= 2) {
      AINFO << "Both lanes are fine";
    }
    // When only left lane line is available
  } else if (!b_left_valid && b_right_valid) {
    // Generate virtual left lane based on right lane
    offset_distance = single_virtual_egolane_width_in_meter_;
    MakeVirtualLane(egolane_ground->right_line, yaw_rate, offset_distance,
                    &egolane_ground->left_line);
    if (debug_level_ >= 2) {
      AINFO << "Made left lane with offset: " << offset_distance;
    }

    // When only right lane line is available
  } else if (b_left_valid && !b_right_valid) {
    // Generate virtual right lane based on left lane
    offset_distance = -single_virtual_egolane_width_in_meter_;
    MakeVirtualLane(egolane_ground->left_line, yaw_rate, offset_distance,
                    &egolane_ground->right_line);
    if (debug_level_ >= 2) {
      AINFO << "Made right lane with offset: " << offset_distance;
    }
  }
  return true;
}

// Create virtual lane line
bool Cipv::CreateVirtualEgoLane(const float yaw_rate, const float velocity,
                                EgoLane *egolane_ground) {
  float offset_distance = half_virtual_egolane_width_in_meter_;
  // Generate new egolane using yaw-rate velocity
  MakeVirtualEgoLaneFromYawRate(yaw_rate, velocity, offset_distance,
                                &egolane_ground->left_line,
                                &egolane_ground->right_line);
  if (debug_level_ >= 2) {
    AINFO << "Made both lane_objects with size of "
          << egolane_ground->left_line.line_point.size();
  }

  return true;
}

// Get closest edge of an object in image coordinate
bool Cipv::FindClosestObjectImage(const std::shared_ptr<base::Object> &object,
                                  const EgoLane &egolane_image,
                                  LineSegment2Df *closted_object_edge,
                                  float *distance) {
  float size_x = object->size(0);
  float size_y = object->size(1);
  float size_z = object->size(2);

  if (size_x < 1.0e-2 && size_y < 1.0e-2 && size_z < 1.0e-2) {
    // size_x = 0.1;
    // size_y = 0.1;
    // size_z = 0.1;
    return false;
  }
  // Footprint (left + width/2, top + height) as a center position
  float center_x = (object->camera_supplement.box.xmin +
                    object->camera_supplement.box.xmin) *
                   0.5f;
  float center_y = object->camera_supplement.box.ymax;

  if (debug_level_ >= 3) {
    AINFO << "object->camera_supplement.box = base::RectF("
          << object->camera_supplement.box.xmin << ", "
          << object->camera_supplement.box.ymin << ", "
          << object->camera_supplement.box.xmax -
                 object->camera_supplement.box.xmin
          << ", "
          << object->camera_supplement.box.ymax -
                 object->camera_supplement.box.ymin
          << ");";
  }
  closted_object_edge->start_point(0) = object->camera_supplement.box.xmin;
  closted_object_edge->start_point(1) = object->camera_supplement.box.ymax;

  closted_object_edge->end_point(0) = object->camera_supplement.box.xmax;
  closted_object_edge->end_point(1) = object->camera_supplement.box.ymax;

  *distance =
      static_cast<float>(sqrt(center_x * center_x + center_y * center_y));
  if (debug_level_ >= 2) {
    AINFO << "start(" << closted_object_edge->start_point(0) << ", "
          << closted_object_edge->start_point(1) << ")->";
    AINFO << "end(" << closted_object_edge->end_point(0) << ", "
          << closted_object_edge->end_point(1) << ")";
    AINFO << "closest distance: " << *distance;
  }
  return true;
}
// Get closest edge of an object in ground coordinate
// TODO(techoe): This function should be changed to find min-y and max-y edges
// to decide CIPV.
bool Cipv::FindClosestObjectGround(const std::shared_ptr<base::Object> &object,
                                   const EgoLane &egolane_ground,
                                   const Eigen::Affine3d world2camera,
                                   LineSegment2Df *closted_object_edge,
                                   float *distance) {
  if (debug_level_ >= 2) {
    AINFO << "object->track_id = " << object->track_id;
  }
  float size_x = object->size(0);
  float size_y = object->size(1);
  float size_z = object->size(2);
  if (size_x < 1.0e-2 && size_y < 1.0e-2 && size_z < 1.0e-2) {
    // size_x = 0.1;
    // size_y = 0.1;
    // size_z = 0.1;
    return false;
  }
  // Option 1. Get position from center
  auto pos = world2camera * object->center;
  float center_x = static_cast<float>(pos(2));
  float center_y = static_cast<float>(-pos(0));

  // Option 2. Homography based 3D inference
  // base::RectF rect(object->camera_supplement.box);
  // float footprint_x = static_cast<float>(rect.x + rect.width * 0.5);
  // float footprint_y = static_cast<float>(rect.y + rect.height);
  // float center_x;
  // float center_y;
  // image2ground(footprint_x, footprint_y, &center_x, &center_y);

  double theta_ray = atan2(pos(0), pos(2));
  double theta = object->camera_supplement.alpha + theta_ray;
  theta -= M_PI_2;

  if (debug_level_ >= 3) {
    AINFO << "object->camera_supplement.box = base::RectF("
          << object->camera_supplement.box.xmin << ", "
          << object->camera_supplement.box.ymin << ", "
          << object->camera_supplement.box.xmax -
                 object->camera_supplement.box.xmin
          << ", "
          << object->camera_supplement.box.ymax -
                 object->camera_supplement.box.ymin
          << ");";

    AINFO << "object.center(0) = " << object->center(0) << ";";
    AINFO << "object.center(1) = " << object->center(1) << ";";
    AINFO << "object.center(2) = " << object->center(2) << ";";
    AINFO << "pos(0) = " << pos(0) << ";";
    AINFO << "pos(1) = " << pos(1) << ";";
    AINFO << "pos(2) = " << pos(2) << ";";
    AINFO << "object->camera_supplement.local_center(0) = "
          << object->camera_supplement.local_center(0) << ";";
    AINFO << "object->camera_supplement.local_center(1) = "
          << object->camera_supplement.local_center(1) << ";";
    AINFO << "object->camera_supplement.local_center(2) = "
          << object->camera_supplement.local_center(2) << ";";
    AINFO << "theta_ray = " << theta_ray << ";";
    AINFO << "object->camera_supplement.alpha = "
          << object->camera_supplement.alpha << ";";
    AINFO << "theta = " << theta << ";";
    AINFO << "object.anchor_point(0) = " << object->anchor_point(0) << ";";
    AINFO << "object.anchor_point(1) = " << object->anchor_point(1) << ";";
    AINFO << "object.anchor_point(2) = " << object->anchor_point(2) << ";";
    AINFO << "object.direction(0) = " << object->direction(0) << ";";
    AINFO << "object.direction(1) = " << object->direction(1) << ";";
    AINFO << "object.direction(2) = " << object->direction(2) << ";";
    AINFO << "object.size(0) = " << object->size(0) << ";";
    AINFO << "object.size(1) = " << object->size(1) << ";";
    AINFO << "object.size(2) = " << object->size(2) << ";";
  }
  float x1 = size_x * 0.5f;
  float x2 = -x1;
  float y1 = size_y * 0.5f;
  float y2 = -y1;
  float cos_theta = static_cast<float>(cos(theta));  // direction_x / len;
  float sin_theta = static_cast<float>(sin(theta));  // -direction_y / len;

  Point2Df p[4];

  p[0](0) = x2 * cos_theta + y1 * sin_theta + center_x;
  p[0](1) = y1 * cos_theta - x2 * sin_theta + center_y;

  p[1](0) = x2 * cos_theta + y2 * sin_theta + center_x;
  p[1](1) = y2 * cos_theta - x2 * sin_theta + center_y;

  p[2](0) = x1 * cos_theta + y1 * sin_theta + center_x;
  p[2](1) = y1 * cos_theta - x1 * sin_theta + center_y;

  p[3](0) = x1 * cos_theta + y2 * sin_theta + center_x;
  p[3](1) = y2 * cos_theta - x1 * sin_theta + center_y;

  if (debug_level_ >= 2) {
    AINFO << "P0(" << p[0](0) << ", " << p[0](1) << ")";
    AINFO << "P1(" << p[1](0) << ", " << p[1](1) << ")";
    AINFO << "P2(" << p[2](0) << ", " << p[2](1) << ")";
    AINFO << "P3(" << p[3](0) << ", " << p[3](1) << ")";
  }

  float closest_x = kMaxFloat;
  float left_y = kMaxFloat;
  float right_y = -kMaxFloat;
  int32_t closest_index = -1;
  int32_t left_index = -1;
  int32_t right_index = -1;
  for (int32_t i = 0; i < 4; i++) {
    if (p[i](1) <= left_y) {
      left_index = i;
      left_y = p[i](1);
    }
    if (p[i](1) > right_y) {
      right_index = i;
      right_y = p[i](1);
    }
    if (p[i](0) < closest_x) {
      closest_index = i;
      closest_x = p[i](0);
    }
  }

  if (left_index < 0 || right_index < 0 || left_index == right_index) {
    if (debug_level_ >= 2) {
      AINFO << "left_index: " << left_index;
      AINFO << "right_index: " << right_index;
    }
    return false;
  }

  closted_object_edge->start_point(0) = p[right_index](0);
  closted_object_edge->start_point(1) = p[right_index](1);

  closted_object_edge->end_point(0) = p[left_index](0);
  closted_object_edge->end_point(1) = p[left_index](1);

  // Added filter to consider an object only in front of ego-car.
  if ((p[left_index](0) < 0 && p[right_index](0) < 0) ||
      (p[closest_index](0) < 0)) {
    return false;
  }

  *distance =
      static_cast<float>(sqrt(p[closest_index](0) * p[closest_index](0) +
                              p[closest_index](1) * p[closest_index](1)));
  if (debug_level_ >= 2) {
    AINFO << "start(" << closted_object_edge->start_point(0) << ", "
          << closted_object_edge->start_point(1) << ")->";
    AINFO << "end(" << closted_object_edge->end_point(0) << ", "
          << closted_object_edge->end_point(1) << ")";
    AINFO << "closest distance to p[" << closest_index << "]("
          << p[closest_index](0) << ", " << p[closest_index](1)
          << "): " << *distance;
  }
  return true;
}

// Check if the distance between lane and object are OK
bool Cipv::AreDistancesSane(const float distance_start_point_to_right_lane,
                            const float distance_start_point_to_left_lane,
                            const float distance_end_point_to_right_lane,
                            const float distance_end_point_to_left_lane) {
  float distance = -1.0f;
  if (distance_start_point_to_right_lane > kMaxDistObjectToLaneInMeter) {
    if (debug_level_ >= 1) {
      AINFO << "distance from start to right lane("
            << distance_start_point_to_right_lane << " m) is too long";
    }
    return false;
  }
  if (distance_start_point_to_left_lane > kMaxDistObjectToLaneInMeter) {
    if (debug_level_ >= 1) {
      AINFO << "distance from start to left lane("
            << distance_start_point_to_left_lane << " m) is too long";
    }
    return false;
  }
  if (distance_end_point_to_right_lane > kMaxDistObjectToLaneInMeter) {
    if (debug_level_ >= 1) {
      AINFO << "distance from end to right lane("
            << distance_end_point_to_right_lane << " m) is too long";
    }
    return false;
  }
  if (distance_end_point_to_left_lane > kMaxDistObjectToLaneInMeter) {
    if (debug_level_ >= 1) {
      AINFO << "distance from end to left lane("
            << distance_end_point_to_left_lane << " m) is too long";
    }
    return false;
  }
  distance = static_cast<float>(fabs(distance_start_point_to_right_lane -
                                     distance_end_point_to_right_lane));
  if (distance > kMaxVehicleWidthInMeter) {
    if (debug_level_ >= 1) {
      AINFO << "width of vehicle (" << distance << " m) is too long";
    }
    return false;
  }

  distance = static_cast<float>(fabs(distance_end_point_to_left_lane -
                                     distance_start_point_to_left_lane));
  if (distance > kMaxVehicleWidthInMeter) {
    if (debug_level_ >= 1) {
      AINFO << "width of vehicle (" << distance << " m) is too long";
    }
    return false;
  }
  // put more conditions here if required.

  // AINFO << "Distances are sane!";

  return true;
}

// Check if a point is left of a line segment
bool Cipv::IsPointLeftOfLine(const Point2Df &point,
                             const Point2Df &line_seg_start_point,
                             const Point2Df &line_seg_end_point) {
  float cross_product = ((line_seg_end_point(0) - line_seg_start_point(0)) *
                         (point(1) - line_seg_start_point(1))) -
                        ((line_seg_end_point(1) - line_seg_start_point(1)) *
                         (point(0) - line_seg_start_point(0)));

  if (cross_product > 0.0f) {
    if (debug_level_ >= 2) {
      AINFO << "point (" << point(0) << ", " << point(1)
            << ") is left of line_segment (" << line_seg_start_point(0) << ", "
            << line_seg_start_point(1) << ")->(" << line_seg_end_point(0)
            << ", " << line_seg_end_point(1)
            << "), cross_product: " << cross_product;
    }
    return true;
  }
  if (debug_level_ >= 2) {
    AINFO << "point (" << point(0) << ", " << point(1)
          << ") is right of line_segment (" << line_seg_start_point(0) << ", "
          << line_seg_start_point(1) << ")->(" << line_seg_end_point(0) << ", "
          << line_seg_end_point(1) << "), cross_product: " << cross_product;
  }
  return false;
}

// Check if the object is in the lane in image space
bool Cipv::IsObjectInTheLaneImage(const std::shared_ptr<base::Object> &object,
                                  const EgoLane &egolane_image,
                                  float *object_distance) {
  LineSegment2Df closted_object_edge;
  bool b_left_lane_clear = false;
  bool b_right_lane_clear = false;
  float shortest_distance = kMaxFloat;
  float distance = 0.0f;

  int closest_index = -1;
  // Find closest edge of a given object bounding box
  float b_valid_object = FindClosestObjectImage(
      object, egolane_image, &closted_object_edge, &distance);
  if (!b_valid_object) {
    if (debug_level_ >= 1) {
      AINFO << "The closest edge of an object is not available";
    }
    return false;
  }
  *object_distance = distance;

  if (debug_level_ >= 3) {
    AINFO << "egolane_image.left_line.line_point.size(): "
          << egolane_image.left_line.line_point.size();
  }
  if (egolane_image.left_line.line_point.size() <= 1) {
    if (debug_level_ >= 1) {
      AINFO << "No left lane";
    }
    return false;
  }

  // Check end_point and left lane
  closest_index = -1;
  shortest_distance = kMaxFloat;
  for (size_t i = 0; i + 1 < egolane_image.left_line.line_point.size(); ++i) {
    // If an end point is in the closest left lane line segments
    distance = kMaxFloat;
    if (DistanceFromPointToLineSegment(
            closted_object_edge.end_point,
            egolane_image.left_line.line_point[i],
            egolane_image.left_line.line_point[i + 1], &distance)) {
      if (distance < shortest_distance) {
        closest_index = static_cast<int>(i);
        shortest_distance = distance;
      }
    }
  }

  // When the closest line segment was found
  if (closest_index >= 0) {
    // Check if the end point is on the right of the line segment
    if (debug_level_ >= 3) {
      AINFO << "[Left] closest_index: " << closest_index
            << ", shortest_distance: " << shortest_distance;
      AINFO << "Should be left to be selected";
    }
    if (IsPointLeftOfLine(
            closted_object_edge.end_point,
            egolane_image.left_line.line_point[closest_index],
            egolane_image.left_line.line_point[closest_index + 1])) {
      b_left_lane_clear = true;
      AINFO << "The left lane is clear";
    }
  }

  if (debug_level_ >= 3) {
    AINFO << "egolane_image.right_line.line_point.size(): "
          << egolane_image.right_line.line_point.size();
  }
  // Check start_point and right lane
  if (egolane_image.right_line.line_point.size() <= 1) {
    if (debug_level_ >= 1) {
      AINFO << "No right lane";
    }
    return false;
  }
  closest_index = -1;
  shortest_distance = kMaxFloat;
  for (size_t i = 0; i + 1 < egolane_image.right_line.line_point.size(); ++i) {
    // If an end point is in the closest right lane line segments
    distance = kMaxFloat;
    if (DistanceFromPointToLineSegment(
            closted_object_edge.start_point,
            egolane_image.right_line.line_point[i],
            egolane_image.right_line.line_point[i + 1], &distance)) {
      if (distance < shortest_distance) {
        closest_index = static_cast<int>(i);
        shortest_distance = distance;
      }
    }
  }
  // When the closest line segment was found
  if (closest_index >= 0) {
    if (debug_level_ >= 3) {
      AINFO << "[right] closest_index: " << closest_index
            << ", shortest_distance: " << shortest_distance;
      AINFO << "Should be right to be selected";
    }
    // Check if the end point is on the right of the line segment
    if (!IsPointLeftOfLine(
            closted_object_edge.start_point,
            egolane_image.right_line.line_point[closest_index],
            egolane_image.right_line.line_point[closest_index + 1])) {
      b_right_lane_clear = true;
      AINFO << "The right lane is clear";
    }
  }

  if (b_left_lane_clear && b_right_lane_clear) {
    AINFO << "The object is in the ego lane";
  } else {
    AINFO << "The object is out of the ego lane";
  }
  return (b_left_lane_clear && b_right_lane_clear);
}

// Check if the object is in the lane in ego-ground space
//  |           |
//  | *------*  |
//  |         *-+-----*
//  |           |  *--------* <- closest edge of object
// *+------*    |
//  |           |
// l_lane     r_lane
bool Cipv::IsObjectInTheLaneGround(const std::shared_ptr<base::Object> &object,
                                   const EgoLane &egolane_ground,
                                   const Eigen::Affine3d world2camera,
                                   const bool b_virtual,
                                   float *object_distance) {
  LineSegment2Df closted_object_edge;
  bool b_left_lane_clear = false;
  bool b_right_lane_clear = false;
  float shortest_distance = 0.0f;
  float distance = 0.0f;
  float max_dist_object_to_lane_in_meter =
      b_virtual ? max_dist_object_to_virtual_lane_in_meter_
                : max_dist_object_to_lane_in_meter_;
  int closest_index = -1;
  // Find closest edge of a given object bounding box
  float b_valid_object = FindClosestObjectGround(
      object, egolane_ground, world2camera, &closted_object_edge, &distance);
  if (!b_valid_object) {
    if (debug_level_ >= 1) {
      AINFO << "The closest edge of an object is not available";
    }
    return false;
  }
  *object_distance = distance;

  if (debug_level_ >= 3) {
    AINFO << "egolane_ground.left_line.line_point.size(): "
          << egolane_ground.left_line.line_point.size();
  }
  if (egolane_ground.left_line.line_point.size() <= 1) {
    if (debug_level_ >= 1) {
      AINFO << "No left lane";
    }
    return false;
  }

  // Check end_point and left lane
  closest_index = -1;
  shortest_distance = kMaxFloat;
  for (size_t i = 0; i + 1 < egolane_ground.left_line.line_point.size(); ++i) {
    // If an end point is in the closest left lane line segments
    distance = kMaxFloat;
    if (DistanceFromPointToLineSegment(
            closted_object_edge.end_point,
            egolane_ground.left_line.line_point[i],
            egolane_ground.left_line.line_point[i + 1], &distance)) {
      if (distance < shortest_distance) {
        closest_index = static_cast<int>(i);
        shortest_distance = distance;
      }
    }
  }

  // When the closest line segment was found
  if (closest_index >= 0) {
    // Check if the end point is on the right of the line segment
    if (debug_level_ >= 3) {
      AINFO << "[Left] closest_index: " << closest_index
            << ", shortest_distance: " << shortest_distance;
    }
    if (!IsPointLeftOfLine(
            closted_object_edge.end_point,
            egolane_ground.left_line.line_point[closest_index],
            egolane_ground.left_line.line_point[closest_index + 1]) &&
        shortest_distance < max_dist_object_to_lane_in_meter) {
      b_left_lane_clear = true;
    }
  }

  if (debug_level_ >= 3) {
    AINFO << "egolane_ground.right_line.line_point.size(): "
          << egolane_ground.right_line.line_point.size();
  }
  // Check start_point and right lane
  if (egolane_ground.right_line.line_point.size() <= 1) {
    if (debug_level_ >= 1) {
      AINFO << "No right lane";
    }
    return false;
  }
  closest_index = -1;
  shortest_distance = kMaxFloat;
  for (size_t i = 0; i + 1 < egolane_ground.right_line.line_point.size(); ++i) {
    // If an end point is in the closest right lane line segments
    distance = kMaxFloat;
    if (DistanceFromPointToLineSegment(
            closted_object_edge.start_point,
            egolane_ground.right_line.line_point[i],
            egolane_ground.right_line.line_point[i + 1], &distance)) {
      if (distance < shortest_distance) {
        closest_index = static_cast<int>(i);
        shortest_distance = distance;
      }
    }
  }
  // When the closest line segment was found
  if (closest_index >= 0) {
    if (debug_level_ >= 3) {
      AINFO << "[right] closest_index: " << closest_index
            << ", shortest_distance: " << shortest_distance;
    }
    // Check if the end point is on the right of the line segment
    if (IsPointLeftOfLine(
            closted_object_edge.start_point,
            egolane_ground.right_line.line_point[closest_index],
            egolane_ground.right_line.line_point[closest_index + 1]) &&
        shortest_distance < max_dist_object_to_lane_in_meter) {
      b_right_lane_clear = true;
    }
  }

  return b_left_lane_clear && b_right_lane_clear;
}

// Check if the object is in the lane in ego-ground space
bool Cipv::IsObjectInTheLane(const std::shared_ptr<base::Object> &object,
                             const EgoLane &egolane_image,
                             const EgoLane &egolane_ground,
                             const Eigen::Affine3d world2camera,
                             const bool b_virtual,
                             float *distance) {
  if (b_image_based_cipv_) {
    return IsObjectInTheLaneImage(object, egolane_image, distance);
  }
  return IsObjectInTheLaneGround(object, egolane_ground, world2camera,
                                 b_virtual, distance);
}

// =====================================================================
// Decide CIPV among multiple objects
bool Cipv::DetermineCipv(const std::vector<base::LaneLine> &lane_objects,
                         const CipvOptions &options,
                         const Eigen::Affine3d &world2camera,
                         std::vector<std::shared_ptr<base::Object>> *objects) {
  if (debug_level_ >= 3) {
    AINFO << "Cipv Got SensorObjects with size of " << objects->size();
    AINFO << "Cipv Got lane object with size of " << lane_objects.size();
  }

  // float yaw_rate = options.yaw_rate;
  // float velocity = options.velocity;
  int32_t cipv_index = -1;
  //    int32_t old_cipv_track_id = sensor_objects.cipv_track_id;
  int32_t cipv_track_id = -1;
  bool b_left_valid = false;
  bool b_right_valid = false;
  int32_t old_cipv_index = -1;
  EgoLane egolane_image;
  EgoLane egolane_ground;
  EgoLane virtual_egolane_ground;
  egolane_ground.left_line.line_point.clear();
  egolane_ground.right_line.line_point.clear();
  virtual_egolane_ground.left_line.line_point.clear();
  virtual_egolane_ground.right_line.line_point.clear();

  // Get ego lanes (in both image and ground coordinate)
  GetEgoLane(lane_objects, &egolane_image, &egolane_ground, &b_left_valid,
             &b_right_valid);
  ElongateEgoLane(lane_objects, b_left_valid, b_right_valid, options.yaw_rate,
                  options.velocity, &egolane_image, &egolane_ground);

  CreateVirtualEgoLane(options.yaw_rate, options.velocity,
                       &virtual_egolane_ground);

  float min_distance = std::numeric_limits<float>::max();
  float distance;
  for (int32_t i = 0; i < static_cast<int32_t>(objects->size()); ++i) {
    if (debug_level_ >= 2) {
      AINFO << "objects[" << i << "]->track_id: " << (*objects)[i]->track_id;
    }
    if (IsObjectInTheLane((*objects)[i], egolane_image, egolane_ground,
                          world2camera, false, &distance) ||
        IsObjectInTheLane((*objects)[i], egolane_image, virtual_egolane_ground,
                          world2camera, true, &distance)) {
      if (cipv_index < 0 || distance < min_distance) {
        cipv_index = i;
        cipv_track_id = (*objects)[i]->track_id;
        min_distance = distance;
      }

      if (debug_level_ >= 2) {
        AINFO << "current cipv_index: " << cipv_index;
      }
    }
    if ((*objects)[i]->track_id == old_cipv_track_id_) {
      old_cipv_index = cipv_index;
    }
  }
  if (debug_level_ >= 1) {
    AINFO << "old_cipv_index: " << old_cipv_index;
    AINFO << "old_cipv_track_id_: " << old_cipv_track_id_;
  }
  if (cipv_index >= 0) {
    if (old_cipv_index >= 0 && old_cipv_index != cipv_index &&
        old_cipv_index < static_cast<int32_t>(objects->size())) {
      (*objects)[old_cipv_index]->b_cipv = false;
    }
    (*objects)[cipv_index]->b_cipv = true;
    old_cipv_track_id_ = cipv_track_id;
    // sensor_objects.cipv_index = cipv_index;
    // sensor_objects.cipv_track_id = cipv_track_id;
    if (debug_level_ >= 1) {
      AINFO << "final cipv_index: " << cipv_index;
      AINFO << "final cipv_track_id: " << cipv_track_id;
      // AINFO << "CIPV Index is changed from " << old_cipv_index << "th
      // object to "
      //            << cipv_index << "th object.";
      // AINFO << "CIPV Track_ID is changed from " << old_cipv_track_id <<
      // " to "
      //            << cipv_track_id << ".";
    }
  } else {
    if (debug_level_ >= 1) {
      AINFO << "No cipv";
    }
  }

  return true;
}

bool Cipv::TranformPoint(const Eigen::VectorXf &in,
                         const Eigen::Matrix4f &motion_matrix,
                         Eigen::Vector3d *out) {
  CHECK(in.rows() == motion_matrix.cols());
  Eigen::VectorXf trans_pt = motion_matrix * in;
  if (fabs(trans_pt(3)) < kFloatEpsilon) {
    return false;
  }

  trans_pt /= trans_pt(3);
  *out << trans_pt(0), trans_pt(1), trans_pt(2);
  return true;
}

bool Cipv::CollectDrops(const base::MotionBufferPtr &motion_buffer,
                        const Eigen::Affine3d &world2camera,
                        std::vector<std::shared_ptr<base::Object>> *objects) {
  int motion_size = static_cast<int>(motion_buffer->size());
  if (debug_level_ >= 2) {
    AINFO << " motion_size: " << motion_size;
  }
  if (motion_size <= 0) {
    ADEBUG << " motion_size: " << motion_size;
    return false;
  }
  // std::map<int, std::vector<std::pair<float, float>>>
  //     tmp_object_trackjectories;
  // std::swap(object_trackjectories_, tmp_object_trackjectories);

  if (debug_level_ >= 2) {
    AINFO << "object_trackjectories_.size(): " << object_trackjectories_.size();
  }
  for (auto obj : *objects) {
    int cur_id = obj->track_id;
    if (debug_level_ >= 2) {
      AINFO << "target ID: " << cur_id;
    }
    // for (auto point : tmp_object_trackjectories[cur_id]) {
    //   object_trackjectories_[cur_id].emplace_back(point);
    // }

    // If it is the first object, set capacity.
    if (object_trackjectories_[cur_id].size() == 0) {
      object_trackjectories_[cur_id].set_capacity(kDropsHistorySize);
    }

    object_id_skip_count_[cur_id] = 0;

    auto pos = world2camera * obj->center;
    float center_x = static_cast<float>(pos(2));
    float center_y = static_cast<float>(-pos(0));

    object_trackjectories_[cur_id].push_back(
        std::make_pair(center_x, center_y));

    if (debug_level_ >= 2) {
      AINFO << "object_trackjectories_[" << cur_id
            << " ].size(): " << object_trackjectories_[cur_id].size();
    }

    Eigen::Matrix4f accum_motion_buffer =
        motion_buffer->at(motion_size - 1).motion;
    // Add drops
    for (std::size_t it = object_trackjectories_[cur_id].size() - 1, count = 0;
         it > 0; it--) {
      if (count >= kDropsHistorySize || count > motion_buffer->size()) {
        break;
      }
      if (static_cast<int>(it) + 1 > motion_size) {
        continue;
      }
      Eigen::VectorXf pt =
          Eigen::VectorXf::Zero((*motion_buffer)[0].motion.cols());
      pt(0) = object_trackjectories_[cur_id][it].first;
      pt(1) = object_trackjectories_[cur_id][it].second;
      pt(2) = 0.0f;
      pt(3) = 1.0f;

      Eigen::Vector3d transformed_pt;
      accum_motion_buffer *= motion_buffer->at(motion_size - it - 1).motion;
      TranformPoint(pt, accum_motion_buffer, &transformed_pt);
      // TranformPoint(pt, (*motion_buffer)[motion_size - count - 1].motion,
      //               &transformed_pt);
      if (debug_level_ >= 3) {
        AINFO << "(*motion_buffer)[" << motion_size - it - 1 << "].motion:";
        AINFO << motion_buffer->at(motion_size - it - 1).motion;
        AINFO << "accum_motion_buffer[" << motion_size - it - 1 << "] =";
        AINFO << accum_motion_buffer;
        AINFO << "target[" << obj->track_id << "][" << it << "]: ("
              << transformed_pt(0) << ", " << transformed_pt(1) << ")";
      }
      obj->drops[count] = transformed_pt;
      obj->drop_num = count++;
    }
  }

  // Currently remove trajectory if they do not exist in the current frame
  // TODO(techoe): need to wait several frames
  for (const auto &each_object : object_trackjectories_) {
    int obj_id = each_object.first;
    bool b_found_id = false;
    for (auto obj : *objects) {
      int cur_id = obj->track_id;
      if (obj_id == cur_id) {
        b_found_id = true;
        break;
      }
    }
    // If object ID was not found erase it from map
    if (!b_found_id && object_trackjectories_[obj_id].size() > 0) {
      //      object_id_skip_count_[obj_id].second++;
      object_id_skip_count_[obj_id]++;
      if (debug_level_ >= 2) {
        AINFO << "object_id_skip_count_[" << obj_id
              << " ]: " << object_id_skip_count_[obj_id];
      }
      if (object_id_skip_count_[obj_id] >= kMaxAllowedSkipObject) {
        if (debug_level_ >= 2) {
          AINFO << "Removed obsolete object " << obj_id;
        }
        object_trackjectories_.erase(obj_id);
        object_id_skip_count_.erase(obj_id);
      }
    }
  }
  if (debug_level_ >= 2) {
    for (auto obj : *objects) {
      int cur_id = obj->track_id;
      AINFO << "obj->track_id: " << cur_id;
      AINFO << "obj->drop_num: " << obj->drop_num;
    }
  }
  return true;
}

bool Cipv::image2ground(const float image_x, const float image_y,
                        float *ground_x, float *ground_y) {
  Eigen::Vector3d p_homo;

  p_homo << image_x, image_y, 1;
  Eigen::Vector3d p_ground;
  p_ground = homography_im2car_ * p_homo;
  if (fabs(p_ground(2)) > std::numeric_limits<double>::min()) {
    *ground_x = static_cast<float>(p_ground(0) / p_ground(2));
    *ground_y = static_cast<float>(p_ground(1) / p_ground(2));
    return true;
  }
  if (debug_level_ >= 1) {
    AINFO << "p_ground(2) too small :" << p_ground(2);
  }
  return false;
}

bool Cipv::ground2image(const float ground_x, const float ground_y,
                        float *image_x, float *image_y) {
  Eigen::Vector3d p_homo_ground;

  p_homo_ground << ground_x, ground_y, 1;
  Eigen::Vector3d p_image;
  p_image = homography_car2im_ * p_homo_ground;
  if (fabs(p_image(2)) > std::numeric_limits<double>::min()) {
    *image_x = static_cast<float>(p_image(0) / p_image(2));
    *image_y = static_cast<float>(p_image(1) / p_image(2));
    return true;
  }
  if (debug_level_ >= 1) {
    AINFO << "p_image(2) too small :" << p_image(2);
  }
  return false;
}

std::string Cipv::Name() const { return "Cipv"; }

// Register plugin.
// REGISTER_CIPV(Cipv);

}  // namespace perception
}  // namespace apollo
