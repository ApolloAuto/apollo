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

#include "modules/perception/obstacle/camera/cipv/cipv.h"

#include <cmath>

#include "modules/common/log.h"
#include "modules/common/math/line_segment2d.h"

namespace apollo {
namespace perception {

Cipv::Cipv(void)
    : vehicle_param_(
          common::VehicleConfigHelper::instance()->GetConfig().vehicle_param()),
      EGO_CAR_WIDTH_METER(vehicle_param_.width()),
      EGO_CAR_LENGTH_METER(vehicle_param_.length()),
      EGO_CAR_MARGIN_METER(0.5f),
      EGO_CAR_VIRTUAL_LANE(EGO_CAR_WIDTH_METER + EGO_CAR_MARGIN_METER),
      EGO_CAR_HALF_VIRTUAL_LANE(EGO_CAR_VIRTUAL_LANE / 2.0f) {}

Cipv::~Cipv(void) {}

bool Cipv::Init() {
  b_image_based_cipv_ = false;
  debug_level_ = 0;  // 0: no debug message
                     // 1: minimal output
                     // 2: some important output
                     // 3: verbose message
                     // 4: visualization
                     // 5: all
                     // -x: specific debugging, where x is the specific number
  time_unit_ = AVERAGE_FRATE_RATE;
  vehicle_param_ =
      common::VehicleConfigHelper::instance()->GetConfig().vehicle_param();
  return true;
}

// Distance from a point to a line segment
bool Cipv::DistanceFromPointToLineSegment(const Point2Df &point,
                                          const Point2Df &line_seg_start_point,
                                          const Point2Df &line_seg_end_point,
                                          float *distance) {
  common::math::Vec2d p = {point[0], point[1]};
  common::math::LineSegment2d line_seg(
      {line_seg_start_point[0], line_seg_start_point[1]},
      {line_seg_end_point[0], line_seg_end_point[1]});
  if (line_seg.length_sqr() <= B_FLT_EPSILON) {
    // line length = 0
    return false;
  }
  *distance = line_seg.DistanceTo(p);
  return true;
}

// Determine CIPV among multiple objects
bool Cipv::GetEgoLane(const LaneObjectsPtr lane_objects, EgoLane *egolane_image,
                      EgoLane *egolane_ground, bool *b_left_valid,
                      bool *b_right_valid) {
  for (size_t i = 0; i < lane_objects->size(); ++i) {
    if ((*lane_objects)[i].spatial == L_0) {
      if (debug_level_ >= 2) {
        AINFO << "[GetEgoLane]LEFT(*lane_objects)[i].image_pos.size(): "
              << (*lane_objects)[i].image_pos.size();
      }
      if ((*lane_objects)[i].image_pos.size() <
          MIN_LANE_LINE_LENGTH_FOR_CIPV_DETERMINATION) {
        *b_left_valid = false;
      } else {
        *b_left_valid = true;

        for (size_t j = 0; j < (*lane_objects)[i].image_pos.size(); ++j) {
          Eigen::Vector2f image_point((*lane_objects)[i].image_pos[j][0],
                                      (*lane_objects)[i].image_pos[j][1]);
          egolane_image->left_line.line_point.push_back(image_point);

          Eigen::Vector2f ground_point((*lane_objects)[i].pos[j][0],
                                       (*lane_objects)[i].pos[j][1]);
          egolane_ground->left_line.line_point.push_back(ground_point);
        }
      }
    } else if ((*lane_objects)[i].spatial == R_0) {
      if (debug_level_ >= 2) {
        AINFO << "[GetEgoLane]RIGHT(*lane_objects)[i].image_pos.size(): "
              << (*lane_objects)[i].image_pos.size();
      }
      if ((*lane_objects)[i].image_pos.size() <
          MIN_LANE_LINE_LENGTH_FOR_CIPV_DETERMINATION) {
        *b_right_valid = false;
      } else {
        *b_right_valid = true;
        for (size_t j = 0; j < (*lane_objects)[i].image_pos.size(); ++j) {
          Eigen::Vector2f image_point((*lane_objects)[i].image_pos[j][0],
                                      (*lane_objects)[i].image_pos[j][1]);
          egolane_image->right_line.line_point.push_back(image_point);

          Eigen::Vector2f ground_point((*lane_objects)[i].pos[j][0],
                                       (*lane_objects)[i].pos[j][1]);
          egolane_ground->right_line.line_point.push_back(ground_point);
        }
      }
    }
  }
  return true;
}

// Make a virtual lane line using a reference lane line and its offset distance
bool Cipv::MakeVirtualLane(const LaneLine &ref_lane_line, const float yaw_rate,
                           const float offset_distance,
                           LaneLine *virtual_lane_line) {
  // TODO(All): Use union of lane line and yaw_rate path to determine the
  // virtual lane
  virtual_lane_line->line_point.clear();
  if (b_image_based_cipv_ == false) {
    for (uint32_t i = 0; i < ref_lane_line.line_point.size(); ++i) {
      Eigen::Vector2f virtual_line_point(
          ref_lane_line.line_point[i][0],
          ref_lane_line.line_point[i][1] + offset_distance);
      virtual_lane_line->line_point.push_back(virtual_line_point);
    }
  } else {
    // Image based extension requires to reproject virtual laneline points to
    // image space.
  }
  return true;
}

float Cipv::VehicleDynamics(const uint32_t tick, const float yaw_rate,
                            const float velocity, const float time_unit,
                            float *x, float *y) {
  // straight model;
  *x = tick;
  *y = 0.0f;

  // float theta = time_unit_ * yaw_rate;
  // float displacement = time_unit_ * velocity;

  // Eigen::Rotation2Df rot2d(theta);
  // Eigen::Vector2f trans;

  // trans(0) = displacement * cos(theta);
  // trans(1) = displacement * sin(theta);

  // motion_2d.block(0, 0, 2, 2) = rot2d.toRotationMatrix().transpose();
  // motion_2d.block(0, 2, 2, 1) = -rot2d.toRotationMatrix().transpose() *
  // trans;
  return true;
}

// Make a virtual lane line using a yaw_rate
bool Cipv::MakeVirtualEgoLaneFromYawRate(const float yaw_rate,
                                         const float velocity,
                                         const float offset_distance,
                                         LaneLine *left_lane_line,
                                         LaneLine *right_lane_line) {
  // ** to do *** Use union of lane line and yaw_rate path to determine the
  // virtual lane
  float x = 0.0f;
  float y = 0.0f;
  left_lane_line->line_point.clear();
  right_lane_line->line_point.clear();

  if (b_image_based_cipv_ == false) {
    for (uint32_t i = 0; i < 120; i += 5) {
      VehicleDynamics(i, yaw_rate, velocity, time_unit_, &x, &y);
      Eigen::Vector2f left_point(x, y + offset_distance);
      left_lane_line->line_point.push_back(left_point);
      Eigen::Vector2f right_point(x, y - offset_distance);
      right_lane_line->line_point.push_back(right_point);
    }
  } else {
    // Image based extension requires to reproject virtual laneline points to
    // image space.
  }
  return true;
}

// Elongate lane line
bool Cipv::ElongateEgoLane(const LaneObjectsPtr lane_objects,
                           const bool b_left_valid, const bool b_right_valid,
                           const float yaw_rate, const float velocity,
                           EgoLane *egolane_image, EgoLane *egolane_ground) {
  float offset_distance = EGO_CAR_HALF_VIRTUAL_LANE;
  // When left lane line is available
  if (b_left_valid && b_right_valid) {
    // elongate both lanes or do nothing
    AINFO << "Both lanes are fine";
    // When only left lane line is avaiable
  } else if (!b_left_valid && b_right_valid) {
    // Generate virtual left lane based on right lane
    offset_distance = -(fabs(egolane_ground->right_line.line_point[0][1]) +
                        EGO_CAR_HALF_VIRTUAL_LANE);
    MakeVirtualLane(egolane_ground->right_line, yaw_rate, offset_distance,
                    &egolane_ground->left_line);
    AINFO << "Made left lane";

    // When only right lane line is avaiable
  } else if (b_left_valid && !b_right_valid) {
    // Generate virtual right lane based on left lane
    offset_distance = (fabs(egolane_ground->left_line.line_point[0][1]) +
                       EGO_CAR_HALF_VIRTUAL_LANE);
    MakeVirtualLane(egolane_ground->left_line, yaw_rate, offset_distance,
                    &egolane_ground->right_line);
    AINFO << "Made right lane";

    // When there is no lane lines available
  } else {  // if (!b_left_valid && !b_right_valid)
    // Generate new egolane using yaw-rate and yaw-angle
    MakeVirtualEgoLaneFromYawRate(yaw_rate, velocity, offset_distance,
                                  &egolane_ground->left_line,
                                  &egolane_ground->right_line);
    AINFO << "Made both lane_objects";
  }

  return true;
}

// Get closest edge of an object in image cooridnate
bool Cipv::FindClosestEdgeOfObjectImage(const std::shared_ptr<Object> &object,
                                        const EgoLane &egolane_image,
                                        LineSegment2Df *closted_object_edge) {
  float size_x = object->length;
  float size_y = object->width;
  float size_z = object->height;
  if (size_x < 1.0e-2 && size_y < 1.0e-2 && size_z < 1.0e-2) {
    // size_x = 0.1;
    // size_y = 0.1;
    // size_z = 0.1;
    return false;
  }
  float center_x = object->center[0];
  float center_y = object->center[1];
  float direction_x = object->direction[0];
  float direction_y = object->direction[1];
  float x1 = size_x / 2;
  float x2 = 0 - x1;
  float y1 = size_y / 2;
  float y2 = 0 - y1;
  double len = sqrt(direction_x * direction_x + direction_y * direction_y);
  float cos_theta = direction_x / len;
  float sin_theta = -direction_y / len;

  // Check if object direction is less than +-45 degree
  // angle = atan2(y, x)
  if (fabs(atan2(object->direction[1], object->direction[0])) <=
      FOURTY_FIVE_DEGREE) {
    // get back of the vehicle
    closted_object_edge->start_point[0] =
        x2 * cos_theta + y1 * sin_theta + center_x;
    closted_object_edge->start_point[1] =
        y1 * cos_theta - x2 * sin_theta + center_y;

    closted_object_edge->end_point[0] =
        x2 * cos_theta + y2 * sin_theta + center_x;
    closted_object_edge->end_point[1] =
        y2 * cos_theta - x2 * sin_theta + center_y;

    // If a vehicle faces side way, extract the side edge of a vehicle
  } else if (atan2(object->direction[1], object->direction[0]) >
             FOURTY_FIVE_DEGREE) {
    // get left side of the vehicle
    closted_object_edge->start_point[0] =
        x2 * cos_theta + y1 * sin_theta + center_x;
    closted_object_edge->start_point[1] =
        y1 * cos_theta - x2 * sin_theta + center_y;

    closted_object_edge->end_point[0] =
        x2 * cos_theta + y1 * sin_theta + center_x;
    closted_object_edge->end_point[1] =
        y1 * cos_theta - x2 * sin_theta + center_y;
  } else if (atan2(object->direction[1], object->direction[0]) <
             -FOURTY_FIVE_DEGREE) {
    // get right side of the vehicle

    closted_object_edge->start_point[0] =
        x1 * cos_theta + y2 * sin_theta + center_x;
    closted_object_edge->start_point[1] =
        y2 * cos_theta - x1 * sin_theta + center_y;

    closted_object_edge->end_point[0] =
        x2 * cos_theta + y2 * sin_theta + center_x;
    closted_object_edge->end_point[1] =
        y2 * cos_theta - x2 * sin_theta + center_y;
  } else {
    // don't get front of vehicle
    closted_object_edge->start_point[0] =
        x1 * cos_theta + y1 * sin_theta + center_x;
    closted_object_edge->start_point[1] =
        y1 * cos_theta - x1 * sin_theta + center_y;

    closted_object_edge->end_point[0] =
        x1 * cos_theta + y2 * sin_theta + center_x;
    closted_object_edge->end_point[1] =
        y2 * cos_theta - x1 * sin_theta + center_y;
  }

  return true;
}
// Get closest edge of an object in ground cooridnate
// *** TO DO *** This funcion should be changed to find min-y and max-y edges
// to determine CIPV.
bool Cipv::FindClosestEdgeOfObjectGround(const std::shared_ptr<Object> &object,
                                         const EgoLane &egolane_ground,
                                         LineSegment2Df *closted_object_edge) {
  if (debug_level_ >= 2) {
    AINFO << "object->track_id: " << object->track_id;
    // AINFO << "object->length: " << object->length;
    // AINFO << "object->width: " << object->width;
    // AINFO << "object->height: " << object->height;
  }
  float size_x = object->length;
  float size_y = object->width;
  float size_z = object->height;
  if (size_x < 1.0e-2 && size_y < 1.0e-2 && size_z < 1.0e-2) {
    // size_x = 0.1;
    // size_y = 0.1;
    // size_z = 0.1;
    return false;
  }
  if (debug_level_ >= 3) {
    AINFO << "object->center[0]: " << object->center[0];
    AINFO << "object->center[1]: " << object->center[1];
    AINFO << "object->center[2]: " << object->center[2];
    AINFO << "object->direction[0]: " << object->direction[0];
    AINFO << "object->direction[1]: " << object->direction[1];
    AINFO << "object->direction[2]: " << object->direction[2];
  }
  float center_x = object->center[0];
  float center_y = object->center[1];
  float direction_x = object->direction[0];
  float direction_y = object->direction[1];
  float x1 = size_x / 2;
  float x2 = 0 - x1;
  float y1 = size_y / 2;
  float y2 = 0 - y1;
  double len = sqrt(direction_x * direction_x + direction_y * direction_y);
  if (len < B_FLT_EPSILON) {
    return false;
  }
  float cos_theta = direction_x / len;
  float sin_theta = -direction_y / len;

  Point2Df p[4];

  p[0][0] = x2 * cos_theta + y1 * sin_theta + center_x;
  p[0][1] = y1 * cos_theta - x2 * sin_theta + center_y;

  p[1][0] = x2 * cos_theta + y2 * sin_theta + center_x;
  p[1][1] = y2 * cos_theta - x2 * sin_theta + center_y;

  p[2][0] = x1 * cos_theta + y1 * sin_theta + center_x;
  p[2][1] = y1 * cos_theta - x1 * sin_theta + center_y;

  p[3][0] = x1 * cos_theta + y2 * sin_theta + center_x;
  p[3][1] = y2 * cos_theta - x1 * sin_theta + center_y;

  if (debug_level_ >= 2) {
    AINFO << "P0(" << p[0][0] << ", " << p[0][1] << ")";
    AINFO << "P1(" << p[1][0] << ", " << p[1][1] << ")";
    AINFO << "P2(" << p[2][0] << ", " << p[2][1] << ")";
    AINFO << "P3(" << p[3][0] << ", " << p[3][1] << ")";
  }

  float closest_x = MAX_FLOAT;
  float second_closest_x = MAX_FLOAT - 1.0f;
  int32_t closest_index = -1;
  int32_t second_closest_index = -1;
  for (int32_t i = 0; i < 4; i++) {
    if (p[i][0] <= closest_x) {
      second_closest_index = closest_index;
      second_closest_x = closest_x;
      closest_index = i;
      closest_x = p[i][0];
    } else if (p[i][0] <= second_closest_x) {
      second_closest_index = i;
      second_closest_x = p[i][0];
    }
  }
  if (p[closest_index][1] >= p[second_closest_index][1]) {
    closted_object_edge->start_point[0] = p[closest_index][0];
    closted_object_edge->start_point[1] = p[closest_index][1];

    closted_object_edge->end_point[0] = p[second_closest_index][0];
    closted_object_edge->end_point[1] = p[second_closest_index][1];
  } else {
    closted_object_edge->start_point[0] = p[second_closest_index][0];
    closted_object_edge->start_point[1] = p[second_closest_index][1];

    closted_object_edge->end_point[0] = p[closest_index][0];
    closted_object_edge->end_point[1] = p[closest_index][1];
  }

  if (debug_level_ >= 2) {
    AINFO << "start(" << closted_object_edge->start_point[0] << ", "
          << closted_object_edge->start_point[1] << ")->";
    AINFO << "end(" << closted_object_edge->end_point[0] << ", "
          << closted_object_edge->end_point[1] << ")";
  }
  return true;
}

// Check if the distance between lane and object are OK
bool Cipv::AreDistancesSane(const float distance_start_point_to_right_lane,
                            const float distance_start_point_to_left_lane,
                            const float distance_end_point_to_right_lane,
                            const float distance_end_point_to_left_lane) {
  float distance = -1.0f;
  if (distance_start_point_to_right_lane > MAX_DIST_OBJECT_TO_LANE_METER) {
    if (debug_level_ >= 1) {
      AINFO << "distance from start to right lane(" << distance
            << " m) is too long";
    }
    return false;
  }
  if (distance_start_point_to_left_lane > MAX_DIST_OBJECT_TO_LANE_METER) {
    if (debug_level_ >= 1) {
      AINFO << "distance from start to left lane(" << distance
            << " m) is too long";
    }
    return false;
  }
  if (distance_end_point_to_right_lane > MAX_DIST_OBJECT_TO_LANE_METER) {
    if (debug_level_ >= 1) {
      AINFO << "distance from end to right lane(" << distance
            << " m) is too long";
    }
    return false;
  }
  if (distance_end_point_to_left_lane > MAX_DIST_OBJECT_TO_LANE_METER) {
    if (debug_level_ >= 1) {
      AINFO << "distance from end to left lane(" << distance
            << " m) is too long";
    }
    return false;
  }
  distance = fabs(distance_start_point_to_right_lane -
                  distance_end_point_to_right_lane);
  if (distance > MAX_VEHICLE_WIDTH_METER) {
    if (debug_level_ >= 1) {
      AINFO << "width of vehicle (" << distance << " m) is too long";
    }
    return false;
  }

  distance =
      fabs(distance_end_point_to_left_lane - distance_start_point_to_left_lane);
  if (distance > MAX_VEHICLE_WIDTH_METER) {
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
  float cross_product = ((line_seg_end_point[0] - line_seg_start_point[0]) *
                         (point[1] - line_seg_start_point[1])) -
                        ((line_seg_end_point[1] - line_seg_start_point[1]) *
                         (point[0] - line_seg_start_point[0]));

  if (cross_product > 0.0f) {
    if (debug_level_ >= 2) {
      AINFO << "point (" << point[0] << ", " << point[1]
            << ") is left of line_segment (" << line_seg_start_point[0] << ", "
            << line_seg_start_point[1] << ")->(" << line_seg_end_point[0]
            << ", " << line_seg_end_point[1]
            << "), cross_product: " << cross_product;
    }
    return true;
  } else {
    if (debug_level_ >= 2) {
      AINFO << "point (" << point[0] << ", " << point[1]
            << ") is right of line_segment (" << line_seg_start_point[0] << ", "
            << line_seg_start_point[1] << ")->(" << line_seg_end_point[0]
            << ", " << line_seg_end_point[1]
            << "), cross_product: " << cross_product;
    }
    return false;
  }
}
// Check if the object is in the lane in image space
bool Cipv::IsObjectInTheLaneImage(const std::shared_ptr<Object> &object,
                                  const EgoLane &egolane_image) {
  return true;
}

// Check if the object is in the lane in ego-ground space
//  |           |
//  | *------*  |
//  |         *-+-----*
//  |           |  *--------* <- closest edge of object
// *+------*    |
//  |           |
// l_lane     r_lane
bool Cipv::IsObjectInTheLaneGround(const std::shared_ptr<Object> &object,
                                   const EgoLane &egolane_ground) {
  LineSegment2Df closted_object_edge;
  bool b_left_lane_clear = false;
  bool b_right_lane_clear = false;
  float shortest_distance = 0.0f;
  float distance = 0.0f;
  float shortest_distance_start_point_to_right_lane = 0.0f;
  float shortest_distance_start_point_to_left_lane = 0.0f;
  float shortest_distance_end_point_to_right_lane = 0.0f;
  float shortest_distance_end_point_to_left_lane = 0.0f;

  int closest_index = -1;
  // Find closest edge of a given object bounding box
  float b_valid_object = FindClosestEdgeOfObjectGround(object, egolane_ground,
                                                       &closted_object_edge);
  if (!b_valid_object) {
    if (debug_level_ >= 1) {
      ADEBUG << "The closest edge of an object is not available";
    }
    return false;
  }

  if (debug_level_ >= 3) {
    AINFO << "egolane_ground.left_line.line_point.size(): "
          << egolane_ground.left_line.line_point.size();
  }
  if (egolane_ground.left_line.line_point.size() <= 1) {
    if (debug_level_ >= 1) {
      ADEBUG << "No left lane";
    }
    return false;
  }

  // Check end_point and left lane
  closest_index = -1;
  shortest_distance = MAX_FLOAT;
  for (size_t i = 0; i + 1 < egolane_ground.left_line.line_point.size(); ++i) {
    // If a end point is in the clostest left lane line segments
    distance = MAX_FLOAT;
    if (DistanceFromPointToLineSegment(
            closted_object_edge.end_point,
            egolane_ground.left_line.line_point[i],
            egolane_ground.left_line.line_point[i + 1], &distance) == true) {
      if (distance < shortest_distance) {
        closest_index = i;
        shortest_distance = distance;
      }
    }
  }
  // When the clostest line segment was found
  if (closest_index >= 0) {
    // Check if the end point is on the right of the line segment
    if (debug_level_ >= 3) {
      AINFO << "[Left] closest_index: " << closest_index
            << ", shortest_distance: " << shortest_distance;
    }
    if (IsPointLeftOfLine(
            closted_object_edge.end_point,
            egolane_ground.left_line.line_point[closest_index],
            egolane_ground.left_line.line_point[closest_index + 1]) == false) {
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
      ADEBUG << "No right lane";
    }
    return false;
  }
  closest_index = -1;
  shortest_distance = MAX_FLOAT;
  for (size_t i = 0; i + 1 < egolane_ground.right_line.line_point.size(); ++i) {
    // If a end point is in the clostest right lane line segments
    distance = MAX_FLOAT;
    if (DistanceFromPointToLineSegment(
            closted_object_edge.start_point,
            egolane_ground.right_line.line_point[i],
            egolane_ground.right_line.line_point[i + 1], &distance) == true) {
      if (distance < shortest_distance) {
        closest_index = i;
        shortest_distance = distance;
      }
    }
  }
  // When the clostest line segment was found
  if (closest_index >= 0) {
    if (debug_level_ >= 3) {
      AINFO << "[right] closest_index: " << closest_index
            << ", shortest_distance: " << shortest_distance;
    }
    // Check if the end point is on the right of the line segment
    if (IsPointLeftOfLine(
            closted_object_edge.start_point,
            egolane_ground.right_line.line_point[closest_index],
            egolane_ground.right_line.line_point[closest_index + 1]) == true) {
      b_right_lane_clear = true;
    }
  }

  // Check if the distance is sane
  if (AreDistancesSane(shortest_distance_start_point_to_right_lane,
                       shortest_distance_start_point_to_left_lane,
                       shortest_distance_end_point_to_right_lane,
                       shortest_distance_end_point_to_left_lane)) {
    return (b_left_lane_clear && b_right_lane_clear);

  } else {
    return false;
  }

  return true;
}

// Check if the object is in the lane in ego-ground space
bool Cipv::IsObjectInTheLane(const std::shared_ptr<Object> &object,
                             const EgoLane &egolane_image,
                             const EgoLane &egolane_ground) {
  if (b_image_based_cipv_ == true) {
    return IsObjectInTheLaneImage(object, egolane_image);
  } else {
    return IsObjectInTheLaneGround(object, egolane_ground);
  }
}

// =====================================================================
// Determine CIPV among multiple objects
bool Cipv::DetermineCipv(std::shared_ptr<SensorObjects> sensor_objects,
                         CipvOptions *options) {
  if (debug_level_ >= 3) {
    AINFO << "Cipv Got SensorObjects size " << sensor_objects->objects.size();
    AINFO << "Cipv Got lane object size "
          << sensor_objects->lane_objects->size();
  }
  float yaw_rate = options->yaw_rate;
  float velocity = options->yaw_rate;
  int32_t old_cipv_index = sensor_objects->cipv_index;
  int32_t cipv_index = -1;
  //    int32_t old_cipv_track_id = sensor_objects->cipv_track_id;
  int32_t cipv_track_id = -1;
  // AINFO<<"static_cast<int32_t>(sensor_objects->objects.size(): "
  //           << static_cast<int32_t>(sensor_objects->objects.size());
  bool b_left_valid = false;
  bool b_right_valid = false;
  EgoLane egolane_image;
  EgoLane egolane_ground;

  // Get ego lanes (in both image and ground coordinate)
  GetEgoLane(sensor_objects->lane_objects, &egolane_image, &egolane_ground,
             &b_left_valid, &b_right_valid);
  ElongateEgoLane(sensor_objects->lane_objects, b_left_valid, b_right_valid,
                  yaw_rate, velocity, &egolane_image, &egolane_ground);

  for (int32_t i = 0; i < static_cast<int32_t>(sensor_objects->objects.size());
       ++i) {
    if (debug_level_ >= 2) {
      AINFO << "sensor_objects->objects[i]->track_id: "
            << sensor_objects->objects[i]->track_id;
    }
    if (IsObjectInTheLane(sensor_objects->objects[i], egolane_image,
                          egolane_ground) == true) {
      if (cipv_index < 0 ||
          sensor_objects->objects[i]->center[0] <
              sensor_objects->objects[cipv_index]->center[0]) {
        // cipv_index is not set or if objects[i] is closer than
        // objects[cipv_index] in ego-x coordinate

        // AINFO << "sensor_objects->objects[i]->center[0]: "
        //            << sensor_objects->objects[i]->center[0];
        // AINFO << "sensor_objects->objects[cipv_index]->center[0]: "
        //            << sensor_objects->objects[cipv_index]->center[0];
        cipv_index = i;
        cipv_track_id = sensor_objects->objects[i]->track_id;
      }
      if (debug_level_ >= 2) {
        AINFO << "current cipv_index: " << cipv_index;
      }
    }
  }
  // AINFO << "old_cipv_index: " << old_cipv_index;
  if (old_cipv_index != cipv_index && cipv_index >= 0) {
    // AINFO << "sensor_objects->objects[cipv_index]->b_cipv: "
    //            << sensor_objects->objects[cipv_index]->b_cipv;
    // AINFO << "sensor_objects->cipv_index: "
    //            << sensor_objects->cipv_index;
    // AINFO << "sensor_objects->cipv_track_id: "
    //            << sensor_objects->cipv_track_id;
    if (old_cipv_index >= 0) {
      // AINFO << "sensor_objects->objects[old_cipv_index]->b_cipv: "
      //            << sensor_objects->objects[old_cipv_index]->b_cipv;
      sensor_objects->objects[old_cipv_index]->b_cipv = false;
    }
    sensor_objects->objects[cipv_index]->b_cipv = true;
    sensor_objects->cipv_index = cipv_index;
    sensor_objects->cipv_track_id = cipv_track_id;
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

std::string Cipv::Name() const { return "Cipv"; }

// Register plugin.
// REGISTER_CIPV(Cipv);

}  // namespace perception
}  // namespace apollo
