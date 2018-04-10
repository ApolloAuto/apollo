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

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "opencv2/opencv.hpp"

#include "modules/common/log.h"
#include "modules/perception/obstacle/camera/lane_post_process/common/base_type.h"
#include "modules/perception/proto/perception_obstacle.pb.h"

#ifndef MODULES_PERCEPTION_OBSTACLE_CAMERA_LANE_POST_PROCESS_COMMON_TYPE_H_
#define MODULES_PERCEPTION_OBSTACLE_CAMERA_LANE_POST_PROCESS_COMMON_TYPE_H_

namespace apollo {
namespace perception {

#ifndef MAX_LANE_HISTORY
#define MAX_LANE_HISTORY 10
#endif

#ifndef MAX_GROUP_PREDICTION_MARKER_NUM
#define MAX_GROUP_PREDICTION_MARKER_NUM 10
#endif

#ifndef MAX_POLY_ORDER
#define MAX_POLY_ORDER 3
#endif

#ifndef MAX_LANE_SPATIAL_LABELS
#define MAX_LANE_SPATIAL_LABELS 3
#endif

#ifndef MIN_BETWEEN_LANE_DISTANCE
#define MIN_BETWEEN_LANE_DISTANCE 2.5
#endif

#ifndef AVEAGE_LANE_WIDTH_METER
#define AVEAGE_LANE_WIDTH_METER 3.7
#endif

constexpr ScalarType INVERSE_AVEAGE_LANE_WIDTH_METER =
    1.0 / AVEAGE_LANE_WIDTH_METER;

#ifndef INF_NON_MASK_POINT_X
#define INF_NON_MASK_POINT_X 10000
#endif

constexpr ScalarType kEpsilon = 1e-5;

// define colors for visualization (Blue, Green, Red)
const cv::Scalar kBlack(0, 0, 0);
const cv::Scalar kWhite(255, 255, 255);
const cv::Scalar kGrey(128, 128, 128);
const cv::Scalar kRed(0, 0, 255);
const cv::Scalar kGreen(0, 255, 0);
const cv::Scalar kBlue(255, 0, 0);
const cv::Scalar kYellow(0, 255, 255);
const cv::Scalar kCyan(255, 255, 0);
const cv::Scalar kMagenta(255, 0, 255);
const cv::Scalar kPurple(255, 48, 155);
const cv::Scalar kGreenYellow(47, 255, 173);

// delay time for visualization
constexpr int kDelayTime = 0;

enum MarkerShapeType {
  POINT = 0,
  LINE_SEGMENT,
};

enum SpaceType {
  IMAGE = 0,
  VEHICLE,
};

typedef Eigen::Matrix<ScalarType, 2, 1> Vector2D;
typedef Eigen::Matrix<ScalarType, 3, 1> Vector3D;
enum AssociationMethod {
  GREEDY_GROUP_CONNECT = 0,
};

struct AssociationParam {
  AssociationMethod method = AssociationMethod::GREEDY_GROUP_CONNECT;
  ScalarType min_distance = 0.0;
  ScalarType max_distance = 100.0;
  ScalarType distance_weight = 0.4;
  ScalarType max_deviation_angle = static_cast<ScalarType>(M_PI / 6.0);
  ScalarType deviation_angle_weight = 0.4;
  ScalarType max_relative_orie = static_cast<ScalarType>(M_PI / 6.0);
  ScalarType relative_orie_weight = 0.2;
  ScalarType max_departure_distance = 50.0;
  ScalarType departure_distance_weight = 0.4;
  ScalarType min_orientation_estimation_size = 5.0;
};

struct Marker {
  MarkerShapeType shape_type = MarkerShapeType::LINE_SEGMENT;
  int marker_type = 0;
  SpaceType space_type = SpaceType::IMAGE;
  Vector2D pos{0.0, 0.0};
  Vector2D image_pos{0.0, 0.0};
  Vector2D start_pos{0.0, 0.0};
  Vector2D image_start_pos{0.0, 0.0};
  Vector2D orie{0.0, -1.0};
  ScalarType angle = static_cast<ScalarType>(-M_PI / 2.0);
  int original_id = -1;
  int cc_id = -1;
  int inner_edge_id = -1;
  int cc_edge_ascend_id = -1;
  int cc_edge_descend_id = -1;
  int cc_next_marker_id = -1;
  int lane_id = -1;
  ScalarType confidence = 0.0;

  cv::Point vis_pos;
  cv::Point vis_start_pos;

  static bool comp(const Marker &a, const Marker &b) {
    CHECK_EQ(a.space_type, b.space_type);
    return (a.space_type == SpaceType::IMAGE) ? a.pos(1) > b.pos(1)
                                              : a.pos(1) < b.pos(1);
  }
};

typedef Eigen::Matrix<ScalarType, 4, 1> Bbox;  // (x_min, y_min, x_max, y_max)

typedef std::vector<std::pair<int, int>> Graph;

enum SpatialLabelType {
  L_0 = 0,
  L_1,
  L_2,
  R_0,
  R_1,
  R_2,
  L_UNKNOWN,
  R_UNKNOWN,
};

enum SemanticLabelType {
  SOLID = 0,
  DASHED,
  PARRELLE,
  UNKNOWN,
};

typedef Eigen::Matrix<ScalarType, MAX_POLY_ORDER + 1, 1> PolyModel;

struct LaneInstance {
  int graph_id;
  ScalarType siz;
  Bbox bounds;
  PolyModel model;
  ScalarType lateral_dist;

  LaneInstance() : graph_id(-1), siz(0), lateral_dist(0) {
    for (int j = 0; j <= MAX_POLY_ORDER; ++j) {
      this->model(j) = 0;
    }
    bounds << 0, 0, 0, 0;
  }

  LaneInstance(int i, ScalarType s, const Bbox &box)
      : graph_id(i), siz(s), lateral_dist(0) {
    for (int j = 0; j <= MAX_POLY_ORDER; ++j) {
      model(j) = 0;
    }
    bounds = box;
  }

  static bool CompareSiz(const LaneInstance &a, const LaneInstance &b) {
    return a.siz > b.siz;
  }

  static bool CompareBound(const LaneInstance &a, const LaneInstance &b) {
    return a.bounds(0) < b.bounds(0);  // x_min
  }

  bool HasOverlap(const LaneInstance &a) {
    return a.bounds(0) <= this->bounds(2) && a.bounds(2) >= this->bounds(0);
  }
};

struct CubicCurve {
  float x_start;
  float x_end;
  float a;
  float b;
  float c;
  float d;
};

struct LaneObject {
  LaneObject() {
    model.setZero();
    pos.reserve(100);
    orie.reserve(100);
    image_pos.reserve(100);
    confidence.reserve(100);
  }

  size_t point_num = 0;
  std::vector<Vector2D> pos;
  std::vector<Vector2D> orie;
  std::vector<Vector2D> image_pos;
  std::vector<ScalarType> confidence;
  SpatialLabelType spatial = SpatialLabelType::L_0;
  SemanticLabelType semantic = SemanticLabelType::UNKNOWN;
  bool is_compensated = false;

  ScalarType longitude_start = std::numeric_limits<ScalarType>::max();
  ScalarType longitude_end = -std::numeric_limits<ScalarType>::max();
  int order = 0;
  PolyModel model;
  ScalarType lateral_distance = 0.0;

  CubicCurve pos_curve;
  CubicCurve img_curve;
  // L3LaneInfo lane_info;
  double timestamp = 0.0;
  int32_t seq_num = 0;

  // @brief: write to LaneMarker protobuf message API
  void ToLaneMarkerProto(LaneMarker *lane_marker) const {
    // set a constant quality value 1.0 as temporary use
    lane_marker->set_quality(1.0);
    lane_marker->set_model_degree(MAX_POLY_ORDER);
    lane_marker->set_c0_position(model(0));
    lane_marker->set_c1_heading_angle(model(1));
    lane_marker->set_c2_curvature(model(2));
    lane_marker->set_c3_curvature_derivative(model(3));
    lane_marker->set_view_range(
        std::max(longitude_end, static_cast<ScalarType>(0)));
    lane_marker->set_longitude_start(longitude_start);
    lane_marker->set_longitude_end(longitude_end);
  }

  std::string GetSpatialLabel() const {
    switch (spatial) {
      case SpatialLabelType::L_0:
        return "L0";
      case SpatialLabelType::L_1:
        return "L1";
      case SpatialLabelType::L_2:
        return "L2";
      case SpatialLabelType::R_0:
        return "R0";
      case SpatialLabelType::R_1:
        return "R1";
      case SpatialLabelType::R_2:
        return "R2";
      default:
        AERROR << "unknown lane spatial label.";
        return "unknown spatial label";
    }
  }
};

typedef std::vector<LaneObject> LaneObjects;
typedef std::shared_ptr<LaneObjects> LaneObjectsPtr;
typedef const std::shared_ptr<LaneObjects> LaneObjectsConstPtr;

typedef std::vector<LaneInstance> LaneInstances;
typedef std::shared_ptr<LaneInstances> LaneInstancesPtr;
typedef const std::shared_ptr<LaneInstances> LaneInstancesConstPtr;

void LaneObjectsToLaneMarkerProto(const LaneObjects &lane_objects,
                                  LaneMarkers *lane_markers);
}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_CAMERA_LANE_POST_PROCESS_COMMON_TYPE_H_
