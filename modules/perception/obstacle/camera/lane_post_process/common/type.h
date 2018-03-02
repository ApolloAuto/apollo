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

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <algorithm>
#include <limits>

#include "modules/common/log.h"
// #include "modules/perception/obstacle/camera/
//           lane_post_process/common/projector.h"

#ifndef MODULES_PERCEPTION_OBSTACLE_CAMERA_LANE_POST_PROCESS_COMMON_TYPE_H_
#define MODULES_PERCEPTION_OBSTACLE_CAMERA_LANE_POST_PROCESS_COMMON_TYPE_H_

namespace apollo {
namespace perception {

// #ifndef DEBUG
// #define DEBUG true
// #endif

#ifndef M_PI
#define M_PI REAL(3.1415926535897932384626433832795029)
#endif

#ifndef UF_BLOCK_WIDTH
#define UF_BLOCK_WIDTH 32
#endif

#ifndef UF_BLOCK_HEIGHT
#define UF_BLOCK_HEIGHT 16
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

typedef float ScalarType;

const ScalarType kEpsilon = 1e-5;

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
const int kDelayTime(0);

enum MarkerShapeType {
  POINT = 0,
  LINE_SEGMENT,
  POLYNOMIAL,
};

enum SpaceType {
  IMAGE = 0,
  EGO_CAR,
  IPM,
  VEHICLE,
};

typedef Eigen::Matrix<ScalarType, 2, 1> Vector2D;

enum AssociationMethod {
  GREEDY_SEARCH = 0,
  GREEDY_SLIDING_WINDOW,
  GREEDY_GROUP_CONNECT,
};

struct AssociationParam {
  AssociationMethod method;
  ScalarType min_distance;
  ScalarType max_distance;
  ScalarType distance_weight;
  ScalarType max_deviation_angle;
  ScalarType deviation_angle_weight;
  ScalarType max_relative_orie;
  ScalarType relative_orie_weight;
  ScalarType max_departure_distance;
  ScalarType departure_distance_weight;
  ScalarType min_orientation_estimation_size;

  AssociationParam()
      : method(AssociationMethod::GREEDY_SEARCH),
        min_distance(0.0),
        max_distance(100.0),
        distance_weight(0.4),
        max_deviation_angle(static_cast<ScalarType>(M_PI / 6.0)),
        deviation_angle_weight(0.4),
        max_relative_orie(static_cast<ScalarType>(M_PI / 6.0)),
        relative_orie_weight(0.2),
        max_departure_distance(50.0),
        departure_distance_weight(0.4),
        min_orientation_estimation_size(5.0) {}
};

struct Marker {
  MarkerShapeType shape_type;
  int marker_type;
  SpaceType space_type;
  Vector2D pos;
  Vector2D image_pos;
  Vector2D start_pos;
  Vector2D image_start_pos;
  Vector2D orie;
  ScalarType angle;
  int original_id;
  int cc_id;
  int inner_edge_id;
  int cc_edge_ascend_id;
  int cc_edge_descend_id;
  int cc_next_marker_id;
  int lane_id;
  ScalarType confidence;

  cv::Point vis_pos;
  cv::Point vis_start_pos;

  Marker()
      : shape_type(MarkerShapeType::LINE_SEGMENT),
        marker_type(0),
        space_type(SpaceType::IMAGE),
        pos(0.0, 0.0),
        image_pos(0.0, 0.0),
        start_pos(0.0, 0.0),
        image_start_pos(0.0, 0.0),
        orie(0.0, -1.0),
        angle(static_cast<ScalarType>(-M_PI / 2.0)),
        original_id(-1),
        cc_id(-1),
        inner_edge_id(-1),
        cc_edge_ascend_id(-1),
        cc_edge_descend_id(-1),
        cc_next_marker_id(-1),
        lane_id(-1),
        confidence(0.0) {}

  static bool comp(const Marker &a, const Marker &b) {
    assert(a.space_type == b.space_type);
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

  static bool compare_siz(const LaneInstance &a, const LaneInstance &b) {
    return a.siz > b.siz;
  }

  static bool compare_bound(const LaneInstance &a, const LaneInstance &b) {
    return a.bounds(0) < b.bounds(0);  // x_min
  }

  bool has_overlap(const LaneInstance &a) {
    return a.bounds(0) <= this->bounds(2) && a.bounds(2) >= this->bounds(0);
  }
};

struct CCLanePoint {
  Vector2D pos;
  Vector2D orie;
  ScalarType angle;
  Vector2D image_pos;
  ScalarType confidence;
  int frame_id;
  ScalarType score;
  int point_id;
  Eigen::Matrix<ScalarType, 1, MAX_POLY_ORDER + 1> power_x;

  CCLanePoint()
      : pos(0.0, 0.0),
        orie(1.0, 0.0),
        angle(0.0),
        image_pos(0.0, 0.0),
        confidence(1.0),
        frame_id(-1),
        score(0.0),
        point_id(-1) {
    power_x.setZero();
  }

  static bool compare_score(const CCLanePoint &a, const CCLanePoint &b) {
    return a.score > b.score;
  }
};

struct CCLaneSegment {
  ScalarType start;
  ScalarType end;
  std::vector<std::shared_ptr<CCLanePoint>> points;
  PolyModel model;
  int order;

  CCLaneSegment()
      : start(std::numeric_limits<ScalarType>::max()),
        end(-std::numeric_limits<ScalarType>::max()) {
    model.setZero();
    order = 0;
  }

  void add_point(const std::shared_ptr<CCLanePoint> &p) {
    points.push_back(p);
    start = std::min(start, p->pos(0));
    end = std::max(end, p->pos(0));
  }
};

struct CCLaneTrack {
  SpaceType space_type;
  std::vector<CCLaneSegment> segments;
  size_t tot_point_num;
  Eigen::Matrix<ScalarType, Eigen::Dynamic, 1> y_values;
  ScalarType lateral_dist;
  Bbox bbox;

  CCLaneTrack() : space_type(SpaceType::EGO_CAR) {
    tot_point_num = 0;
    segments.clear();
    lateral_dist = 0.0;
    bbox << std::numeric_limits<ScalarType>::max(),  // x_min
        std::numeric_limits<ScalarType>::max(),      // y_min
        -std::numeric_limits<ScalarType>::max(),     // x_max
        -std::numeric_limits<ScalarType>::max();     // y_max
  }

  Bbox compute_bound() {
    bbox(0) = std::numeric_limits<ScalarType>::max();   // x_min
    bbox(1) = std::numeric_limits<ScalarType>::max();   // y_min
    bbox(2) = -std::numeric_limits<ScalarType>::max();  // x_max
    bbox(3) = -std::numeric_limits<ScalarType>::max();  // y_max

    for (auto it_segment = segments.begin(); it_segment != segments.end();
         ++it_segment) {
      for (size_t i = 0; i < it_segment->points.size(); ++i) {
        bbox(0) = std::min(it_segment->points[i]->pos(0), bbox(0));
        bbox(1) = std::min(it_segment->points[i]->pos(1), bbox(1));
        bbox(2) = std::max(it_segment->points[i]->pos(0), bbox(2));
        bbox(3) = std::max(it_segment->points[i]->pos(1), bbox(3));
      }
    }
    return bbox;
  }

  ScalarType size() const {
    return std::max(bbox(2) - bbox(0), bbox(3) - bbox(1));
  }

  ScalarType x_min() const {
    return bbox(0);
  }
  ScalarType y_min() const {
    return bbox(1);
  }
  ScalarType x_max() const {
    return bbox(2);
  }
  ScalarType y_max() const {
    return bbox(3);
  }
};

typedef std::shared_ptr<CCLaneTrack> CCLaneTrackPtr;
typedef const std::shared_ptr<CCLaneTrack> CCLaneTrackConstPtr;

struct L3CubicCurve {
  float x_start;
  float x_end;
  float a;
  float b;
  float c;
  float d;
};

struct L3LaneInfo {
  int lane_id;
  int left_idx;
  int right_idx;
  float lane_width;
  int carleft_idx;
  int carright_idx;
};

struct LaneObject {
  size_t point_num;
  std::vector<Vector2D> pos;
  std::vector<Vector2D> orie;
  std::vector<Vector2D> image_pos;
  std::vector<ScalarType> confidence;
  SpatialLabelType spatial;
  SemanticLabelType semantic;
  bool is_compensated;

  ScalarType longitude_start;
  ScalarType longitude_end;
  int order;
  PolyModel model;
  ScalarType lateral_distance;

  L3CubicCurve pos_curve;
  L3CubicCurve img_curve;
  L3LaneInfo lane_info;
  double timestamp;
  int seq_num;

  LaneObject()
      : point_num(0),
        spatial(SpatialLabelType::L_0),
        semantic(SemanticLabelType::UNKNOWN),
        is_compensated(false),
        longitude_start(std::numeric_limits<ScalarType>::max()),
        longitude_end(-std::numeric_limits<ScalarType>::max()),
        order(0),
        lateral_distance(0.0) {
    model.setZero();
    pos.reserve(100);
    orie.reserve(100);
    image_pos.reserve(100);
    confidence.reserve(100);
  }

  std::string get_spatial_label() const {
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

  /*
  void copy_to(LaneObject& new_lane_object) {
    new_lane_object.point_num = point_num;
    new_lane_object.spatial = spatial;
    new_lane_object.semantic = semantic;
    new_lane_object.is_compensated = is_compensated;

    for (size_t i = 0; i < new_lane_object.point_num; ++i) {
      new_lane_object.pos.push_back(pos[i]);
      new_lane_object.orie.push_back(orie[i]);
      new_lane_object.image_pos.push_back(image_pos[i]);
      new_lane_object.confidence.push_back(confidence[i]);
    }

    new_lane_object.longitude_start = longitude_start;
    new_lane_object.longitude_end = longitude_end;
    new_lane_object.order = order;
    for (int i = 0; i <= MAX_POLY_ORDER; ++i) {
      new_lane_object.model(i) = model(i);
    }
    new_lane_object.lateral_distance = lateral_distance;

    new_lane_object.pos_curve.a = pos_curve.a;
    new_lane_object.pos_curve.b = pos_curve.b;
    new_lane_object.pos_curve.c = pos_curve.c;
    new_lane_object.pos_curve.d = pos_curve.d;
    new_lane_object.pos_curve.x_start = pos_curve.x_start;
    new_lane_object.pos_curve.x_end = pos_curve.x_end;

    new_lane_object.img_curve.a = img_curve.a;
    new_lane_object.img_curve.b = img_curve.b;
    new_lane_object.img_curve.c = img_curve.c;
    new_lane_object.img_curve.d = img_curve.d;
    new_lane_object.img_curve.x_start = img_curve.x_start;
    new_lane_object.img_curve.x_end = img_curve.x_end;

    new_lane_object.lane_info.lane_id = lane_info.lane_id;
    new_lane_object.lane_info.left_idx = lane_info.left_idx;
    new_lane_object.lane_info.right_idx = lane_info.right_idx;
    new_lane_object.lane_info.lane_width = lane_info.lane_width;
    new_lane_object.lane_info.carleft_idx = lane_info.carleft_idx;
    new_lane_object.lane_info.carright_idx = lane_info.carright_idx;

    new_lane_object.timestamp = timestamp;
    new_lane_object.seq_num = seq_num;
  }
  */

  void copy_to(LaneObject* new_lane_object) {
    new_lane_object->point_num = point_num;
    new_lane_object->spatial = spatial;
    new_lane_object->semantic = semantic;
    new_lane_object->is_compensated = is_compensated;

    for (size_t i = 0; i < new_lane_object->point_num; ++i) {
      new_lane_object->pos.push_back(pos[i]);
      new_lane_object->orie.push_back(orie[i]);
      new_lane_object->image_pos.push_back(image_pos[i]);
      new_lane_object->confidence.push_back(confidence[i]);
    }

    new_lane_object->longitude_start = longitude_start;
    new_lane_object->longitude_end = longitude_end;
    new_lane_object->order = order;
    for (int i = 0; i <= MAX_POLY_ORDER; ++i) {
      new_lane_object->model(i) = model(i);
    }
    new_lane_object->lateral_distance = lateral_distance;

    new_lane_object->pos_curve.a = pos_curve.a;
    new_lane_object->pos_curve.b = pos_curve.b;
    new_lane_object->pos_curve.c = pos_curve.c;
    new_lane_object->pos_curve.d = pos_curve.d;
    new_lane_object->pos_curve.x_start = pos_curve.x_start;
    new_lane_object->pos_curve.x_end = pos_curve.x_end;

    new_lane_object->img_curve.a = img_curve.a;
    new_lane_object->img_curve.b = img_curve.b;
    new_lane_object->img_curve.c = img_curve.c;
    new_lane_object->img_curve.d = img_curve.d;
    new_lane_object->img_curve.x_start = img_curve.x_start;
    new_lane_object->img_curve.x_end = img_curve.x_end;

    new_lane_object->lane_info.lane_id = lane_info.lane_id;
    new_lane_object->lane_info.left_idx = lane_info.left_idx;
    new_lane_object->lane_info.right_idx = lane_info.right_idx;
    new_lane_object->lane_info.lane_width = lane_info.lane_width;
    new_lane_object->lane_info.carleft_idx = lane_info.carleft_idx;
    new_lane_object->lane_info.carright_idx = lane_info.carright_idx;

    new_lane_object->timestamp = timestamp;
    new_lane_object->seq_num = seq_num;
  }
};

// struct for L3 Lane information
struct L3LaneLine {
  SpatialLabelType spatial;
  SemanticLabelType semantic;
  L3CubicCurve pos_curve;
  L3CubicCurve img_curve;
};
struct RoadInfo {
  double timestamp;
  double seq_num;
  std::vector<L3LaneLine> lane_line_vec;
  std::vector<L3LaneInfo> lane_vec;
};

/*
struct LaneDebugContent {
    std::vector<LaneObject> lane_objects;
    std::vector<LaneInstance> cur_lane_instances;
};
*/

typedef std::vector<LaneObject> LaneObjects;
typedef std::shared_ptr<LaneObjects> LaneObjectsPtr;
typedef const std::shared_ptr<LaneObjects> LaneObjectsConstPtr;

// typedef adu::perception::obstacle::transformer_tool::
//         Projector<ScalarType> Projector;

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_CAMERA_LANE_POST_PROCESS_COMMON_TYPE_H_
