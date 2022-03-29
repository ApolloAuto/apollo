/******************************************************************************
 * Copyright 2021 The Apollo Authors. All Rights Reserved.
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
#include "modules/prediction/pipeline/vector_net.h"

#include <cmath>
#include <limits>
#include <unordered_set>
#include <utility>

#include "cyber/common/file.h"

namespace apollo {
namespace prediction {
template <typename Points>
void VectorNet::GetOnePolyline(
    const Points& points, double* start_length,
    const common::PointENU& center_point, const double obstacle_phi,
    ATTRIBUTE_TYPE attr_type, BOUNDARY_TYPE bound_type, const int count,
    std::vector<std::vector<double>>* const one_polyline,
    std::vector<double>* const one_p_id) {
  size_t size = points.size();
  std::vector<double> s(size, 0);

  for (size_t i = 1; i < size; ++i) {
    s[i] = std::hypot(points.at(i).x() - points.at(i - 1).x(),
                      points.at(i).y() - points.at(i - 1).y()) +
           s[i - 1];
  }

  std::vector<double> x;
  std::vector<double> y;
  double cur_length = *start_length;

  auto it_lower = std::lower_bound(s.begin(), s.end(), cur_length);
  while (it_lower != s.end()) {
    if (it_lower == s.begin()) {
      x.push_back(points.at(0).x());
      y.push_back(points.at(0).y());
    } else {
      const auto distance = std::distance(s.begin(), it_lower);
      x.push_back(common::math::lerp(points.at(distance - 1).x(),
                                     s[distance - 1], points.at(distance).x(),
                                     s[distance], cur_length));
      y.push_back(common::math::lerp(points.at(distance - 1).y(),
                                     s[distance - 1], points.at(distance).y(),
                                     s[distance], cur_length));
    }
    cur_length += FLAGS_point_distance;
    it_lower = std::lower_bound(s.begin(), s.end(), cur_length);
  }
  size_t point_size = x.size();

  *start_length = cur_length - s[size - 1];
  if (point_size == 0) return;
  const double attr = attribute_map.at(attr_type);
  const double bound = boundary_map.at(bound_type);
  auto last_point_after_rotate = common::math::RotateVector2d(
      {x[0] - center_point.x(), y[0] - center_point.y()},
      M_PI_2 - obstacle_phi);

  for (size_t i = 1; i < point_size; ++i) {
    if (one_p_id->at(0) > last_point_after_rotate.x()) {
      one_p_id->at(0) = last_point_after_rotate.x();
    }
    if (one_p_id->at(1) > last_point_after_rotate.y()) {
      one_p_id->at(1) = last_point_after_rotate.y();
    }

    std::vector<double> one_vector;

    // d_s, d_e
    one_vector.push_back(last_point_after_rotate.x());
    one_vector.push_back(last_point_after_rotate.y());

    Eigen::Vector2d point_after_rotate = common::math::RotateVector2d(
        {x[i] - center_point.x(), y[i] - center_point.y()},
        M_PI_2 - obstacle_phi);

    one_vector.push_back(point_after_rotate.x());
    one_vector.push_back(point_after_rotate.y());
    last_point_after_rotate = std::move(point_after_rotate);

    // attribute
    one_vector.insert(one_vector.end(), {0.0, 0.0, attr, bound});

    one_vector.push_back(count);
    one_polyline->push_back(std::move(one_vector));
  }
}

bool VectorNet::query(const common::PointENU& center_point,
                      const double obstacle_phi,
                      FeatureVector* const feature_ptr,
                      PidVector* const p_id_ptr) {
  CHECK_NOTNULL(feature_ptr);
  count_ = 0;
  GetRoads(center_point, obstacle_phi, feature_ptr, p_id_ptr);
  GetLanes(center_point, obstacle_phi, feature_ptr, p_id_ptr);
  GetJunctions(center_point, obstacle_phi, feature_ptr, p_id_ptr);
  GetCrosswalks(center_point, obstacle_phi, feature_ptr, p_id_ptr);
  return true;
}

bool VectorNet::offline_query(const double obstacle_x, const double obstacle_y,
                              const double obstacle_phi) {
  return offline_query(obstacle_x,
                       obstacle_y,
                       obstacle_phi,
                       FLAGS_prediction_target_file);
}

bool VectorNet::offline_query(const double obstacle_x,
                              const double obstacle_y,
                              const double obstacle_phi,
                              const std::string file_name) {
  FeatureVector offline_feature;
  PidVector p_id;
  common::PointENU center_point =
      common::util::PointFactory::ToPointENU(obstacle_x, obstacle_y);
  query(center_point, obstacle_phi, &offline_feature, &p_id);

  apollo::prediction::VectorNetFeature vector_net_pb_;
  vector_net_pb_.mutable_car_position()->set_x(obstacle_x);
  vector_net_pb_.mutable_car_position()->set_y(obstacle_y);
  vector_net_pb_.mutable_car_position()->set_phi(obstacle_phi);

  size_t i = 0;
  for (const auto& polyline : offline_feature) {
    auto* polyline_pb = vector_net_pb_.add_polyline();
    polyline_pb->set_p_id_x(p_id[i][0]);
    polyline_pb->set_p_id_y(p_id[i][1]);
    i++;
    for (const auto& vector : polyline) {
      auto* vector_pb = polyline_pb->add_vector();
      for (const auto& element : vector) {
        vector_pb->add_element(element);
      }
    }
  }
  cyber::common::SetProtoToASCIIFile(vector_net_pb_,
                                     file_name);

  return true;
}

void VectorNet::GetRoads(const common::PointENU& center_point,
                         const double obstacle_phi,
                         FeatureVector* const feature_ptr,
                         PidVector* const p_id_ptr) {
  std::vector<apollo::hdmap::RoadInfoConstPtr> roads;
  apollo::hdmap::HDMapUtil::BaseMap().GetRoads(center_point,
                                               FLAGS_road_distance, &roads);

  for (const auto& road : roads) {
    for (const auto& section : road->road().section()) {
      for (const auto& edge : section.boundary().outer_polygon().edge()) {
        std::vector<std::vector<double>> one_polyline;
        std::vector<double> one_p_id{std::numeric_limits<float>::max(),
                                     std::numeric_limits<float>::max()};
        double start_length = 0;
        BOUNDARY_TYPE bound_type = UNKNOW;
        if (edge.type() == hdmap::BoundaryEdge::LEFT_BOUNDARY) {
          bound_type = LEFT_BOUNDARY;
        } else if (edge.type() == hdmap::BoundaryEdge::RIGHT_BOUNDARY) {
          bound_type = RIGHT_BOUNDARY;
        } else if (edge.type() == hdmap::BoundaryEdge::NORMAL) {
          bound_type = NORMAL;
        } else {
          bound_type = UNKNOW;
        }

        for (const auto& segment : edge.curve().segment()) {
          GetOnePolyline(segment.line_segment().point(), &start_length,
                         center_point, obstacle_phi, ROAD, bound_type, count_,
                         &one_polyline, &one_p_id);
        }
        if (one_polyline.size() == 0) continue;

        feature_ptr->push_back(std::move(one_polyline));
        p_id_ptr->push_back(std::move(one_p_id));
        ++count_;
      }
    }
  }
}

void VectorNet::GetLaneQueue(
    const std::vector<hdmap::LaneInfoConstPtr>& lanes,
    std::vector<std::deque<hdmap::LaneInfoConstPtr>>* const lane_deque_ptr) {
  std::unordered_set<hdmap::LaneInfoConstPtr> lane_set(lanes.begin(),
                                                       lanes.end());

  while (!lane_set.empty()) {
    std::deque<apollo::hdmap::LaneInfoConstPtr> one_lane_deque;
    auto cur_lane = *lane_set.begin();
    lane_set.erase(lane_set.begin());
    one_lane_deque.push_back(cur_lane);
    while (cur_lane->lane().successor_id_size() > 0) {
      auto id = cur_lane->lane().successor_id(0);
      cur_lane = apollo::hdmap::HDMapUtil::BaseMap().GetLaneById(id);
      if (lane_set.find(cur_lane) != lane_set.end()) {
        lane_set.erase(cur_lane);
        one_lane_deque.push_back(cur_lane);
      } else {
        break;
      }
    }

    cur_lane = one_lane_deque.front();
    while (cur_lane->lane().predecessor_id_size() > 0) {
      auto id = cur_lane->lane().predecessor_id(0);
      cur_lane = apollo::hdmap::HDMapUtil::BaseMap().GetLaneById(id);
      if (lane_set.find(cur_lane) != lane_set.end()) {
        lane_set.erase(cur_lane);
        one_lane_deque.push_front(cur_lane);
      } else {
        break;
      }
    }

    lane_deque_ptr->push_back(one_lane_deque);
  }
}

void VectorNet::GetLanes(const common::PointENU& center_point,
                         const double obstacle_phi,
                         FeatureVector* const feature_ptr,
                         PidVector* const p_id_ptr) {
  std::vector<apollo::hdmap::LaneInfoConstPtr> lanes;
  apollo::hdmap::HDMapUtil::BaseMap().GetLanes(center_point,
                                               FLAGS_road_distance, &lanes);

  std::vector<std::deque<apollo::hdmap::LaneInfoConstPtr>> lane_deque_vector;
  GetLaneQueue(lanes, &lane_deque_vector);

  for (const auto& lane_deque : lane_deque_vector) {
    // Draw lane's left_boundary
    std::vector<std::vector<double>> left_polyline;
    std::vector<double> left_p_id{std::numeric_limits<float>::max(),
                                  std::numeric_limits<float>::max()};
    double start_length = 0;
    for (const auto& lane : lane_deque) {
      std::cout << lane->lane().id().id() << " ";
      // if (lane->lane().left_boundary().virtual_()) continue;
      for (const auto& segment :
           lane->lane().left_boundary().curve().segment()) {
        auto bound_type =
            lane->lane().left_boundary().boundary_type(0).types(0);
        GetOnePolyline(segment.line_segment().point(), &start_length,
                       center_point, obstacle_phi, lane_attr_map.at(bound_type),
                       LEFT_BOUNDARY, count_, &left_polyline, &left_p_id);
      }
    }

    if (left_polyline.size() < 2) continue;
    feature_ptr->push_back(std::move(left_polyline));
    p_id_ptr->push_back(std::move(left_p_id));
    ++count_;

    std::vector<std::vector<double>> right_polyline;
    std::vector<double> right_p_id{std::numeric_limits<float>::max(),
                                   std::numeric_limits<float>::max()};
    start_length = 0;
    // Draw lane's right_boundary
    for (const auto& lane : lane_deque) {
      // if (lane->lane().right_boundary().virtual_()) continue;
      for (const auto& segment :
           lane->lane().right_boundary().curve().segment()) {
        auto bound_type =
            lane->lane().left_boundary().boundary_type(0).types(0);
        GetOnePolyline(segment.line_segment().point(), &start_length,
                       center_point, obstacle_phi, lane_attr_map.at(bound_type),
                       RIGHT_BOUNDARY, count_, &right_polyline, &right_p_id);
      }
    }

    if (right_polyline.size() < 2) continue;
    feature_ptr->push_back(std::move(right_polyline));
    p_id_ptr->push_back(std::move(right_p_id));
    ++count_;
  }
}

void VectorNet::GetJunctions(const common::PointENU& center_point,
                             const double obstacle_phi,
                             FeatureVector* const feature_ptr,
                             PidVector* const p_id_ptr) {
  std::vector<apollo::hdmap::JunctionInfoConstPtr> junctions;
  apollo::hdmap::HDMapUtil::BaseMap().GetJunctions(
      center_point, FLAGS_road_distance, &junctions);
  for (const auto& junction : junctions) {
    std::vector<std::vector<double>> one_polyline;
    std::vector<double> one_p_id{std::numeric_limits<float>::max(),
                                 std::numeric_limits<float>::max()};
    double start_length = 0;
    GetOnePolyline(junction->junction().polygon().point(), &start_length,
                   center_point, obstacle_phi, JUNCTION, UNKNOW, count_,
                   &one_polyline, &one_p_id);

    feature_ptr->push_back(std::move(one_polyline));
    p_id_ptr->push_back(std::move(one_p_id));
    ++count_;
  }
}

void VectorNet::GetCrosswalks(const common::PointENU& center_point,
                              const double obstacle_phi,
                              FeatureVector* const feature_ptr,
                              PidVector* const p_id_ptr) {
  std::vector<apollo::hdmap::CrosswalkInfoConstPtr> crosswalks;
  apollo::hdmap::HDMapUtil::BaseMap().GetCrosswalks(
      center_point, FLAGS_road_distance, &crosswalks);
  for (const auto& crosswalk : crosswalks) {
    std::vector<std::vector<double>> one_polyline;
    std::vector<double> one_p_id{std::numeric_limits<float>::max(),
                                 std::numeric_limits<float>::max()};
    double start_length = 0;
    GetOnePolyline(crosswalk->crosswalk().polygon().point(), &start_length,
                   center_point, obstacle_phi, CROSSWALK, UNKNOW, count_,
                   &one_polyline, &one_p_id);

    feature_ptr->push_back(std::move(one_polyline));
    p_id_ptr->push_back(std::move(one_p_id));
    ++count_;
  }
}
}  // namespace prediction
}  // namespace apollo
