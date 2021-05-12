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

#include <limits>
#include <map>
#include <utility>

#include "cyber/common/file.h"
#include "modules/map/hdmap/hdmap_util.h"

namespace apollo {
namespace prediction {
namespace {
enum ATTRIBUTE_TYPE {
  ROAD,
  LANE,
  JUNCTION,
  CROSSWALK,
};

std::map<ATTRIBUTE_TYPE, std::vector<double>> attribute_map{
    {ROAD, {1, 0, 0, 0, 0}},
    {LANE, {0, 1, 0, 0, 0}},
    {JUNCTION, {0, 0, 1, 0, 0}},
    {CROSSWALK, {0, 0, 0, 1, 0}}};

template <typename Points>
void GetOnePolyline(const Points& points, const common::PointENU& center_point,
                    const double obstacle_phi, ATTRIBUTE_TYPE attr_type,
                    const int count,
                    std::vector<std::vector<double>>* const one_polyline,
                    std::vector<double>* const one_p_id) {
  size_t size = points.size();
  if (size < 2) return;

  const common::PointENU& last_point = points.at(0);
  auto last_point_after_rotate = common::math::RotateVector2d(
      {last_point.x() - center_point.x(), last_point.y() - center_point.y()},
      M_PI_2 - obstacle_phi);
  for (size_t i = 1; i < size; ++i) {
    if (std::fabs(one_p_id->at(0)) > std::fabs(last_point_after_rotate.x())) {
      one_p_id->at(0) = last_point_after_rotate.x();
    }
    if (std::fabs(one_p_id->at(1)) > std::fabs(last_point_after_rotate.y())) {
      one_p_id->at(1) = last_point_after_rotate.y();
    }

    // TODO(Yiqun):
    // check the segments are discretized with the same interval
    std::vector<double> one_vector;

    // d_s, d_e
    one_vector.push_back(last_point_after_rotate.x());
    one_vector.push_back(last_point_after_rotate.y());
    auto& point = points.at(i);
    Eigen::Vector2d point_after_rotate = common::math::RotateVector2d(
        {point.x() - center_point.x(), point.y() - center_point.y()},
        M_PI_2 - obstacle_phi);
    one_vector.push_back(point_after_rotate.x());
    one_vector.push_back(point_after_rotate.y());
    last_point_after_rotate = std::move(point_after_rotate);

    std::vector<double>& attr = attribute_map[attr_type];
    // attribute
    one_vector.insert(one_vector.end(), attr.begin(), attr.end());

    one_vector.push_back(count);
    one_polyline->push_back(std::move(one_vector));
  }
}
}  // namespace

bool VectorNet::query(const common::PointENU& center_point,
                      const double obstacle_phi,
                      FeatureVector* const feature_ptr,
                      PidVector* const p_id_ptr) {
  CHECK_NOTNULL(feature_ptr);
  count_ = 0;
  apollo::hdmap::HDMapUtil::ReloadMaps();
  GetRoads(center_point, obstacle_phi, feature_ptr, p_id_ptr);
  GetLanes(center_point, obstacle_phi, feature_ptr, p_id_ptr);
  GetJunctions(center_point, obstacle_phi, feature_ptr, p_id_ptr);
  GetCrosswalks(center_point, obstacle_phi, feature_ptr, p_id_ptr);
  return true;
}

bool VectorNet::offline_query(const double obstacle_x, const double obstacle_y,
                              const double obstacle_phi) {
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
                                     FLAGS_prediction_target_file);

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

        for (const auto& segment : edge.curve().segment()) {
          GetOnePolyline(segment.line_segment().point(), center_point,
                         obstacle_phi, ROAD, count_, &one_polyline, &one_p_id);
        }
        feature_ptr->push_back(std::move(one_polyline));
        p_id_ptr->push_back(std::move(one_p_id));
        ++count_;
      }
    }
  }
}

void VectorNet::GetLanes(const common::PointENU& center_point,
                         const double obstacle_phi,
                         FeatureVector* const feature_ptr,
                         PidVector* const p_id_ptr) {
  std::vector<apollo::hdmap::LaneInfoConstPtr> lanes;
  apollo::hdmap::HDMapUtil::BaseMap().GetLanes(center_point,
                                               FLAGS_road_distance, &lanes);
  for (const auto& lane : lanes) {
    // Get lane_central first
    for (const auto& segment : lane->lane().central_curve().segment()) {
      std::vector<std::vector<double>> one_polyline;
      std::vector<double> one_p_id{std::numeric_limits<float>::max(),
                                   std::numeric_limits<float>::max()};

      GetOnePolyline(segment.line_segment().point(), center_point, obstacle_phi,
                     LANE, count_, &one_polyline, &one_p_id);
      feature_ptr->push_back(std::move(one_polyline));
      p_id_ptr->push_back(std::move(one_p_id));
      ++count_;
    }

    // Not drawing boundary for virtual city_driving lane
    if (lane->lane().type() == 2 && lane->lane().left_boundary().virtual_() &&
        lane->lane().right_boundary().virtual_()) {
      continue;
    }
    // Draw lane's left_boundary
    for (const auto& segment : lane->lane().left_boundary().curve().segment()) {
      std::vector<std::vector<double>> one_polyline;
      std::vector<double> one_p_id{std::numeric_limits<float>::max(),
                                   std::numeric_limits<float>::max()};

      GetOnePolyline(segment.line_segment().point(), center_point, obstacle_phi,
                     LANE, count_, &one_polyline, &one_p_id);
      feature_ptr->push_back(std::move(one_polyline));
      p_id_ptr->push_back(std::move(one_p_id));
      ++count_;
    }

    // Draw lane's right_boundary
    for (const auto& segment :
         lane->lane().right_boundary().curve().segment()) {
      std::vector<std::vector<double>> one_polyline;
      std::vector<double> one_p_id{std::numeric_limits<float>::max(),
                                   std::numeric_limits<float>::max()};

      GetOnePolyline(segment.line_segment().point(), center_point, obstacle_phi,
                     LANE, count_, &one_polyline, &one_p_id);
      feature_ptr->push_back(std::move(one_polyline));
      p_id_ptr->push_back(std::move(one_p_id));
      ++count_;
    }
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
    GetOnePolyline(junction->junction().polygon().point(), center_point,
                   obstacle_phi, JUNCTION, count_, &one_polyline, &one_p_id);

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
    GetOnePolyline(crosswalk->crosswalk().polygon().point(), center_point,
                   obstacle_phi, CROSSWALK, count_, &one_polyline, &one_p_id);

    feature_ptr->push_back(std::move(one_polyline));
    p_id_ptr->push_back(std::move(one_p_id));
    ++count_;
  }
}
}  // namespace prediction
}  // namespace apollo
