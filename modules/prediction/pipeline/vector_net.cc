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

#include <utility>

#include "cyber/common/file.h"
#include "modules/common/util/point_factory.h"
#include "modules/map/hdmap/hdmap_util.h"

namespace apollo {
namespace prediction {
bool VectorNet::query(const double obstacle_x, const double obstacle_y,
                      const double obstacle_phi,
                      FeatureVector* const feature_ptr) {
  CHECK_NOTNULL(feature_ptr);
  count_ = 0;
  apollo::hdmap::HDMapUtil::ReloadMaps();
  GetRoads(obstacle_x, obstacle_y, obstacle_phi, feature_ptr);

  return true;
}

bool VectorNet::offline_query(const double obstacle_x, const double obstacle_y,
                              const double obstacle_phi) {
  FeatureVector offline_feature;
  query(obstacle_x, obstacle_y, obstacle_phi, &offline_feature);

  apollo::prediction::VectorNetFeature vector_net_pb_;
  vector_net_pb_.mutable_car_position()->set_x(obstacle_x);
  vector_net_pb_.mutable_car_position()->set_y(obstacle_y);
  vector_net_pb_.mutable_car_position()->set_phi(obstacle_phi);

  for (const auto& polyline : offline_feature) {
    auto* polyline_pb = vector_net_pb_.add_polyline();
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

void VectorNet::GetRoads(const double base_x, const double base_y,
                         const double obstacle_phi,
                         FeatureVector* const feature_ptr) {
  common::PointENU center_point =
      common::util::PointFactory::ToPointENU(base_x, base_y);
  std::vector<apollo::hdmap::RoadInfoConstPtr> roads;
  apollo::hdmap::HDMapUtil::BaseMap().GetRoads(center_point,
                                               FLAGS_road_distance, &roads);

  for (const auto& road : roads) {
    for (const auto& section : road->road().section()) {
      for (const auto& edge : section.boundary().outer_polygon().edge()) {
        std::vector<std::vector<double>> one_polyline;
        for (const auto& segment : edge.curve().segment()) {
          auto last_point = segment.line_segment().point(0);
          auto last_point_after_rotate = common::math::RotateVector2d(
              {last_point.x() - base_x, last_point.y() - base_y},
              M_PI_2 - obstacle_phi);
          size_t size = segment.line_segment().point().size();
          for (size_t i = 1; i < size; ++i) {
            // TODO(Yiqun):
            // check the segments are discretized with the same interval
            std::vector<double> one_vector;

            // d_s, d_e
            one_vector.push_back(last_point_after_rotate.x());
            one_vector.push_back(last_point_after_rotate.y());
            auto point = segment.line_segment().point(i);
            Eigen::Vector2d point_after_rotate = common::math::RotateVector2d(
                {point.x() - base_x, point.y() - base_y},
                M_PI_2 - obstacle_phi);
            one_vector.push_back(point_after_rotate.x());
            one_vector.push_back(point_after_rotate.y());
            last_point_after_rotate = std::move(point_after_rotate);

            // attribute
            one_vector.insert(one_vector.end(), {1, 0, 0, 0});

            one_vector.push_back(count_);
            one_polyline.push_back(std::move(one_vector));
          }
        }
        feature_ptr->push_back(std::move(one_polyline));
        ++count_;
      }
    }
  }
}
}  // namespace prediction
}  // namespace apollo
