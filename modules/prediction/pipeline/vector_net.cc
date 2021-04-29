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
                      std::shared_ptr<FeatureVector> feature_ptr) {
  feature_ptr_ = feature_ptr;
  apollo::hdmap::HDMapUtil::ReloadMaps();
  GetRoads(obstacle_x, obstacle_y);

  return true;
}

bool VectorNet::offline_query(const double obstacle_x, const double obstacle_y,
                              const double obstacle_phi) {
  std::shared_ptr<FeatureVector> offline_feature_ptr =
      std::make_shared<FeatureVector>();
  query(obstacle_x, obstacle_y, obstacle_phi, offline_feature_ptr);

  vector_net_pb_.mutable_car_position()->set_x(obstacle_x);
  vector_net_pb_.mutable_car_position()->set_y(obstacle_y);
  for (const auto& polyline : *offline_feature_ptr) {
    auto polyline_ = vector_net_pb_.add_polyline();
    for (const auto& vector : polyline) {
      auto vector_ = polyline_->add_vector();
      for (const auto& element : vector) {
        vector_->add_element(element);
      }
    }
  }
  cyber::common::SetProtoToASCIIFile(vector_net_pb_,
                                     FLAGS_prediction_target_file);
  AINFO << "Obstacle heading: " << obstacle_phi;

  return true;
}

// make std::vectors here
void VectorNet::GetRoads(const double base_x, const double base_y) {
  common::PointENU center_point =
      common::util::PointFactory::ToPointENU(base_x, base_y);
  std::vector<apollo::hdmap::RoadInfoConstPtr> roads;
  apollo::hdmap::HDMapUtil::BaseMap().GetRoads(center_point, 141.4, &roads);
  AINFO << "Road Size: " << roads.size();

  for (const auto& road : roads) {
    std::vector<std::vector<double>> one_polyline;
    const auto& str_id = road->id().id();
    if (id_map_.find(str_id) == id_map_.end()) {
      id_map_[str_id] = static_cast<int>(id_map_.size());
    }
    int int_id = id_map_[str_id];

    for (const auto& section : road->road().section()) {
      for (const auto& edge : section.boundary().outer_polygon().edge()) {
        for (const auto& segment : edge.curve().segment()) {
          auto last_point = segment.line_segment().point(0);
          size_t size = segment.line_segment().point().size();
          for (size_t i = 1; i < size; ++i) {
            auto point = segment.line_segment().point(i);
            std::vector<double> one_vector;
            // d_s, d_e
            one_vector.push_back(last_point.x() - base_x);
            one_vector.push_back(last_point.y() - base_y);
            one_vector.push_back(point.x() - base_x);
            one_vector.push_back(point.y() - base_y);
            last_point = std::move(point);
            // attribute
            if (edge.type() == 2) {  // left edge
              one_vector.push_back(1);
              one_vector.push_back(0);
            } else if (edge.type() == 3) {  // right edge
              one_vector.push_back(0);
              one_vector.push_back(1);
            }
            one_vector.push_back(int_id);
            one_polyline.push_back(std::move(one_vector));
          }
        }
      }
    }
    feature_ptr_.get()->push_back(std::move(one_polyline));
  }
}
}  // namespace prediction
}  // namespace apollo
