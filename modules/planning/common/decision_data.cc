/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

/**
 * @file decision_data.cc
 **/

#include "modules/planning/common/decision_data.h"

namespace apollo {
namespace planning {

// TODO: complete data extracting functions here
const std::vector<Obstacle>& DecisionData::Obstacles() const {
    return obstacles_;
}

std::vector<Obstacle>* DecisionData::MutableObstacles() {
    return &obstacles_;
}

std::vector<const Obstacle*> DecisionData::StaticObstacles() const {
    return std::vector<const Obstacle*>();
}

std::vector<Obstacle*> DecisionData::MutableStaticObstacles() const {
    return std::vector<Obstacle*>();
}

std::vector<const Obstacle*> DecisionData::DynamicObstacles() const {
    return std::vector<const Obstacle*>();
}

std::vector<Obstacle*> DecisionData::MutableDynamicObstacles() const {
    return std::vector<Obstacle*>();
}

std::vector<const MapObject*> DecisionData::MapObjects() const {
    return std::vector<const MapObject*>();
}

std::vector<MapObject*> DecisionData::MutableMapObjects() const {
    return std::vector<MapObject*>();
}

} // namespace planning
} // namespace apollo

