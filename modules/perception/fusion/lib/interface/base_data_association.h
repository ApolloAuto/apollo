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
#pragma once

#include <string>
#include <utility>
#include <vector>

#include "modules/perception/fusion/base/base_forward_declaration.h"
#include "modules/perception/fusion/base/scene.h"
#include "modules/perception/fusion/base/sensor_frame.h"
#include "modules/perception/lib/registerer/registerer.h"

namespace apollo {
namespace perception {
namespace fusion {

struct AssociationOptions {};

typedef std::pair<size_t, size_t> TrackMeasurmentPair;

struct AssociationResult {
  std::vector<TrackMeasurmentPair> assignments;
  std::vector<size_t> unassigned_tracks;
  std::vector<size_t> unassigned_measurements;
  std::vector<double> track2measurements_dist;
  std::vector<double> measurement2track_dist;
};

class BaseDataAssociation {
 public:
  BaseDataAssociation() {}
  virtual ~BaseDataAssociation() {}
  BaseDataAssociation(const BaseDataAssociation&) = delete;
  BaseDataAssociation& operator=(const BaseDataAssociation&) = delete;

  virtual bool Init() = 0;

  // @brief: associate sensor measurements with global scene
  // @param [in]: options
  // @param [in]: sensor_measurements
  // @param [in]: scene
  // @param [out]: association_result
  virtual bool Associate(const AssociationOptions& options,
                         SensorFramePtr sensor_measurements, ScenePtr scene,
                         AssociationResult* association_result) = 0;

  virtual std::string Name() const = 0;
};

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
