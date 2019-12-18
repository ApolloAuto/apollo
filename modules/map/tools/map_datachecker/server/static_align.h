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
#pragma once

#include <memory>
#include <vector>

#include "modules/map/tools/map_datachecker/server/alignment.h"
#include "modules/map/tools/map_datachecker/server/common.h"

namespace apollo {
namespace hdmap {

enum class StaticAlignDetectMethod {
  RANSAC,
  DYNAMIC_CENTROID,
};

typedef struct Point3d {
  Point3d() : x(0.0), y(0.0), z(0.0) {}
  double x, y, z;
} Point3d;

typedef struct Centroid3D {
  Centroid3D() : count(0), start_time(0.0), end_time(0.0) {}
  Point3d center;
  int count;
  double start_time, end_time;
} Centroid3D;

class StaticAlign : public Alignment {
 public:
  explicit StaticAlign(std::shared_ptr<JSonConf> sp_conf);
  ErrorCode Process(const std::vector<FramePose>& poses);

 private:
  void Reset();
  double GetStaticAlignProgress(const std::vector<FramePose>& poses);
  double StaticAlignRansac(const std::vector<FramePose>& poses);
  double StaticAlignDynamicCentroid(const std::vector<FramePose>& poses);
  double GetCentroidTimeDuring();
  void UpdateDynamicCentroid(const FramePose& pose);
  bool IsStaticPose(const FramePose& pose);
  void UpdateGoodPoseInfo(const FramePose& pose);

 private:
  StaticAlignDetectMethod static_align_detect_method_;
  Centroid3D dynamic_centroid_;
};

}  // namespace hdmap
}  // namespace apollo
