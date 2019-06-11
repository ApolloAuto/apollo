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
#ifndef  _MODULES_MAP_TOOLS_MAP_DATACHECKER_STATIC_ALIGN_H
#define _MODULES_MAP_TOOLS_MAP_DATACHECKER_STATIC_ALIGN_H

#include <memory>
#include <vector>
#include "modules/map/tools/map_datachecker/common.hpp"
#include "modules/map/tools/map_datachecker/alignment.hpp"

namespace apollo {
namespace hdmap {

enum class StaticAlignDetectMethod {
  RANSAC,
  DYNAMIC_CENTROID,
};

typedef struct Point3d {
  Point3d():x(0.0), y(0.0), z(0.0) {}
  double x, y, z;
} Point3d;

typedef struct Centroid3D {
  Centroid3D(): count(0), start_time(0.0), end_time(0.0) {}
  Point3d center;
  int count;
  double start_time, end_time;
} Centroid3D;


class StaticAlign: public Alignment {
 public:
  explicit StaticAlign(std::shared_ptr<JSonConf> sp_conf);
  ErrorCode process(const std::vector<FramePose>& poses);

 private:
  void reset();
  double get_static_align_progress(const std::vector<FramePose> & poses);
  double static_align_ransac(const std::vector<FramePose> & poses);
  double static_align_dynamic_centroid(const std::vector<FramePose> & poses);
  double get_centroid_time_during();
  void update_dynamic_centroid(const FramePose& pose);
  bool is_static_pose(const FramePose& pose);
  void update_good_pose_info(const FramePose& pose);

 private:
  StaticAlignDetectMethod _static_align_detect_method;
  Centroid3D _dynamic_centroid;
};

}  // namespace hdmap
}  // namespace apollo

#endif  // #ifndef _MODULES_MAP_TOOLS_MAP_DATACHECKER_STATIC_ALIGN_H
