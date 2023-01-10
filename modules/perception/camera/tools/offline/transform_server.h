/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#pragma once

#include <map>
#include <set>
#include <string>
#include <vector>

#include "Eigen/Geometry"

#include "modules/common/util/eigen_defs.h"

namespace apollo {
namespace perception {
namespace camera {

struct Transform {
  double timestamp;
  double qw;
  double qx;
  double qy;
  double qz;
  double tx;
  double ty;
  double tz;
};

class TransformServer {
 public:
  TransformServer() {}
  ~TransformServer() {}

  inline const std::set<std::string> &vertices() { return vertices_; }

  bool Init(const std::vector<std::string> &camera_names,
            const std::string &params_path);

  bool AddTransform(const std::string &child_frame_id,
                    const std::string &frame_id,
                    const Eigen::Affine3d &transform);

  bool QueryTransform(const std::string &child_frame_id,
                      const std::string &frame_id, Eigen::Affine3d *transform);

  void print();

  bool LoadFromFile(const std::string &tf_input, float frequency = 200.0f);

  bool QueryPos(double timestamp, Eigen::Affine3d *pose);

 private:
  struct Edge {
    std::string child_frame_id;
    std::string frame_id;
    Eigen::Affine3d transform;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  std::vector<Transform> tf_;

  double error_limit_ = 1.0;
  // frame ids
  std::set<std::string> vertices_;

  // multimap from child frame id to frame id
  apollo::common::EigenMultiMap<std::string, Edge> edges_;

  bool FindTransform(const std::string &child_frame_id,
                     const std::string &frame_id, Eigen::Affine3d *transform,
                     std::map<std::string, bool> *visited);
};

}  // namespace camera
}  // namespace perception
}  // namespace apollo
