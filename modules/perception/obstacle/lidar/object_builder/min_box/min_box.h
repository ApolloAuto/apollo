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

#ifndef MODULES_PERCEPTION_OBSTACLE_LIDAR_OBJECT_BUILDER_MIN_BOX_H
#define MODULES_PERCEPTION_OBSTACLE_LIDAR_OBJECT_BUILDER_MIN_BOX_H

#include <memory>
#include <string>
#include <vector>

#include "modules/perception/obstacle/base/object.h"
#include "modules/perception/obstacle/lidar/interface/base_object_builder.h"

namespace apollo {
namespace perception {

class MinBoxObjectBuilder : public BaseObjectBuilder {
 public:
  MinBoxObjectBuilder() : BaseObjectBuilder() {}
  virtual ~MinBoxObjectBuilder() {}

  bool Init() override { return true; }

  bool Build(const ObjectBuilderOptions& options,
             std::vector<std::shared_ptr<Object>>* objects) override;
  std::string name() const override { return "MinBoxObjectBuilder"; }

 protected:
  void BuildObject(ObjectBuilderOptions options,
                   std::shared_ptr<Object> object);

  void ComputePolygon2dxy(std::shared_ptr<Object> obj);

  double ComputeAreaAlongOneEdge(std::shared_ptr<Object> obj,
                                 size_t first_in_point, Eigen::Vector3d* center,
                                 double* lenth, double* width,
                                 Eigen::Vector3d* dir);

  void ReconstructPolygon(const Eigen::Vector3d& ref_ct,
                          std::shared_ptr<Object> obj);

  void ComputeGeometricFeature(const Eigen::Vector3d& ref_ct,
                               std::shared_ptr<Object> obj);

 private:
  DISALLOW_COPY_AND_ASSIGN(MinBoxObjectBuilder);
};

// Register plugin.
REGISTER_OBJECTBUILDER(MinBoxObjectBuilder);

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_LIDAR_OBJECT_BUILDER_MIN_BOX_H
