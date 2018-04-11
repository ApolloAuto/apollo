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
#ifndef MODULES_PERCEPTION_OBSTACLE_LIDAR_INTERFACE_BASE_OBJECT_FILTER_H_
#define MODULES_PERCEPTION_OBSTACLE_LIDAR_INTERFACE_BASE_OBJECT_FILTER_H_

// SAMPLE CODE:
//
// class MyObjectFilter : public BaseObjectFilter {
// public:
//     MyObjectFilter() : BaseObjectFilter() {}
//     virtual ~MyObjectFilter() {}
//
//     virtual bool init() override {
//         // Do something.
//         return true;
//     }
//
//     virtual bool filter(
//             const ObjectFilterOptions& obj_filter_options,
//             std::vector<std::shared_ptr<Object>>& objects) override {
//
//          // Do something.
//          return true;
//      }
//
//      virtual std::string name() const override {
//          return "MyObjectFilter";
//      }
//
// };
//
// // Register plugin.
// REGISTER_GROUNDDETECTOR(MyObjectFilter);
////////////////////////////////////////////////////////
// USING CODE:
//
// BaseObjectFilter* roi_filter =
//          BaseObjectFilterRegisterer::get_instance_by_name("MyObjectFilter");
// using roi_filter to do somethings.
// ////////////////////////////////////////////////////

#include <memory>
#include <string>
#include <vector>

#include "modules/common/macro.h"
#include "modules/perception/lib/base/registerer.h"
#include "modules/perception/obstacle/base/hdmap_struct.h"
#include "modules/perception/obstacle/base/object.h"

namespace apollo {
namespace perception {

struct ObjectFilterOptions {
  ObjectFilterOptions() = default;
  explicit ObjectFilterOptions(Eigen::Matrix4d *pose) : velodyne_trans(pose) {}

  std::shared_ptr<const Eigen::Matrix4d> velodyne_trans = nullptr;

  HdmapStructConstPtr hdmap = nullptr;
};

class BaseObjectFilter {
 public:
  BaseObjectFilter() {}
  virtual ~BaseObjectFilter() {}

  virtual bool Init() = 0;

  virtual bool Filter(const ObjectFilterOptions &obj_filter_options,
                      std::vector<std::shared_ptr<Object>> *objects) = 0;

  virtual std::string name() const = 0;

 private:
  DISALLOW_COPY_AND_ASSIGN(BaseObjectFilter);
};

REGISTER_REGISTERER(BaseObjectFilter);
#define REGISTER_OBJECTFILTER(name) REGISTER_CLASS(BaseObjectFilter, name)

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_LIDAR_INTERFACE_BASE_OBJECT_FILTER_H_
