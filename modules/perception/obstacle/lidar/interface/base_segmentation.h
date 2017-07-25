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
#ifndef MODULES_PERCEPTION_OBSTACLE_LIDAR_INTERFACE_BASE_SEGMENTATION_H_
#define MODULES_PERCEPTION_OBSTACLE_LIDAR_INTERFACE_BASE_SEGMENTATION_H_

// SAMPLE CODE:
//
// class NCutSegmentation : public BaseSegmentation {
// public:
//     NCutSegmentation() : BaseSegmentation() {}
//     virtual ~NCutSegmentation() {}
//
//     virtual bool init() override {
//         // Do something.
//         return true;
//     }
//
//     virtual bool segment(
//              const pcl_util::PointCloudPtr& cloud,
//              const pcl_util::PointIndices& non_ground_indices,
//              const SegmentationOptions& options,
//              std::vector<ObjectPtr>* objects) override {
//
//          // Do something.
//          return true;
//      }
//
//      virtual std::string name() const override {
//          return "NCutSegmentation";
//      }
//
// };
//
// // Register plugin.
// REGISTER_SEGMENTATION(NCutSegmentation);
////////////////////////////////////////////////////////
// USING CODE:
//
// BaseSegmentation* segmentation =
//    BaseSegmentationRegisterer::get_instance_by_name("NCutSegmentation");
// using segmentation to do somethings.
// ////////////////////////////////////////////////////

#include <string>
#include <vector>

#include "modules/common/macro.h"
#include "modules/perception/lib/base/registerer.h"
#include "modules/perception/lib/pcl_util/pcl_types.h"
#include "modules/perception/obstacle/base/object.h"

namespace apollo {
namespace perception {

struct SegmentationOptions {};

class BaseSegmentation {
 public:
  BaseSegmentation() {}
  virtual ~BaseSegmentation() {}

  virtual bool Init() = 0;

  // @brief: segment the point cloud.
  // @param [in]: input point cloud.
  // @param [in]: non ground points index.
  // @param [in]: some options
  // @param [out]: segmented object.
  virtual bool Segment(const pcl_util::PointCloudPtr &cloud,
                       const pcl_util::PointIndices &non_ground_indices,
                       const SegmentationOptions &options,
                       std::vector<ObjectPtr> *objects) = 0;

  virtual std::string name() const = 0;

 private:
  DISALLOW_COPY_AND_ASSIGN(BaseSegmentation);
};

REGISTER_REGISTERER(BaseSegmentation);
#define REGISTER_SEGMENTATION(name) REGISTER_CLASS(BaseSegmentation, name)

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_LIDAR_INTERFACE_BASE_SEGMENTATION_H_
