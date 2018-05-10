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
//              std::vector<std::shared_ptr<Object>>* objects) override {
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
// REGISTER_SEGMENTATION(CNNSegmentation);
////////////////////////////////////////////////////////
// USING CODE:
//
// BaseSegmentation* segmentation =
//    BaseSegmentationRegisterer::get_instance_by_name("CNNSegmentation");
// using segmentation to do somethings.
// ////////////////////////////////////////////////////

#include <memory>
#include <string>
#include <vector>

#include "modules/common/macro.h"
#include "modules/perception/common/pcl_types.h"
#include "modules/perception/lib/base/registerer.h"
#include "modules/perception/obstacle/base/object.h"

namespace apollo {
namespace perception {

struct SegmentationOptions {
  // original point cloud without ROI filtering
  pcl_util::PointCloudPtr origin_cloud;
  // indices of roi-filtered cloud in original cloud if enabled
  pcl_util::PointIndicesPtr roi_cloud_indices;
  // indices of non-ground points in original clound if enabled
  pcl_util::PointIndicesPtr non_ground_indices;
};

class BaseSegmentation {
 public:
  BaseSegmentation() {}
  virtual ~BaseSegmentation() {}

  virtual bool Init() = 0;

  // @brief: segment the point cloud.
  // @param [in]: input point cloud.
  // @param [in]: valid indices of points for segmentation.
  // @param [in]: segmentation options
  // @param [out]: segmented object.
  virtual bool Segment(const pcl_util::PointCloudPtr &cloud,
                       const pcl_util::PointIndices &valid_indices,
                       const SegmentationOptions &options,
                       std::vector<std::shared_ptr<Object>> *objects) = 0;

  virtual std::string name() const = 0;

 private:
  DISALLOW_COPY_AND_ASSIGN(BaseSegmentation);
};

REGISTER_REGISTERER(BaseSegmentation);
#define REGISTER_SEGMENTATION(name) REGISTER_CLASS(BaseSegmentation, name)

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_LIDAR_INTERFACE_BASE_SEGMENTATION_H_
