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
#ifndef MODULES_PERCEPTION_OBSTACLE_LIDAR_INTERFACE_BASE_ROI_FILTER_H_
#define MODULES_PERCEPTION_OBSTACLE_LIDAR_INTERFACE_BASE_ROI_FILTER_H_
// SAMPLE CODE:
//
// class MyROIFilter : public BaseROIFilter {
// public:
//     MyROIFilter() : BaseROIFilter() {}
//     virtual ~MyROIFilter() {}
//
//     virtual bool init() override {
//         // Do something.
//         return true;
//     }
//
//     virtual bool filter(
//              const pcl_util::PointCloudPtr& cloud,
//              const ROIFilterOptions &roi_filter_options,
//              pcl_util::PointCloudPtr* roi_cloud) override {
//
//          // Do something.
//          return true;
//      }
//
//      virtual std::string name() const override {
//          return "MyROIFilter";
//      }
//
// };
//
// // Register plugin.
// REGISTER_GROUNDDETECTOR(MyROIFilter);
////////////////////////////////////////////////////////
// USING CODE:
//
// BaseROIFilter* roi_filter =
//     BaseROIFilterRegisterer::get_instance_by_name("MyROIFilter");
// using roi_filter to do somethings.
// ////////////////////////////////////////////////////

#include <memory>
#include <string>
#include <vector>

#include "Eigen/Core"

#include "modules/common/macro.h"
#include "modules/perception/common/pcl_types.h"
#include "modules/perception/lib/base/registerer.h"
#include "modules/perception/obstacle/base/hdmap_struct.h"
#include "modules/perception/obstacle/base/object.h"
#include "modules/perception/obstacle/base/types.h"

namespace apollo {
namespace perception {

struct ROIFilterOptions {
  ROIFilterOptions() {
    velodyne_trans = nullptr;
    hdmap = nullptr;
  }

  HdmapStructConstPtr hdmap;
  std::shared_ptr<const Eigen::Matrix4d> velodyne_trans;
};

class BaseROIFilter {
 public:
  BaseROIFilter() {}
  virtual ~BaseROIFilter() {}

  virtual bool Init() = 0;

  virtual bool Filter(const pcl_util::PointCloudPtr &cloud,
                      const ROIFilterOptions &roi_filter_options,
                      pcl_util::PointIndices *roi_indices) = 0;

  virtual std::string name() const = 0;

 private:
  DISALLOW_COPY_AND_ASSIGN(BaseROIFilter);
};

REGISTER_REGISTERER(BaseROIFilter);
#define REGISTER_ROIFILTER(name) REGISTER_CLASS(BaseROIFilter, name)

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_LIDAR_INTERFACE_BASE_ROI_FILTER_H_
