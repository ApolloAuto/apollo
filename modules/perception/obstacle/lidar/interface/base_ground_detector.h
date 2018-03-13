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
#ifndef APOLLO_PERCEPTION_OBSTACLE_LIDAR_INTERFACE_BASE_GROUND_DETECTOR_H_
#define APOLLO_PERCEPTION_OBSTACLE_LIDAR_INTERFACE_BASE_GROUND_DETECTOR_H_

// SAMPLE CODE:
//
// class MyGroundDetector : public BaseGroundDetector {
// public:
//     MyGroundDetector() : BaseGroundDetector() {}
//     virtual ~MyGroundDetector() {}
//
//     virtual bool init() override {
//         // Do something.
//         return true;
//     }
//
//     virtual bool detect(
//              pcl_util::PointCloudPtr& cloud,
//              const GroundDetectorOptions& options,
//              pcl_util::PointIndices* non_ground_indices) override {
//
//          // Do something.
//          return true;
//      }
//
//      virtual std::string name() const override {
//          return "MyGroundDetector";
//      }
//
// };
//
// // Register plugin.
// REGISTER_GROUNDDETECTOR(MyGroundDetector);
////////////////////////////////////////////////////////
// USING CODE:
// // BaseGroundDetector* ground_detector = //
// BaseGroundDetectorRegisterer::get_instance_by_name("MyGroundDetector");
// using ground_detector to do somethings.
// ////////////////////////////////////////////////////

#include <string>

#include "modules/common/macro.h"
#include "modules/perception/common/pcl_types.h"
#include "modules/perception/lib/base/registerer.h"

namespace apollo {
namespace perception {

struct GroundDetectorOptions {
  GroundDetectorOptions() {
    velodyne_position.x = 0.0;
    velodyne_position.y = 0.0;
    velodyne_position.z = 0.0;
    velodyne_ground_position.x = 0.0;
    velodyne_ground_position.y = 0.0;
    velodyne_ground_position.z = 0.0;
  }

  pcl_util::Point velodyne_position;
  pcl_util::Point velodyne_ground_position;
  Eigen::Matrix4d trans_velodyne_to_world;
};

class BaseGroundDetector {
 public:
  BaseGroundDetector() {}
  virtual ~BaseGroundDetector() {}

  virtual bool Init() = 0;

  // @brief: detect ground points from point cloud.
  //         and will update height field.
  // @param [in/out]: input point cloud.
  // @param [in]: options
  // @param [out]: non ground points index.
  virtual bool Detect(const GroundDetectorOptions &options,
                      pcl_util::PointCloudPtr cloud,
                      pcl_util::PointIndicesPtr non_ground_indices) = 0;

  virtual std::string name() const = 0;

 private:
  DISALLOW_COPY_AND_ASSIGN(BaseGroundDetector);
};

REGISTER_REGISTERER(BaseGroundDetector);
#define REGISTER_GROUNDDETECTOR(name) REGISTER_CLASS(BaseGroundDetector, name)

}  // namespace perception
}  // namespace apollo

#endif  // APOLLO_PERCEPTION_OBSTACLE_LIDAR_INTERFACE_BASE_GROUND_DETECTOR_H_
