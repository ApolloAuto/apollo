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

#ifndef MODULES_PERCEPTION_OBSTACLE_RADAR_INTERFACE_RADAR_DETECTOR_H_
#define MODULES_PERCEPTION_OBSTACLE_RADAR_INTERFACE_RADAR_DETECTOR_H_

// SAMPLE CODE:
//
// class DefaultRadarDetector : public BaseRadarDetector {
// public:
//     DefaultRadarDetector() : BaseRadarDetector() {}
//     virtual ~DefaultRadarDetector() {}
//
//     virtual bool init() override {
//         // Do something.
//         return true;
//     }
//
//     virtual bool detect(
//              const RadarRawObstacles& raw_obstacles,
//              const std::vector<PolygonType>& map_polygons,
//              const RadarDetectorOptions& options,
//              std::vector<ObjectPtr>* objects) override {
//          // Do something.
//          return true;
//      }
//
//      virtual std::string name() const override {
//          return "DefaultRadarDetector";
//      }
//
// };
//
// // Register plugin.
// REGISTER_RADAR_DETECTOR(DefaultRadarDetector);
////////////////////////////////////////////////////////
// USING CODE:
//
// BaseRadarDetector* radar_detector =
//    BaseRadarDetectorRegisterer::get_instance_by_name("DefaultRadarDetector");
// using radar_detector to do somethings.
// ////////////////////////////////////////////////////

#include <string>
#include <vector>
#include <Eigen/Core>

// defined in apollo/common
#include "modules/drivers/proto/sensor_radar.pb.h"
#include "modules/common/macro.h"
#include "modules/perception/lib/base/registerer.h"
#include "modules/perception/lib/pcl_util/pcl_types.h"
#include "modules/perception/obstacle/base/types.h"
#include "modules/perception/obstacle/base/object.h"

namespace apollo {
namespace perception {

using ::apollo::common::Header;
using ::apollo::drivers::RadarType;
using ::apollo::drivers::ContiRadarObs;
using ::apollo::drivers::RadarObsArray;

struct RadarDetectorOptions {
  Eigen::Matrix4d *radar2world_pose = nullptr;
  Eigen::Vector3f car_linear_speed = Eigen::Vector3f::Zero();
};
enum ContiObjectType {
  CONTI_POINT = 0,
  CONTI_CAR = 1,
  CONTI_TRUCK = 2,
  CONTI_PEDESTRIAN = 3,
  CONTI_MOTOCYCLE = 4,
  CONTI_BICYCLE = 5,
  CONTI_WIDE = 6,
  CONTI_UNKNOWN = 7,
  CONTI_MAX_OBJECT_TYPE = 8,
};
enum ContiMeasState {
  CONTI_DELETED = 0,
  CONTI_NEW = 1,
  CONTI_MEASURED = 2,
  CONTI_PREDICTED = 3,
  CONTI_DELETED_FOR = 4,
  CONTI_NEW_FROM_MERGE = 5,
};
struct ContiParams {
  double probexist_vehicle;
  double probexist_pedestrian;
  double probexist_bicycle;
  double probexist_unknown;
  double lo_vel_rms_vehicle;
  double la_vel_rms_vehicle;
  double lo_dist_rms_vehicle;
  double la_dist_rms_vehicle;
  double lo_vel_rms_pedestrian;
  double la_vel_rms_pedestrian;
  double lo_dist_rms_pedestrian;
  double la_dist_rms_pedestrian;
  double lo_vel_rms_bicycle;
  double la_vel_rms_bicycle;
  double lo_dist_rms_bicycle;
  double la_dist_rms_bicycle;
  double lo_vel_rms_unknown;
  double la_vel_rms_unknown;
  double lo_dist_rms_unknown;
  double la_dist_rms_unknown;
};
class BaseRadarDetector {
 public:
  BaseRadarDetector() = default;
  virtual ~BaseRadarDetector() = default;
  virtual bool Init() = 0;
  // @brief: Radar raw obstacles -> objects.
  // @param [in]: raw obstacles from radar driver.
  // @param [in]: roi map polygons, using world frame.
  // @param [in]: options.
  // @param [out]: transformed objects.
  virtual bool Detect(
      const RadarObsArray &raw_obstacles,
      const std::vector<PolygonDType> &map_polygons,
      const RadarDetectorOptions &options,
      std::vector<ObjectPtr> *objects) = 0;
  virtual std::string name() const = 0;
 private:
 DISALLOW_COPY_AND_ASSIGN(BaseRadarDetector);
};

REGISTER_REGISTERER(BaseRadarDetector);
#define REGISTER_RADARDETECTOR(name) REGISTER_CLASS(BaseRadarDetector, name)

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_RADAR_INTERFACE_RADAR_DETECTOR_H_
