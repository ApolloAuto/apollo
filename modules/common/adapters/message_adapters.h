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

#ifndef MODULES_ADAPTERS_MESSAGE_ADAPTERS_H_
#define MODULES_ADAPTERS_MESSAGE_ADAPTERS_H_

#include "modules/calibration/republish_msg/proto/relative_odometry.pb.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/canbus/proto/chassis_detail.pb.h"
#include "modules/common/adapters/adapter.h"
#include "modules/common/monitor/proto/monitor.pb.h"
#include "modules/common/proto/gnss_status.pb.h"
#include "modules/control/proto/control_cmd.pb.h"
#include "modules/control/proto/pad_msg.pb.h"
#include "modules/drivers/gnss/proto/ins.pb.h"
#include "modules/drivers/proto/delphi_esr.pb.h"
#include "modules/drivers/proto/conti_radar.pb.h"
#include "modules/drivers/proto/mobileye.pb.h"
#include "modules/localization/proto/gps.pb.h"
#include "modules/localization/proto/imu.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/monitor/proto/system_status.pb.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/perception/proto/traffic_light_detection.pb.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"
#include "modules/routing/proto/routing.pb.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"

/**
 * @file message_adapters.h
 * @namespace apollo::common::adapter
 * @brief This is an agglomeration of all the message adapters supported that
 * specializes the adapter template.
 */
namespace apollo {
namespace common {
namespace adapter {

using ChassisAdapter = Adapter<::apollo::canbus::Chassis>;
using ChassisDetailAdapter = Adapter<::apollo::canbus::ChassisDetail>;
using ControlCommandAdapter = Adapter<control::ControlCommand>;
using GpsAdapter = Adapter<apollo::localization::Gps>;
using ImuAdapter = Adapter<localization::Imu>;
using LocalizationAdapter = Adapter<apollo::localization::LocalizationEstimate>;
using MonitorAdapter = Adapter<apollo::common::monitor::MonitorMessage>;
using PadAdapter = Adapter<control::PadMessage>;
using PerceptionObstaclesAdapter = Adapter<perception::PerceptionObstacles>;
using PlanningAdapter = Adapter<planning::ADCTrajectory>;
using PointCloudAdapter = Adapter<::sensor_msgs::PointCloud2>;
using ImageShortAdapter = Adapter<::sensor_msgs::Image>;
using ImageLongAdapter = Adapter<::sensor_msgs::Image>;
using PredictionAdapter = Adapter<prediction::PredictionObstacles>;
using TrafficLightDetectionAdapter = Adapter<perception::TrafficLightDetection>;
using RoutingRequestAdapter = Adapter<routing::RoutingRequest>;
using RoutingResponseAdapter = Adapter<routing::RoutingResponse>;
using RelativeOdometryAdapter =
    Adapter<calibration::republish_msg::RelativeOdometry>;
using InsStatAdapter = Adapter<drivers::gnss::InsStat>;
using InsStatusAdapter = Adapter<gnss_status::InsStatus>;
using GnssStatusAdapter = Adapter<gnss_status::GnssStatus>;
using SystemStatusAdapter = Adapter<apollo::monitor::SystemStatus>;
using MobileyeAdapter = Adapter<drivers::Mobileye>;
using DelphiESRAdapter = Adapter<drivers::DelphiESR>;
using ContiRadarAdapter = Adapter<drivers::ContiRadar>;
using CompressedImageAdapter = Adapter<sensor_msgs::CompressedImage>;

}  // namespace adapter
}  // namespace common
}  // namespace apollo

#endif  // MODULES_ADAPTERS_MESSAGE_ADAPTERS_H_
