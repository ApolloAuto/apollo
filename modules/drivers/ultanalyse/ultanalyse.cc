/******************************************************************************
 * Copyright 2018 wh. All Rights Reserved.
 * ultrasound analyse module
 * 
 * wh  27 Nov. 2018
 * modify: wh 14 Dec. 2018
 * modify: wh 22 Dec.2018 (don not set Timer to publish data)
 * mofify: wh 12 Jau. 2019 
 *         1. solve problems form the bug（form tester Zhang)
 *         2. reduce the consumption of CPU( from 40% to --%)
 *         3. less/no information in log text 
 * add: wh 30 Jau. 2019
 *      add log info 
 * 
 *****************************************************************************/
#include "modules/drivers/ultanalyse/ultanalyse.h"

#include <cmath>
#include <vector>
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/drivers/ultanalyse/common/ultanalyse_gflags.h"
#include "ros/include/ros/ros.h"
#include "std_msgs/Int32MultiArray.h"

namespace apollo {
namespace ultanalyse {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::adapter::AdapterManager;
using apollo::ultanalyse::ObstAnalyse;
using apollo::ultanalyse::ChassisUlt;
using apollo::monitor::SystemStatus;

#define PI 3.1415926

std::string Ultanalyse::Name() const { return FLAGS_module_name; }

Status Ultanalyse::Init() {
  AdapterManager::Init(FLAGS_adapter_config_filename);

  CHECK(AdapterManager::GetUltrasound()) << "Ultrasound is not initialized.";
  CHECK(AdapterManager::GetSystemStatus()) <<
  "SystemStatus is not initialized.";

  AdapterManager::AddUltrasoundCallback(&Ultanalyse::OnUltrasound, this);

  AINFO << "finish initialization";
  return Status::OK();
}

Status Ultanalyse::Start() {
  return Status::OK();
}

void Ultanalyse::Stop() {
}

void Ultanalyse::OnUltrasound(const std_msgs::Int32MultiArray& message) {
  ChassisUlt_.clear_sonar_range();
  for (std::vector<int>::const_iterator it =
  message.data.begin(); it != message.data.end(); ++it) {
    ChassisUlt_.add_sonar_range(static_cast<double>(*it));
  }
  ObstAnalyse();
  AdapterManager::FillUltanalyseHeader(FLAGS_node_name, &obst_analyse_);
  AdapterManager::PublishUltanalyse(obst_analyse_);
}

void Ultanalyse::OnSystemStatus(const SystemStatus& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  system_status_.CopyFrom(message);
}

void Ultanalyse::ObstAnalyse() {
  // set original location of 12 ultrasonic sensors(b0-11,a0-11)
  double b[12] = {30.0, 5.5, -30.0, -53.0, -53.0, -53.0, -30.0, 0.0,
                  30.0, 53.0, 53.0, 53.0};
  double a[12] = {125.0, 125.0, 125.0, 85.0, 55.0, 25.0, -20.0, -20.0,
                  -20.0, 25.0, 55.0, 85.0, };
  double Beta[12]={40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0,
                   40.0, 40.0, 40.0, 40.0};  // set elevation angle as Beta
  double Sita[12] = {24.0, 0.0, 336.0, 294.0, 270.0, 246.0, 204.0,
                     180.0, 156.0, 114.0, 90.0, 66.0};  // set Sita
  double y[12] = {0.0};  // the location of obstacles(yn,xn)
  double x[12] = {0.0};
  bool obst_flag = 0;  // flag for obstacle
  int count = 0;  // number of the detected obstacles
  double obst_location_y = 0.0;
  double obst_location_x = 0.0;
  double obst_distance = 0.0;

  std::lock_guard<std::mutex> lock(mutex_);

  obst_analyse_.clear_obstinfo();
  for (int i = 0; i < 12; i++) {
    if ((ChassisUlt_.sonar_range(i) >= FLAGS_min_sonar_range_detectable) &&
        (ChassisUlt_.sonar_range(i) <= FLAGS_max_sonar_range_detectable)) {
      obst_flag = 1;
      obst_distance = ChassisUlt_.sonar_range(i) * cos(Beta[i]*PI/180);
      y[i] = b[i] +
             (ChassisUlt_.sonar_range(i) * cos(Beta[i]*PI/180)) *
             sin(Sita[i]*PI/180);
      x[i] = a[i] +
             (ChassisUlt_.sonar_range(i) * cos(Beta[i]*PI/180)) *
             cos(Sita[i]*PI/180);
      obst_location_y = y[i];
      obst_location_x = x[i];

      ADEBUG << "ObstInfo: y[" << i << "]=" << y[i] << ", x["
             << i << "]=" << x[i]
             << ", sensor No.:" << count << ", d=" << obst_distance;

      obst_analyse_.add_obstinfo();
      obst_analyse_.mutable_obstinfo(count)->set_location_y(obst_location_y);
      obst_analyse_.mutable_obstinfo(count)->set_location_x(obst_location_x);
      obst_analyse_.mutable_obstinfo(count)->set_detec_sonar(i+1);
      obst_analyse_.mutable_obstinfo(count)->set_obst_distance(obst_distance);
      count++;  // count the number of the detected obstacles

    } else if ((ChassisUlt_.sonar_range(i) < FLAGS_min_sonar_range_detectable)
               || (ChassisUlt_.sonar_range(i) == 65535)) {
      AERROR << "distance is invalid, sensor is FAULT";
    } else {
      y[i] = 0.0;
      x[i] = 0.0;
      obst_flag = obst_flag > 0 ? 1 : 0;
    }
  }
  obst_analyse_.set_obst_flag(obst_flag);
  obst_analyse_.set_obst_number(count);
}
  
}  // namespace ultanalyse
}  // namespace apollo
