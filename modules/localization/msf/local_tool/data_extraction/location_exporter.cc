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

#include "modules/localization/msf/local_tool/data_extraction/location_exporter.h"
#include <string>
#include "modules/localization/proto/localization.pb.h"
#include "modules/localization/proto/measure.pb.h"

namespace apollo {
namespace localization {
namespace msf {

LocationExporter::LocationExporter(const std::string &loc_file_folder) {
  gnss_loc_file_ = loc_file_folder + "/gnss_loc.txt";
  lidar_loc_file_ = loc_file_folder + "/lidar_loc.txt";
  fusion_loc_file_ = loc_file_folder + "/fusion_loc.txt";

  if ((gnss_loc_file_handle_ = fopen(gnss_loc_file_.c_str(), "a")) == NULL) {
    std::cerr << "Cannot open gnss localization file!" << std::endl;
  }

  if ((lidar_loc_file_handle_ = fopen(lidar_loc_file_.c_str(), "a")) == NULL) {
    std::cerr << "Cannot open lidar localization file!" << std::endl;
  }

  if ((fusion_loc_file_handle_ = fopen(fusion_loc_file_.c_str(), "a")) ==
      NULL) {
    std::cerr << "Cannot open fusion localization file!" << std::endl;
  }
}

LocationExporter::~LocationExporter() {
  if (gnss_loc_file_handle_ != NULL) {
    fclose(gnss_loc_file_handle_);
  }

  if (lidar_loc_file_handle_ != NULL) {
    fclose(lidar_loc_file_handle_);
  }

  if (fusion_loc_file_handle_ != NULL) {
    fclose(fusion_loc_file_handle_);
  }
}

void LocationExporter::GnssLocCallback(
    const rosbag::MessageInstance &msg_instance) {
  std::cout << "GNSS location callback." << std::endl;
  boost::shared_ptr<LocalizationEstimate> msg =
      msg_instance.instantiate<LocalizationEstimate>();
  static unsigned int index = 1;

  double timestamp = msg->measurement_time();
  double x = msg->pose().position().x();
  double y = msg->pose().position().y();
  double z = msg->pose().position().z();

  double qx = msg->pose().orientation().qx();
  double qy = msg->pose().orientation().qy();
  double qz = msg->pose().orientation().qz();
  double qw = msg->pose().orientation().qw();

  double std_x = msg->uncertainty().position_std_dev().x();
  double std_y = msg->uncertainty().position_std_dev().y();
  double std_z = msg->uncertainty().position_std_dev().z();

  fprintf(gnss_loc_file_handle_,
          "%d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n", index, timestamp,
          x, y, z, qx, qy, qz, qw, std_x, std_y, std_z);

  ++index;
}

void LocationExporter::LidarLocCallback(
    const rosbag::MessageInstance &msg_instance) {
  std::cout << "Lidar location callback." << std::endl;
  boost::shared_ptr<LocalizationEstimate> msg =
      msg_instance.instantiate<LocalizationEstimate>();
  static unsigned int index = 1;

  double timestamp = msg->measurement_time();
  double x = msg->pose().position().x();
  double y = msg->pose().position().y();
  double z = msg->pose().position().z();

  double qx = msg->pose().orientation().qx();
  double qy = msg->pose().orientation().qy();
  double qz = msg->pose().orientation().qz();
  double qw = msg->pose().orientation().qw();

  double std_x = msg->uncertainty().position_std_dev().x();
  double std_y = msg->uncertainty().position_std_dev().y();
  double std_z = msg->uncertainty().position_std_dev().z();

  fprintf(lidar_loc_file_handle_,
          "%d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n", index, timestamp,
          x, y, z, qx, qy, qz, qw, std_x, std_y, std_z);

  ++index;
}

void LocationExporter::FusionLocCallback(
    const rosbag::MessageInstance &msg_instance) {
  std::cout << "Fusion location callback." << std::endl;
  boost::shared_ptr<LocalizationEstimate> msg =
      msg_instance.instantiate<LocalizationEstimate>();
  static unsigned int index = 1;

  double timestamp = msg->measurement_time();
  double x = msg->pose().position().x();
  double y = msg->pose().position().y();
  double z = msg->pose().position().z();

  double qx = msg->pose().orientation().qx();
  double qy = msg->pose().orientation().qy();
  double qz = msg->pose().orientation().qz();
  double qw = msg->pose().orientation().qw();

  double std_x = msg->uncertainty().position_std_dev().x();
  double std_y = msg->uncertainty().position_std_dev().y();
  double std_z = msg->uncertainty().position_std_dev().z();

  fprintf(fusion_loc_file_handle_,
          "%d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n", index, timestamp,
          x, y, z, qx, qy, qz, qw, std_x, std_y, std_z);

  ++index;
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
