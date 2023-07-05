/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/localization/msf/local_tool/data_extraction/pcd_exporter.h"

#include "cyber/cyber.h"
#include "modules/localization/msf/common/io/pcl_point_types.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"

namespace apollo {
namespace localization {
namespace msf {

PCDExporter::PCDExporter(const std::string &pcd_folder) {
  pcd_folder_ = pcd_folder;
  std::string stamp_file = pcd_folder_ + "/pcd_timestamp.txt";

  if ((stamp_file_handle_ = fopen(stamp_file.c_str(), "a")) == nullptr) {
    AERROR << "Cannot open stamp file!";
  }
}

PCDExporter::~PCDExporter() {
  if (stamp_file_handle_ != nullptr) {
    fclose(stamp_file_handle_);
  }
}

void PCDExporter::CompensatedPcdCallback(const std::string &msg_string) {
  AINFO << "Compensated pcd callback.";
  drivers::PointCloud msg;
  msg.ParseFromString(msg_string);

  static unsigned int index = 1;

  std::stringstream ss_pcd;
  ss_pcd << pcd_folder_ << "/" << index << ".pcd";
  std::string pcd_filename = ss_pcd.str();

  WritePcdFile(pcd_filename, msg);
  double timestamp = cyber::Time(msg.measurement_time()).ToSecond();
  fprintf(stamp_file_handle_, "%u %lf\n", index, timestamp);

  ++index;
}

void PCDExporter::WritePcdFile(const std::string &filename,
                               const drivers::PointCloud &msg) {
  pcl::PointCloud<velodyne::PointXYZIT> cloud;
  cloud.width = msg.width();
  cloud.height = msg.height();
  cloud.is_dense = false;
  cloud.points.resize(cloud.width * cloud.height);

  if (cloud.width == 0 || cloud.height == 0) {
    cloud.width = 1;
    cloud.height = msg.point_size();
    cloud.points.resize(msg.point_size());
  }

  for (unsigned int i = 0; i < static_cast<unsigned int>(cloud.points.size());
       ++i) {
    cloud.points[i].x = msg.point(i).x();
    cloud.points[i].y = msg.point(i).y();
    cloud.points[i].z = msg.point(i).z();
    cloud.points[i].intensity =
        static_cast<unsigned char>(msg.point(i).intensity());
  }

  pcl::io::savePCDFileBinaryCompressed(filename, cloud);
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
