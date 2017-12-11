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

#include <pcl/common/time.h>
#include <pcl_conversions/pcl_conversions.h>
#include <string>
#include "modules/localization/msf/local_tool/data_extraction/pcd_exporter.h"

namespace apollo {
namespace localization {
namespace msf {

PCDExporter::PCDExporter(const std::string &pcd_folder) {
  pcd_folder_ = pcd_folder;
  std::string stamp_file = pcd_folder_ + "/pcd_timestamp.txt";

  if ((stamp_file_handle_ = fopen(stamp_file.c_str(), "a")) == NULL) {
    std::cerr << "Cannot open stamp file!" << std::endl;
  }
}

PCDExporter::~PCDExporter() {
  if (stamp_file_handle_ != NULL) {
    fclose(stamp_file_handle_);
  }
}

void PCDExporter::CompensatedPcdCallback(
    const rosbag::MessageInstance &msg_instance) {
  std::cout << "Compensated pcd callback." << std::endl;
  sensor_msgs::PointCloud2::ConstPtr msg =
      msg_instance.instantiate<sensor_msgs::PointCloud2>();

  static unsigned int index = 1;

  std::stringstream ss_pcd;
  ss_pcd << pcd_folder_ << "/" << index << ".pcd";
  std::string pcd_filename = ss_pcd.str();

  WritePcdFile(pcd_filename, msg);
  fprintf(stamp_file_handle_, "%u %lf\n", index, msg->header.stamp.toSec());

  ++index;
}

void PCDExporter::WritePcdFile(const std::string &filename,
                               const sensor_msgs::PointCloud2::ConstPtr &msg) {
  pcl::PCLPointCloud2 pcl_cloud;
  pcl_conversions::toPCL(*msg, pcl_cloud);
  pcl::PCDWriter writer;
  writer.writeBinaryCompressed(filename, pcl_cloud);
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
