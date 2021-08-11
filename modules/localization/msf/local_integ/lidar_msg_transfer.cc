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

#include "modules/localization/msf/local_integ/lidar_msg_transfer.h"

#include "Eigen/Core"
#include "Eigen/Geometry"

#include "cyber/common/log.h"
#include "cyber/time/time.h"
#include "modules/localization/common/localization_gflags.h"

namespace apollo {
namespace localization {
namespace msf {

void LidarMsgTransfer::Transfer(const drivers::PointCloud &msg,
                                LidarFrame *lidar_frame) {
  CHECK_NOTNULL(lidar_frame);

  if (msg.height() > 1 && msg.width() > 1) {
    for (unsigned int i = 0; i < msg.height(); ++i) {
      for (unsigned int j = 0; j < msg.width(); ++j) {
        Eigen::Vector3d pt3d;
        pt3d[0] = static_cast<double>(msg.point(i * msg.width() + j).x());
        pt3d[1] = static_cast<double>(msg.point(i * msg.width() + j).y());
        pt3d[2] = static_cast<double>(msg.point(i * msg.width() + j).z());
        if (!std::isnan(pt3d[0])) {
          Eigen::Vector3d pt3d_tem = pt3d;

          if (pt3d_tem[2] > max_height_) {
            continue;
          }
          unsigned char intensity = static_cast<unsigned char>(
              msg.point(i * msg.width() + j).intensity());
          lidar_frame->pt_xs.push_back(pt3d[0]);
          lidar_frame->pt_ys.push_back(pt3d[1]);
          lidar_frame->pt_zs.push_back(pt3d[2]);
          lidar_frame->intensities.push_back(intensity);
        }
      }
    }
  } else {
    AINFO << "Receiving un-organized-point-cloud, width " << msg.width()
          << " height " << msg.height() << "size " << msg.point_size();
    for (int i = 0; i < msg.point_size(); ++i) {
      Eigen::Vector3d pt3d;
      pt3d[0] = static_cast<double>(msg.point(i).x());
      pt3d[1] = static_cast<double>(msg.point(i).y());
      pt3d[2] = static_cast<double>(msg.point(i).z());
      if (!std::isnan(pt3d[0])) {
        Eigen::Vector3d pt3d_tem = pt3d;

        if (pt3d_tem[2] > max_height_) {
          continue;
        }
        unsigned char intensity =
            static_cast<unsigned char>(msg.point(i).intensity());
        lidar_frame->pt_xs.push_back(pt3d[0]);
        lidar_frame->pt_ys.push_back(pt3d[1]);
        lidar_frame->pt_zs.push_back(pt3d[2]);
        lidar_frame->intensities.push_back(intensity);
      }
    }
  }

  lidar_frame->measurement_time =
      cyber::Time(msg.measurement_time()).ToSecond();
  if (FLAGS_lidar_debug_log_flag) {
    AINFO << std::setprecision(15) << "LocalLidar Debug Log: velodyne msg. "
          << "[time:" << lidar_frame->measurement_time
          << "][height:" << msg.height() << "][width:" << msg.width()
          << "][point_cnt:" << msg.point_size() << "]";
  }
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
