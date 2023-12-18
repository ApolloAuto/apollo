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
#include "modules/perception/common/base/point_cloud_util.h"

#include <algorithm>
#include <limits>
#include "Eigen/Dense"
#include "opencv2/opencv.hpp"

#include "cyber/common/log.h"

namespace apollo {
namespace perception {
namespace base {

bool DownSamplePointCloudBeams(base::PointFCloudPtr cloud_ptr,
                               base::PointFCloudPtr out_cloud_ptr,
                               int downsample_factor) {
  if (downsample_factor <= 0) {
    return false;
  }
  for (size_t i = 0; i < cloud_ptr->size(); ++i) {
    int32_t beam_id = cloud_ptr->points_beam_id(i);
    if (beam_id % downsample_factor == 0) {
      base::PointF point = cloud_ptr->at(i);
      double timestamp = cloud_ptr->points_timestamp(i);
      float height = cloud_ptr->points_height(i);
      uint8_t label = cloud_ptr->points_label(i);
      out_cloud_ptr->push_back(point, timestamp, height, beam_id, label);
    }
  }
  return true;
}

void GetPointCloudCentroid(const PointFCloud& cloud, PointF* centroid) {
  for (size_t i = 0; i < cloud.size(); ++i) {
    centroid->x += cloud[i].x;
    centroid->y += cloud[i].y;
    centroid->z += cloud[i].z;
  }
  centroid->x /= static_cast<float>(cloud.size());
  centroid->y /= static_cast<float>(cloud.size());
  centroid->z /= static_cast<float>(cloud.size());
}

void CloudDemean(PointFCloud* pc) {
  // Demean by using centroid.
  PointF centroid;
  GetPointCloudCentroid(*pc, &centroid);
  for (size_t i = 0; i < pc->size(); ++i) {
    (*pc)[i].x -= centroid.x;
    (*pc)[i].y -= centroid.y;
    (*pc)[i].z -= centroid.z;
  }
}

double OrientCloud(const PointFCloud& pc, PointFCloud* pc_out, bool demean) {
  // Approach#1:
  // Find car dominant direction on XY plane.
  /*Eigen::VectorXf coeff;
  find_dominant_direction_xy(pc, coeff);
  // This theta has ambiguity. To calculate the true heading,
  // we need to consider both obstacle's velocity direction and
  // robot's current velocity direction.
  double theta = atan2(coeff[4], coeff[3]);*/
  // Approach#2:
  // Use Minimum Area Bounding Box
  BoundingCube bbox;
  GetPointCloudMinareaBbox(pc, &bbox);
  float theta = static_cast<float>(bbox.yaw);
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.rotate(Eigen::AngleAxisf(-theta, Eigen::Vector3f(0, 0, 1)));
  pc.TransformPointCloud(transform, pc_out, true);
  if (demean) {
    CloudDemean(pc_out);
  }
  return theta;
}

bool GetPointCloudMinareaBbox(const PointFCloud& pc, BoundingCube* box,
                              const int& min_num_points, const bool& verbose) {
  if (pc.size() <= static_cast<size_t>(min_num_points)) {
    return false;
  }
  std::vector<cv::Point2f> pts;
  float min_z = std::numeric_limits<float>::max();
  float max_z = -std::numeric_limits<float>::max();
  for (size_t i = 0; i < pc.size(); ++i) {
    pts.push_back(cv::Point2f(pc[i].x, pc[i].y));
    min_z = std::min(min_z, pc[i].z);
    max_z = std::max(max_z, pc[i].z);
  }
  // compute MinAreaRect
  cv::RotatedRect mar = cv::minAreaRect(pts);
  // adjust angle
  if (mar.size.width < mar.size.height) {
    mar.angle -= 90;
    float tmp = mar.size.width;
    mar.size.width = mar.size.height;
    mar.size.height = tmp;
  }
  if (verbose) {
    AINFO << "center = " << mar.center.x << " " << mar.center.y << std::endl;
    AINFO << "size = " << mar.size.height << " " << mar.size.width << std::endl;
    AINFO << "yaw = " << mar.angle << std::endl;
    AINFO << "height = " << max_z - min_z << std::endl;
  }
  box->x = mar.center.x;
  box->y = mar.center.y;
  box->z = static_cast<float>((min_z + max_z) / 2.0);
  box->length = mar.size.width;
  box->width = mar.size.height;
  box->height = max_z - min_z;
  box->yaw = static_cast<float>((M_PI * (mar.angle + 180)) / 180);
  return true;
}

}  // namespace base
}  // namespace perception
}  // namespace apollo
