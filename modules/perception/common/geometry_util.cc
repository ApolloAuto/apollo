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

#include "modules/perception/common/geometry_util.h"

#include "modules/common/math/math_utils.h"

namespace apollo {
namespace perception {

using pcl_util::Point;
using pcl_util::PointCloud;
using pcl_util::PointCloudPtr;

/*
 * Transform point cloud methods
 */
void TransformPointCloud(pcl_util::PointCloudPtr cloud,
                         const std::vector<int>& indices,
                         pcl_util::PointDCloud* trans_cloud) {
  if (trans_cloud->size() != indices.size()) {
    trans_cloud->resize(indices.size());
  }
  for (size_t i = 0; i < indices.size(); ++i) {
    const Point& p = cloud->at(indices[i]);
    Eigen::Vector3d v(p.x, p.y, p.z);
    pcl_util::PointD& tp = trans_cloud->at(i);
    tp.x = v.x();
    tp.y = v.y();
    tp.z = v.z();
    tp.intensity = p.intensity;
  }
}

void TransformPointCloud(pcl_util::PointCloudPtr cloud,
                         const Eigen::Matrix4d& pose_velodyne,
                         pcl_util::PointDCloudPtr trans_cloud) {
  Eigen::Matrix4d pose = pose_velodyne;

  if (trans_cloud->size() != cloud->size()) {
    trans_cloud->resize(cloud->size());
  }
  for (size_t i = 0; i < cloud->points.size(); ++i) {
    const Point& p = cloud->at(i);
    Eigen::Vector4d v(p.x, p.y, p.z, 1);
    v = pose * v;
    pcl_util::PointD& tp = trans_cloud->at(i);
    tp.x = v.x();
    tp.y = v.y();
    tp.z = v.z();
    tp.intensity = p.intensity;
  }
}

/*
 * Vector & Matrix related methods
 */
void TransAffineToMatrix4(const Eigen::Vector3d& translation,
                          const Eigen::Vector4d& rotation,
                          Eigen::Matrix4d* trans_matrix) {
  const double t_x = translation(0);
  const double t_y = translation(1);
  const double t_z = translation(2);

  const double qx = rotation(0);
  const double qy = rotation(1);
  const double qz = rotation(2);
  const double qw = rotation(3);

  (*trans_matrix) << 1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw),
      2 * (qx * qz + qy * qw), t_x, 2 * (qx * qy + qz * qw),
      1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw), t_y,
      2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw),
      1 - 2 * (qx * qx + qy * qy), t_z, 0, 0, 0, 1;
}

void ComputeMostConsistentBboxDirection(const Eigen::Vector3f& previous_dir,
                                        Eigen::Vector3f* current_dir) {
  float dot_val_00 =
      previous_dir(0) * (*current_dir)(0) + previous_dir(1) * (*current_dir)(1);
  float dot_val_01 =
      previous_dir(0) * (*current_dir)(1) - previous_dir(1) * (*current_dir)(0);
  if (fabs(dot_val_00) >= fabs(dot_val_01)) {
    if (dot_val_00 < 0) {
      (*current_dir) = -(*current_dir);
    }
  } else {
    if (dot_val_01 < 0) {
      (*current_dir) =
          Eigen::Vector3f((*current_dir)(1), -(*current_dir)(0), 0);
    } else {
      (*current_dir) =
          Eigen::Vector3f(-(*current_dir)(1), (*current_dir)(0), 0);
    }
  }
}

double VectorCosTheta2dXy(const Eigen::Vector3f& v1,
                          const Eigen::Vector3f& v2) {
  double v1_len = sqrt((v1.head(2).cwiseProduct(v1.head(2))).sum());
  double v2_len = sqrt((v2.head(2).cwiseProduct(v2.head(2))).sum());
  double cos_theta =
      (v1.head(2).cwiseProduct(v2.head(2))).sum() / (v1_len * v2_len);
  return cos_theta;
}

double VectorTheta2dXy(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2) {
  double v1_len = sqrt((v1.head(2).cwiseProduct(v1.head(2))).sum());
  double v2_len = sqrt((v2.head(2).cwiseProduct(v2.head(2))).sum());
  double cos_theta =
      (v1.head(2).cwiseProduct(v2.head(2))).sum() / (v1_len * v2_len);
  double sin_theta = (v1(0) * v2(1) - v1(1) * v2(0)) / (v1_len * v2_len);

  cos_theta = common::math::Clamp(cos_theta, 1.0, -1.0);

  double theta = acos(cos_theta);
  if (sin_theta < 0) {
    theta = -theta;
  }
  return theta;
}

}  // namespace perception
}  // namespace apollo
