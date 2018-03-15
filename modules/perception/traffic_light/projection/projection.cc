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
#include "modules/perception/traffic_light/projection/projection.h"

#include <algorithm>
#include <vector>

#include "gflags/gflags.h"
#include "modules/common/log.h"
#include "modules/perception/traffic_light/base/utils.h"

DEFINE_double(light_height_adjust, 0, " adjust height without chaning code");

namespace apollo {
namespace perception {
namespace traffic_light {

bool BoundaryProjection::Project(const CameraCoeffient &camera_coeffient,
                                 const Eigen::Matrix4d &pose,
                                 const apollo::hdmap::Signal &tl_info,
                                 Light *light) const {
  int bound_size = tl_info.boundary().point_size();
  if (bound_size < 4) {
    AERROR << "Light boundary should be rectangle, which has four points! Got :"
           << bound_size;
    return false;
  }

  std::vector<int> x(bound_size);
  std::vector<int> y(bound_size);

  for (int i = 0; i < bound_size; ++i) {
    if (!ProjectPointDistort(camera_coeffient, pose,
                             tl_info.boundary().point(i), &x[i], &y[i])) {
      return false;
    }
  }
  int minx = std::min(x[0], x[2]);
  int miny = std::min(y[0], y[2]);
  int maxx = std::max(x[0], x[2]);
  int maxy = std::max(y[0], y[2]);

  cv::Rect roi(minx, miny, maxx - minx, maxy - miny);
  AINFO << "projection get ROI:" << roi;
  if (minx < 0 || miny < 0 ||
      maxx >= static_cast<int>(camera_coeffient.image_width) ||
      maxy >= static_cast<int>(camera_coeffient.image_height)) {
    AWARN << "Projection get ROI outside the image. ";
    return false;
  }
  light->region.projection_roi = RefinedBox(
      roi,
      cv::Size(camera_coeffient.image_width, camera_coeffient.image_height));
  AINFO << "refined ROI:" << light->region.projection_roi;

  return true;
}
bool BoundaryProjection::ProjectPoint(const CameraCoeffient &coeffient,
                                      const Eigen::Matrix4d &pose,
                                      const apollo::common::Point3D &point,
                                      int *center_x, int *center_y) const {
  Eigen::Matrix<double, 4, 1> TL_loc_LTM;
  Eigen::Matrix<double, 3, 1> TL_loc_cam;

  TL_loc_LTM << point.x(), point.y(), point.z() + FLAGS_light_height_adjust,
      1.0;
  TL_loc_LTM = coeffient.camera_extrinsic * pose.inverse() * TL_loc_LTM;

  // The light may behind the car, we can't project them on the images.
  if (TL_loc_LTM(2) < 0) {
    AWARN << "Compute a light behind the car. light to car Pose:\n"
          << TL_loc_LTM;
    return false;
  }
  TL_loc_cam = coeffient.camera_intrinsic * TL_loc_LTM;

  TL_loc_cam /= TL_loc_cam(2, 0);
  *center_x = static_cast<int>(TL_loc_cam(0, 0));
  *center_y = static_cast<int>(TL_loc_cam(1, 0));

  return true;
}

bool BoundaryProjection::ProjectPointDistort(const CameraCoeffient &coeffient,
                                             const Eigen::Matrix4d &pose,
                                             const common::PointENU &point,
                                             int *center_x,
                                             int *center_y) const {
  Eigen::Matrix<double, 4, 1> TL_loc_LTM;
  Eigen::Matrix<double, 3, 1> TL_loc_cam;

  TL_loc_LTM << point.x(), point.y(), point.z() + FLAGS_light_height_adjust,
      1.0;
  TL_loc_LTM = coeffient.camera_extrinsic * pose.inverse() * TL_loc_LTM;

  if (TL_loc_LTM(2) < 0) {
    AWARN << "Compute a light behind the car. light to car Pose:\n"
          << TL_loc_LTM;
    return false;
  }

  Eigen::Matrix<double, 2, 1> pt2d;
  pt2d[0] = TL_loc_LTM[0] / TL_loc_LTM[2];
  pt2d[1] = TL_loc_LTM[1] / TL_loc_LTM[2];

  pt2d = PixelDenormalize(pt2d, coeffient.camera_intrinsic,
                          coeffient.distort_params);
  *center_x = pt2d[0];
  *center_y = pt2d[1];
  return true;
}

Eigen::Matrix<double, 2, 1> BoundaryProjection::PixelDenormalize(
    const Eigen::Matrix<double, 2, 1> &pt2d,
    const Eigen::Matrix<double, 3, 4> &camera_intrinsic,
    const Eigen::Matrix<double, 5, 1> &distort_params) const {
  // add distortion
  double r_sq = pt2d[0] * pt2d[0] + pt2d[1] * pt2d[1];
  Eigen::Matrix<double, 2, 1> pt2d_radial =
      pt2d * (1 + distort_params[0] * r_sq + distort_params[1] * r_sq * r_sq +
              distort_params[4] * r_sq * r_sq * r_sq);
  Eigen::Matrix<double, 2, 1> dpt2d;
  dpt2d[0] = 2 * distort_params[2] * pt2d[0] * pt2d[1] +
             distort_params[3] * (r_sq + 2 * pt2d[0] * pt2d[0]);
  dpt2d[1] = distort_params[2] * (r_sq + 2 * pt2d[1] * pt2d[1]) +
             2 * distort_params[3] * pt2d[0] * pt2d[1];

  Eigen::Matrix<double, 2, 1> pt2d_distort;
  pt2d_distort[0] = pt2d_radial[0] + dpt2d[0];
  pt2d_distort[1] = pt2d_radial[1] + dpt2d[1];

  // add intrinsic K
  double focal_length_x = camera_intrinsic(0, 0);
  double focal_length_y = camera_intrinsic(1, 1);
  double center_x = camera_intrinsic(0, 2);
  double center_y = camera_intrinsic(1, 2);

  Eigen::Matrix<double, 2, 1> pt;
  pt[0] = pt2d_distort[0] * focal_length_x + center_x;
  pt[1] = pt2d_distort[1] * focal_length_y + center_y;

  return pt;
}

}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
