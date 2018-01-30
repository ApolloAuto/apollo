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

#ifndef MODULES_PERCEPTION_OBSTACLE_LIDAR_VISUALIZER_ARC_BALL_H_
#define MODULES_PERCEPTION_OBSTACLE_LIDAR_VISUALIZER_ARC_BALL_H_

#include <Eigen/Dense>

namespace apollo {
namespace perception {

class ArcBall {
 public:
  ArcBall() = default;
  ~ArcBall() = default;

  template <typename T>
  static Eigen::Quaterniond RotateByMouse(T pre_x, T pre_y, T cur_x, T cur_y,
                                          T obj_cen_x, T obj_cen_y,
                                          T screen_width, T screen_height) {
    double px = static_cast<double>(pre_x - obj_cen_x) / screen_width;
    double py = static_cast<double>(obj_cen_y - pre_y) / screen_height;
    double dx = static_cast<double>(cur_x - obj_cen_x) / screen_width;
    double dy = static_cast<double>(obj_cen_y - cur_y) / screen_height;
    const Eigen::Vector3d p1(px, py, ProjectToBall(px, py));
    const Eigen::Vector3d p2(dx, dy, ProjectToBall(dx, dy));
    Eigen::Vector3d axis = p2.cross(p1);
    const double angle =
        2.0 *
        asin(sqrt(axis.squaredNorm() / p1.squaredNorm() / p2.squaredNorm()));
    axis.normalize();
    Eigen::AngleAxisd angleAxis(angle, axis);
    return Eigen::Quaterniond(angleAxis);
  }

  static double ProjectToBall(double x, double y) {
    const double rad = 1.0;
    const double rad2 = rad * rad;
    const double limit = rad2 * 0.5;

    const double d = x * x + y * y;
    return d < limit ? sqrt(rad2 - d) : limit / sqrt(d);
  }
};

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_LIDAR_VISUALIZER_ARC_BALL_H_
