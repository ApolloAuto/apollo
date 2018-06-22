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

#ifndef MODULES_PERCEPTION_OBSTACLE_VISUALIZER_FRAME_H_
#define MODULES_PERCEPTION_OBSTACLE_VISUALIZER_FRAME_H_

#include <Eigen/Geometry>

namespace apollo {
namespace perception {
namespace lowcostvisualizer {

class Frame {
 public:
  Frame();

  virtual ~Frame() {}

  Frame(const Frame &frame);

  Frame &operator=(const Frame &frame);

  Frame(const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation);

  void set_position(const Eigen::Vector3d &position);

  void set_position(double x, double y, double z);

  void set_orientation(const Eigen::Quaterniond &orientation);

  void set_orientation(double q0, double q1, double q2, double q3);

  void set_position_and_orientation(const Eigen::Vector3d &position,
                                    const Eigen::Quaterniond &orientation);

  Eigen::Vector3d position() const {
    return inverse_coordinates_of(Eigen::Vector3d(0.0, 0.0, 0.0));
  }

  Eigen::Quaterniond orientation() const;

  void get_position(double *x, double *y, double *z) const;

  void get_orientation(double *q0, double *q1, double *q2, double *q3) const;

  void set_translation(const Eigen::Vector3d &translation) {
    t_ = translation;
  }

  void set_translation(double x, double y, double z);

  void set_rotation(const Eigen::Quaterniond &rotation) {
    q_ = rotation;
  }

  void set_rotation(double q0, double q1, double q2, double q3);

  void set_translation_and_rotation(const Eigen::Vector3d &translation,
                                    const Eigen::Quaterniond &rotation);

  Eigen::Vector3d translation() const {
    return t_;
  }

  Eigen::Quaterniond rotation() const {
    return q_;
  }

  void get_translation(double *x, double *y, double *z) const;

  void get_rotation(double *q0, double *q1, double *q2, double *q3) const;

  const Frame *reference_frame() const {
    return reference_frame_;
  }

  void set_reference_frame(const Frame *const refFrame);

  bool setting_asreference_frame_will_create_a_loop(const Frame *const frame);

  void translate(const Eigen::Vector3d &t);

  void translate_t(const Eigen::Vector3d &t);

  void translate(double x, double y, double z);

  void translate(double *x, double *y, double *z);

  void rotate(const Eigen::Quaterniond &q);

  void rotate_q(const Eigen::Quaterniond &q);

  void rotate(double q0, double q1, double q2, double q3);

  void rotate(double *q0, double *q1, double *q2, double *q3);

  void rotate_around_point(const Eigen::Quaterniond &rotation,
                           const Eigen::Vector3d &point);

  void align_with_frame(const Frame *const frame, bool move = false,
                        double threshold = 0.85f);

  void project_on_line(const Eigen::Vector3d &origin,
                       const Eigen::Vector3d &direction);

  Eigen::Vector3d coordinates_of(const Eigen::Vector3d &src) const;

  Eigen::Vector3d inverse_coordinates_of(const Eigen::Vector3d &src) const;

  Eigen::Vector3d local_coordinates_of(const Eigen::Vector3d &src) const;

  Eigen::Vector3d local_inverse_coordinates_of(
      const Eigen::Vector3d &src) const;

  Eigen::Vector3d coordinates_of_in(const Eigen::Vector3d &src,
                                    const Frame *const in) const;

  Eigen::Vector3d coordinates_of_from(const Eigen::Vector3d &src,
                                      const Frame *const from) const;

  void get_coordinates_of(const double src[3], double res[3]) const;

  void get_inverse_coordinates_of(const double src[3], double res[3]) const;

  void get_local_coordinates_of(const double src[3], double res[3]) const;

  void get_local_inverse_coordinates_of(const double src[3],
                                        double res[3]) const;

  void get_coordinates_of_in(const double src[3], double res[3],
                             const Frame *const in) const;

  void get_coordinates_of_from(const double src[3], double res[3],
                               const Frame *const from) const;

  Eigen::Vector3d transform_of(
      const Eigen::Vector3d &src) const;  // world coordinate
  Eigen::Vector3d inverse_transform_of(const Eigen::Vector3d &src) const;

  Eigen::Vector3d local_transform_of(const Eigen::Vector3d &src) const;

  Eigen::Vector3d local_inverse_transform_of(const Eigen::Vector3d &src) const;

  Eigen::Vector3d transform_of_in(const Eigen::Vector3d &src,
                                  const Frame *const in) const;

  Eigen::Vector3d transform_of_from(const Eigen::Vector3d &src,
                                    const Frame *const from) const;

  void get_transform_of(const double src[3], double res[3]) const;

  void get_inverse_transform_of(const double src[3], double res[3]) const;

  void get_local_transform_of(const double src[3], double res[3]) const;

  void get_local_inverse_transform_of(const double src[3], double res[3]) const;

  void get_transform_of_in(const double src[3], double res[3],
                           const Frame *const in) const;

  void get_transform_of_from(const double src[3], double res[3],
                             const Frame *const from) const;

  const double *matrix() const;

  void get_matrix(double m[4][4]) const;

  void get_matrix(double m[16]) const;

  const double *world_matrix() const;

  void get_world_matrix(double m[4][4]) const;

  void get_world_matrix(double m[16]) const;

  void set_from_matrix(const double m[4][4]);

  void set_from_matrix(const double m[16]);

  Frame inverse() const;

  Frame world_inverse() const {
    return Frame(-(orientation().inverse()._transformVector(position())),
                 orientation().inverse());
  }

 private:
  // Position and orientation
  Eigen::Vector3d t_;
  Eigen::Quaterniond q_;

  // Frame composition
  const Frame *reference_frame_;
};

Eigen::Vector3d get_quaternion_axis(const Eigen::Quaterniond &quat);

double get_quaternion_angle(const Eigen::Quaterniond &quat);

}  // namespace lowcostvisualizer
}  // namespace perception
}  // namespace apollo

#endif  // XFRAME_H
