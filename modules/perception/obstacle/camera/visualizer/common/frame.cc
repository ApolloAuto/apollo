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

#include "modules/perception/obstacle/camera/visualizer/common/frame.h"

#include <cmath>
#include <iostream>

namespace apollo {
namespace perception {
namespace lowcostvisualizer {

const double frame_pi = 3.141592653;

Eigen::Vector3d get_quaternion_axis(const Eigen::Quaterniond &quat) {
  Eigen::Vector3d res(quat.x(), quat.y(), quat.z());
  const double sinus = res.norm();
  if (sinus > 1E-8) {
    res /= sinus;
  }
  Eigen::Vector3d invRes = -res;  // just for avoiding the gcc bug on "? :"
  return (acos(quat.w()) <= frame_pi / 2.0) ? res : invRes;
}

double get_quaternion_angle(const Eigen::Quaterniond &quat) {
  const double angle = 2.0 * acos(quat.w());
  return (angle <= frame_pi) ? angle : 2.0 * frame_pi - angle;
}

Frame::Frame()
    : t_(0, 0, 0), q_(Eigen::Quaterniond(1, 0, 0, 0)), reference_frame_(NULL) {}

Frame::Frame(const Eigen::Vector3d &position,
             const Eigen::Quaterniond &orientation)
    : t_(position), q_(orientation), reference_frame_(NULL) {}

Frame &Frame::operator=(const Frame &frame) {
  // Automatic compiler generated version would not emit the modified() signals
  // as is done in
  // set_translation_and_rotation.
  set_translation_and_rotation(frame.translation(), frame.rotation());
  set_reference_frame(frame.reference_frame());
  return *this;
}

Frame::Frame(const Frame &frame) { (*this) = frame; }

const double *Frame::matrix() const {
  static double m[4][4];
  get_matrix(m);
  return (const double *)(m);
}

void Frame::get_matrix(double m[4][4]) const {
  Eigen::Matrix3d rotMat = q_.toRotationMatrix();
  m[0][0] = rotMat(0, 0);
  m[0][1] = rotMat(1, 0);
  m[0][2] = rotMat(2, 0);
  m[1][0] = rotMat(0, 1);
  m[1][1] = rotMat(1, 1);
  m[1][2] = rotMat(2, 1);
  m[2][0] = rotMat(0, 2);
  m[2][1] = rotMat(1, 2);
  m[2][2] = rotMat(2, 2);
  m[3][0] = t_(0);
  m[3][1] = t_(1);
  m[3][2] = t_(2);
  m[0][3] = 0;
  m[1][3] = 0;
  m[2][3] = 0;
  m[3][3] = 1.0l;
}

void Frame::get_matrix(double m[16]) const {
  Eigen::Matrix3d rotMat = q_.toRotationMatrix();
  m[0] = rotMat(0, 0);
  m[1] = rotMat(1, 0);
  m[2] = rotMat(2, 0);
  m[3] = 0;
  m[4] = rotMat(0, 1);
  m[5] = rotMat(1, 1);
  m[6] = rotMat(2, 1);
  m[7] = 0;
  m[8] = rotMat(0, 2);
  m[9] = rotMat(1, 2);
  m[10] = rotMat(2, 2);
  m[11] = 0;
  m[12] = t_(0);
  m[13] = t_(1);
  m[14] = t_(2);
  m[15] = 1.0l;
}

Frame Frame::inverse() const {
  Frame fr(-(q_.inverse()._transformVector(t_)), q_.inverse());
  fr.set_reference_frame(reference_frame());
  return fr;
}

const double *Frame::world_matrix() const {
  // This test is done for efficiency reasons (creates lots of temp objects
  // otherwise).
  if (reference_frame()) {
    static Frame fr;
    fr.set_translation(position());
    fr.set_rotation(orientation());
    return fr.matrix();
  } else {
    return matrix();
  }
}

void Frame::get_world_matrix(double m[4][4]) const {
  const double *mat = world_matrix();
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      m[i][j] = mat[i * 4 + j];
    }
  }
}

void Frame::get_world_matrix(double m[16]) const {
  const double *mat = world_matrix();
  for (int i = 0; i < 16; ++i) {
    m[i] = mat[i];
  }
}

void Frame::set_from_matrix(const double m[4][4]) {
  if (fabs(m[3][3]) < 1E-8) {
    std::cout << "Frame::set_from_matrix: Null homogeneous coefficient"
              << std::endl;
    return;
  }

  Eigen::Matrix3d rot;
  for (int i = 0; i < 3; ++i) {
    t_[i] = m[3][i] / m[3][3];
    for (int j = 0; j < 3; ++j) {
      rot(i, j) = m[j][i] / m[3][3];
    }
  }
  q_ = Eigen::Quaterniond(rot);
}

void Frame::set_from_matrix(const double m[16]) {
  double mat[4][4];
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      mat[i][j] = m[i * 4 + j];
    }
  }
  set_from_matrix(mat);
}

void Frame::set_translation(double x, double y, double z) {
  set_translation(Eigen::Vector3d(x, y, z));
}

void Frame::get_translation(double *x, double *y, double *z) const {
  const Eigen::Vector3d t = translation();
  *x = t(0);
  *y = t(1);
  *z = t(2);
}

void Frame::set_rotation(double q0, double q1, double q2, double q3) {
  set_rotation(Eigen::Quaterniond(q3, q0, q1, q2));
}

void Frame::get_rotation(double *q0, double *q1, double *q2, double *q3) const {
  const Eigen::Quaterniond q = rotation();
  *q0 = q.x();
  *q1 = q.y();
  *q2 = q.z();
  *q3 = q.w();
}

void Frame::translate(const Eigen::Vector3d &t) {
  Eigen::Vector3d tbis = t;
  translate(tbis);
}

void Frame::translate_t(const Eigen::Vector3d &t) { t_ += t; }

void Frame::translate(double x, double y, double z) {
  Eigen::Vector3d t(x, y, z);
  translate_t(t);
}

void Frame::translate(double *x, double *y, double *z) {
  Eigen::Vector3d t(*x, *y, *z);
  translate_t(t);
  *x = t(0);
  *y = t(1);
  *z = t(2);
}

void Frame::rotate(const Eigen::Quaterniond &q) {
  Eigen::Quaterniond qbis = q;
  rotate_q(qbis);
}

void Frame::rotate_q(const Eigen::Quaterniond &q) {
  q_ *= q;
  q_.normalize();  // Prevents numerical drift
}

void Frame::rotate(double *q0, double *q1, double *q2, double *q3) {
  Eigen::Quaterniond q(*q3, *q0, *q1, *q2);
  rotate_q(q);
  *q0 = q.x();
  *q1 = q.y();
  *q2 = q.z();
  *q3 = q.w();
}

void Frame::rotate(double q0, double q1, double q2, double q3) {
  Eigen::Quaterniond q(q3, q0, q1, q2);
  rotate_q(q);
}

void Frame::rotate_around_point(const Eigen::Quaterniond &rotation,
                                const Eigen::Vector3d &point) {
  q_ *= rotation;
  q_.normalize();  // Prevents numerical drift

  Eigen::Vector3d rotAxis = get_quaternion_axis(rotation);
  Eigen::Vector3d resRotAxis = inverse_transform_of(rotAxis);
  resRotAxis.normalize();
  double rotAngle = get_quaternion_angle(rotation);
  Eigen::AngleAxisd angleAxis(rotAngle, resRotAxis);

  Eigen::Vector3d trans =
      point +
      Eigen::Quaterniond(angleAxis)._transformVector(position() - point) - t_;

  t_ += trans;
}

void Frame::set_position(const Eigen::Vector3d &position) {
  if (reference_frame())
    set_translation(reference_frame()->coordinates_of(position));
  else
    set_translation(position);
}

void Frame::set_position(double x, double y, double z) {
  set_position(Eigen::Vector3d(x, y, z));
}

void Frame::set_position_and_orientation(
    const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation) {
  if (reference_frame()) {
    t_ = reference_frame()->coordinates_of(position);
    q_ = reference_frame()->orientation().inverse() * orientation;
  } else {
    t_ = position;
    q_ = orientation;
  }
}

void Frame::set_translation_and_rotation(const Eigen::Vector3d &translation,
                                         const Eigen::Quaterniond &rotation) {
  t_ = translation;
  q_ = rotation;
}

void Frame::get_position(double *x, double *y, double *z) const {
  Eigen::Vector3d p = position();
  *x = p(0);
  *y = p(1);
  *z = p(2);
}

void Frame::set_orientation(const Eigen::Quaterniond &orientation) {
  if (reference_frame())
    set_rotation(reference_frame()->orientation().inverse() * orientation);
  else
    set_rotation(orientation);
}

void Frame::set_orientation(double q0, double q1, double q2, double q3) {
  set_orientation(Eigen::Quaterniond(q3, q0, q1, q2));
}

void Frame::get_orientation(double *q0, double *q1, double *q2,
                            double *q3) const {
  Eigen::Quaterniond o = orientation();
  *q0 = o.x();
  *q1 = o.y();
  *q2 = o.z();
  *q3 = o.w();
}

Eigen::Quaterniond Frame::orientation() const {
  Eigen::Quaterniond res = rotation();
  const Frame *fr = reference_frame();
  while (fr != NULL) {
    res = fr->rotation() * res;
    fr = fr->reference_frame();
  }
  return res;
}

void Frame::set_reference_frame(const Frame *const refFrame) {
  if (setting_asreference_frame_will_create_a_loop(refFrame)) {
    std::cerr
        << "Frame::set_reference_frame would create a loop in Frame hierarchy"
        << std::endl;
  } else {
    reference_frame_ = refFrame;
  }
}

bool Frame::setting_asreference_frame_will_create_a_loop(
    const Frame *const frame) {
  const Frame *f = frame;
  while (f != NULL) {
    if (f == this) return true;
    f = f->reference_frame();
  }
  return false;
}

Eigen::Vector3d Frame::coordinates_of(const Eigen::Vector3d &src) const {
  if (reference_frame())
    return local_coordinates_of(reference_frame()->coordinates_of(src));
  else
    return local_coordinates_of(src);
}

Eigen::Vector3d Frame::inverse_coordinates_of(
    const Eigen::Vector3d &src) const {
  const Frame *fr = this;
  Eigen::Vector3d res = src;
  while (fr != NULL) {
    res = fr->local_inverse_coordinates_of(res);
    fr = fr->reference_frame();
  }
  return res;
}

Eigen::Vector3d Frame::local_coordinates_of(const Eigen::Vector3d &src) const {
  return rotation().inverse()._transformVector(src - translation());
}

Eigen::Vector3d Frame::local_inverse_coordinates_of(
    const Eigen::Vector3d &src) const {
  return rotation()._transformVector(src) + translation();
}

Eigen::Vector3d Frame::coordinates_of_from(const Eigen::Vector3d &src,
                                           const Frame *const from) const {
  if (this == from)
    return src;
  else if (reference_frame())
    return local_coordinates_of(
        reference_frame()->coordinates_of_from(src, from));
  else
    return local_coordinates_of(from->inverse_coordinates_of(src));
}

Eigen::Vector3d Frame::coordinates_of_in(const Eigen::Vector3d &src,
                                         const Frame *const in) const {
  const Frame *fr = this;
  Eigen::Vector3d res = src;
  while ((fr != NULL) && (fr != in)) {
    res = fr->local_inverse_coordinates_of(res);
    fr = fr->reference_frame();
  }

  if (fr != in)
    // in was not found in the branch of this, res is now expressed in the world
    // coordinate system. Simply convert to in coordinate system.
    res = in->coordinates_of(res);

  return res;
}

void Frame::get_coordinates_of(const double src[3], double res[3]) const {
  const Eigen::Vector3d r =
      coordinates_of(Eigen::Vector3d(src[0], src[1], src[2]));
  for (int i = 0; i < 3; ++i) {
    res[i] = r[i];
  }
}

void Frame::get_inverse_coordinates_of(const double src[3],
                                       double res[3]) const {
  const Eigen::Vector3d r =
      inverse_coordinates_of(Eigen::Vector3d(src[0], src[1], src[2]));
  for (int i = 0; i < 3; ++i) {
    res[i] = r[i];
  }
}

void Frame::get_local_coordinates_of(const double src[3], double res[3]) const {
  const Eigen::Vector3d r =
      local_coordinates_of(Eigen::Vector3d(src[0], src[1], src[2]));
  for (int i = 0; i < 3; ++i) {
    res[i] = r[i];
  }
}

void Frame::get_local_inverse_coordinates_of(const double src[3],
                                             double res[3]) const {
  const Eigen::Vector3d r =
      local_inverse_coordinates_of(Eigen::Vector3d(src[0], src[1], src[2]));
  for (int i = 0; i < 3; ++i) {
    res[i] = r[i];
  }
}

void Frame::get_coordinates_of_in(const double src[3], double res[3],
                                  const Frame *const in) const {
  const Eigen::Vector3d r =
      coordinates_of_in(Eigen::Vector3d(src[0], src[1], src[2]), in);
  for (int i = 0; i < 3; ++i) {
    res[i] = r[i];
  }
}

void Frame::get_coordinates_of_from(const double src[3], double res[3],
                                    const Frame *const from) const {
  const Eigen::Vector3d r =
      coordinates_of_from(Eigen::Vector3d(src[0], src[1], src[2]), from);
  for (int i = 0; i < 3; ++i) {
    res[i] = r[i];
  }
}

Eigen::Vector3d Frame::transform_of(const Eigen::Vector3d &src) const {
  if (reference_frame())
    return local_transform_of(reference_frame()->transform_of(src));
  else
    return local_transform_of(src);
}

Eigen::Vector3d Frame::inverse_transform_of(const Eigen::Vector3d &src) const {
  const Frame *fr = this;
  Eigen::Vector3d res = src;
  while (fr != NULL) {
    res = fr->local_inverse_transform_of(res);
    fr = fr->reference_frame();
  }
  return res;
}

Eigen::Vector3d Frame::local_transform_of(const Eigen::Vector3d &src) const {
  return rotation().inverse()._transformVector(src);
}

Eigen::Vector3d Frame::local_inverse_transform_of(
    const Eigen::Vector3d &src) const {
  return rotation()._transformVector(src);
}

Eigen::Vector3d Frame::transform_of_from(const Eigen::Vector3d &src,
                                         const Frame *const from) const {
  if (this == from)
    return src;
  else if (reference_frame())
    return local_transform_of(reference_frame()->transform_of_from(src, from));
  else
    return local_transform_of(from->inverse_transform_of(src));
}

Eigen::Vector3d Frame::transform_of_in(const Eigen::Vector3d &src,
                                       const Frame *const in) const {
  const Frame *fr = this;
  Eigen::Vector3d res = src;
  while ((fr != NULL) && (fr != in)) {
    res = fr->local_inverse_transform_of(res);
    fr = fr->reference_frame();
  }

  if (fr != in)
    // in was not found in the branch of this, res is now expressed in the world
    // coordinate system. Simply convert to in coordinate system.
    res = in->transform_of(res);

  return res;
}

void Frame::get_transform_of(const double src[3], double res[3]) const {
  Eigen::Vector3d r = transform_of(Eigen::Vector3d(src[0], src[1], src[2]));
  for (int i = 0; i < 3; ++i) {
    res[i] = r[i];
  }
}

void Frame::get_inverse_transform_of(const double src[3], double res[3]) const {
  Eigen::Vector3d r =
      inverse_transform_of(Eigen::Vector3d(src[0], src[1], src[2]));
  for (int i = 0; i < 3; ++i) {
    res[i] = r[i];
  }
}

void Frame::get_local_transform_of(const double src[3], double res[3]) const {
  Eigen::Vector3d r =
      local_transform_of(Eigen::Vector3d(src[0], src[1], src[2]));
  for (int i = 0; i < 3; ++i) {
    res[i] = r[i];
  }
}

void Frame::get_local_inverse_transform_of(const double src[3],
                                           double res[3]) const {
  Eigen::Vector3d r =
      local_inverse_transform_of(Eigen::Vector3d(src[0], src[1], src[2]));
  for (int i = 0; i < 3; ++i) {
    res[i] = r[i];
  }
}

void Frame::get_transform_of_in(const double src[3], double res[3],
                                const Frame *const in) const {
  Eigen::Vector3d r =
      transform_of_in(Eigen::Vector3d(src[0], src[1], src[2]), in);
  for (int i = 0; i < 3; ++i) {
    res[i] = r[i];
  }
}

void Frame::get_transform_of_from(const double src[3], double res[3],
                                  const Frame *const from) const {
  Eigen::Vector3d r =
      transform_of_from(Eigen::Vector3d(src[0], src[1], src[2]), from);
  for (int i = 0; i < 3; ++i) {
    res[i] = r[i];
  }
}

void Frame::align_with_frame(const Frame *const frame, bool move,
                             double threshold) {
  Eigen::Vector3d directions[2][3];
  for (int d = 0; d < 3; ++d) {
    Eigen::Vector3d dir((d == 0) ? 1.0 : 0.0, (d == 1) ? 1.0 : 0.0,
                        (d == 2) ? 1.0 : 0.0);
    if (frame) {
      directions[0][d] = frame->inverse_transform_of(dir);
    } else {
      directions[0][d] = dir;
    }
    directions[1][d] = inverse_transform_of(dir);
  }

  double maxProj = 0.0f;
  double proj = 0.0;
  int index[2];
  index[0] = index[1] = 0;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      if ((proj = fabs(directions[0][i].dot(directions[1][j]))) >= maxProj) {
        index[0] = i;
        index[1] = j;
        maxProj = proj;
      }
    }
  }

  Frame old;
  old = *this;

  double coef = directions[0][index[0]].dot(directions[1][index[1]]);
  if (fabs(coef) >= threshold) {
    const Eigen::Vector3d axis =
        directions[0][index[0]].cross(directions[1][index[1]]);
    double angle = asin(axis.norm());
    if (coef >= 0.0) angle = -angle;
    // set_orientation(Eigen::Quaterniond(axis, angle) * orientation());
    // axis.normalize();
    Eigen::AngleAxisd angleAxis(angle, axis);
    rotate(rotation().inverse() * Eigen::Quaterniond(angleAxis) *
           orientation());

    // Try to align an other axis direction
    int d = (index[1] + 1) % 3;
    Eigen::Vector3d dir((d == 0) ? 1.0 : 0.0, (d == 1) ? 1.0 : 0.0,
                        (d == 2) ? 1.0 : 0.0);
    dir = inverse_transform_of(dir);

    double max = 0.0f;
    for (int i = 0; i < 3; ++i) {
      double proj = fabs(directions[0][i].dot(dir));
      if (proj > max) {
        index[0] = i;
        max = proj;
      }
    }

    if (max >= threshold) {
      const Eigen::Vector3d axis = directions[0][index[0]].cross(dir);
      double angle = asin(axis.norm());
      if (directions[0][index[0]].dot(dir) >= 0.0) angle = -angle;
      // set_orientation(Eigen::Quaterniond(axis, angle) * orientation());
      // axis.normalize();
      Eigen::AngleAxisd angleAxis(angle, axis);
      rotate(rotation().inverse() * Eigen::Quaterniond(angleAxis) *
             orientation());
    }
  }

  if (move) {
    Eigen::Vector3d center;
    if (frame) center = frame->position();

    // set_position(center - orientation().rotate(old.coordinates_of(center)));
    translate(center -
              orientation()._transformVector(old.coordinates_of(center)) -
              translation());
  }
}

void Frame::project_on_line(const Eigen::Vector3d &origin,
                            const Eigen::Vector3d &direction) {
  const Eigen::Vector3d shift = origin - position();
  Eigen::Vector3d proj = shift;

  if (direction.squaredNorm() < 1.0E-10) {
    std::cout << "Vec::projectOnAxis: axis direction is not normalized (norm)."
              << std::endl;
  }

  proj = (proj.dot(direction) / direction.squaredNorm()) * direction;
  translate(shift - proj);
}

}  // namespace lowcostvisualizer
}  // namespace perception
}  // namespace apollo
