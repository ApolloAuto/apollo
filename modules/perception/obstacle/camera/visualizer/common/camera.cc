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

#include "modules/perception/obstacle/camera/visualizer/common/camera.h"

#include <cmath>

#include <algorithm>
#include <iostream>

namespace apollo {
namespace perception {
namespace lowcostvisualizer {

// using namespace std;
Camera::Camera() : field_of_view_(M_PI / 4.0f) {
  set_frame(new Frame());

  setscene_radius(1.0);
  ortho_coef_ = tan(field_of_view() / 2.0);
  setscene_center(Eigen::Vector3d(0.0, 0.0, 0.0));
  set_type(PERSPECTIVE);

  // #CONNECTION# initFromDOMElement default values
  setznear_coefficient(0.005f);
  setzclipping_coefficient(sqrt(3.0));

  // Dummy values
  setscreen_widthandheight(600, 400);

  // Stereo parameters
  setio_distance(0.062f);
  setphysical_distance_to_screen(0.5f);
  set_physicalscreen_width(0.4f);
  // focus_distance is set from setfield_of_view()

  for (int j = 0; j < 16; ++j) {
    model_view_matrix_[j] = ((j % 5 == 0) ? 1.0 : 0.0);
    projection_matrix_[j] = 0.0;
  }
  compute_projection_matrix();
}

Camera::~Camera() { delete frame_; }

Camera::Camera(const Camera &camera) {
  set_frame(new Frame());

  for (int j = 0; j < 16; ++j) {
    model_view_matrix_[j] = ((j % 5 == 0) ? 1.0 : 0.0);
    projection_matrix_[j] = 0.0;
  }

  (*this) = camera;
}

Camera &Camera::operator=(const Camera &camera) {
  setscreen_widthandheight(camera.screen_width(), camera.screen_height());
  setfield_of_view(camera.field_of_view());
  setscene_radius(camera.scene_radius());
  setscene_center(camera.scene_center());
  setznear_coefficient(camera.znear_coefficient());
  setzclipping_coefficient(camera.zclipping_coefficient());
  set_type(camera.type());

  // Stereo parameters
  setio_distance(camera.io_distance());
  setfocus_distance(camera.focus_distance());
  set_physicalscreen_width(camera.physicalscreen_width());
  setphysical_distance_to_screen(camera.physical_distance_to_screen());

  ortho_coef_ = camera.ortho_coef_;

  // frame_ and interpolationKfi_ pointers are not shared.
  frame_->set_reference_frame(NULL);
  frame_->set_position(camera.position());
  frame_->set_orientation(camera.orientation());

  compute_projection_matrix();
  compute_model_view_matrix();

  return *this;
}

void Camera::setscreen_widthandheight(int width, int height) {
  // Prevent negative and zero dimensions that would cause divisions by zero.
  screen_width_ = width > 0 ? width : 1;
  screen_height_ = height > 0 ? height : 1;
}

double Camera::znear() const {
  double z =
      distance_to_scene_center() - zclipping_coefficient() * scene_radius();

  // Prevents negative or null zNear values.
  const double zMin =
      znear_coefficient() * zclipping_coefficient() * scene_radius();

  if (z < zMin) {
    switch (type()) {
      case Camera::PERSPECTIVE:
        z = zMin;
        break;
      case Camera::ORTHOGRAPHIC:
        z = 0.0;
        break;
    }
  }
  return z;
}

double Camera::zfar() const {
  /*double dis = distance_to_scene_center();
  double z = zclipping_coefficient();
  double s = scene_radius();
  double dd = dis + z*s;*/
  return distance_to_scene_center() + zclipping_coefficient() * scene_radius();
}

void Camera::set_type(Type type) {
  // make ORTHOGRAPHIC frustum fit PERSPECTIVE (at least in plane normal to
  // view_direction(), passing
  // through RAP). Done only when CHANGING type since ortho_coef_ may have been
  // changed with a
  // set_revolve_around_point() in the meantime.
  if ((type == Camera::ORTHOGRAPHIC) && (type_ == Camera::PERSPECTIVE))
    ortho_coef_ = tan(field_of_view() / 2.0);
  type_ = type;
}

void Camera::set_frame(Frame *const mcf) {
  if (!mcf) {
    return;
  }

  frame_ = mcf;
}

double Camera::distance_to_scene_center() const {
  // Eigen::Vector3d center = scene_center();
  // Eigen::Vector3d xcen = frame()->coordinates_of(center);

  return fabs((frame()->coordinates_of(scene_center()))(2));
}

void Camera::get_ortho_width_height(GLdouble *halfWidth,
                                    GLdouble *halfHeight) const {
  const double dist =
      ortho_coef_ * fabs(cameracoordinates_of(revolve_around_point())(2));
  // #CONNECTION# fit_screen_region
  *halfWidth = dist * ((aspect_ratio() < 1.0) ? 1.0 : aspect_ratio());
  *halfHeight = dist * ((aspect_ratio() < 1.0) ? 1.0 / aspect_ratio() : 1.0);
}

void Camera::compute_projection_matrix() const {
  const double ZNear = znear();
  const double ZFar = zfar();

  switch (type()) {
    case Camera::PERSPECTIVE: {
      // #CONNECTION# all non null coefficients were set to 0.0 in constructor.
      const double f = 1.0 / tan(field_of_view() / 2.0);
      projection_matrix_[0] = f / aspect_ratio();
      projection_matrix_[5] = f;
      projection_matrix_[10] = (ZNear + ZFar) / (ZNear - ZFar);
      projection_matrix_[11] = -1.0;
      projection_matrix_[14] = 2.0 * ZNear * ZFar / (ZNear - ZFar);
      projection_matrix_[15] = 0.0;
      // same as gluPerspective( 180.0*field_of_view()/M_PI, aspect_ratio(),
      // znear(), zfar() );
      break;
    }
    case Camera::ORTHOGRAPHIC: {
      GLdouble w = 0.0;
      GLdouble h = 0.0;
      get_ortho_width_height(&w, &h);
      projection_matrix_[0] = 1.0 / w;
      projection_matrix_[5] = 1.0 / h;
      projection_matrix_[10] = -2.0 / (ZFar - ZNear);
      projection_matrix_[11] = 0.0;
      projection_matrix_[14] = -(ZFar + ZNear) / (ZFar - ZNear);
      projection_matrix_[15] = 1.0;
      // same as glOrtho( -w, w, -h, h, znear(), zfar() );
      break;
    }
  }
}

void Camera::compute_model_view_matrix() const {
  const Eigen::Quaterniond q = frame()->orientation();
  Eigen::Matrix3d rotMat = q.toRotationMatrix();

  model_view_matrix_[0] = rotMat(0, 0);
  model_view_matrix_[1] = rotMat(0, 1);
  model_view_matrix_[2] = rotMat(0, 2);
  model_view_matrix_[3] = 0.0l;

  model_view_matrix_[4] = rotMat(1, 0);
  model_view_matrix_[5] = rotMat(1, 1);
  model_view_matrix_[6] = rotMat(1, 2);
  model_view_matrix_[7] = 0.0l;

  model_view_matrix_[8] = rotMat(2, 0);
  model_view_matrix_[9] = rotMat(2, 1);
  model_view_matrix_[10] = rotMat(2, 2);
  model_view_matrix_[11] = 0.0l;

  const Eigen::Vector3d t = q.inverse()._transformVector(frame()->position());

  model_view_matrix_[12] = -t(0);
  model_view_matrix_[13] = -t(1);
  model_view_matrix_[14] = -t(2);
  model_view_matrix_[15] = 1.0l;
}

void Camera::load_projection_matrix(bool reset) const {
  // WARNING: makeCurrent must be called by every calling method
  glMatrixMode(GL_PROJECTION);

  if (reset) glLoadIdentity();

  compute_projection_matrix();

  glMultMatrixd(projection_matrix_);
}

void Camera::load_model_view_matrix(bool reset) const {
  // WARNING: makeCurrent must be called by every calling method
  glMatrixMode(GL_MODELVIEW);
  compute_model_view_matrix();
  if (reset)
    glLoadMatrixd(model_view_matrix_);
  else
    glMultMatrixd(model_view_matrix_);
}

void Camera::load_projection_matrix_stereo(bool leftBuffer) const {
  double left = 0.0;
  double right = 0.0;
  double bottom = 0.0;
  double top = 0.0;

  double screenHalfWidth = 0.0;
  double halfWidth = 0.0;
  double side = 0.0;
  double shift = 0.0;
  double delta = 0.0;
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

  switch (type()) {
    case Camera::PERSPECTIVE:
      // compute half width of screen,
      // corresponding to zero parallax plane to deduce decay of cameras
      screenHalfWidth = focus_distance() * tan(horizontalfield_of_view() / 2.0);
      shift = screenHalfWidth * io_distance() / physicalscreen_width();
      // should be * current y    / y total
      // to take into account that the window doesn't cover the entire screen

      // compute half width of "view" at znear and the delta corresponding to
      // the shifted camera to deduce what to set for asymmetric frustums
      halfWidth = znear() * tan(horizontalfield_of_view() / 2.0);
      delta = shift * znear() / focus_distance();
      side = leftBuffer ? -1.0 : 1.0;

      left = -halfWidth + side * delta;
      right = halfWidth + side * delta;
      top = halfWidth / aspect_ratio();
      bottom = -top;
      glFrustum(left, right, bottom, top, znear(), zfar());
      break;

    case Camera::ORTHOGRAPHIC:
      std::cout << "Camera::setProjectionMatrixStereo: Stereo not available "
                   "with Ortho mode"
                << std::endl;
      break;
  }
}

void Camera::load_model_view_matrix_stereo(bool leftBuffer) const {
  // WARNING: makeCurrent must be called by every calling method
  glMatrixMode(GL_MODELVIEW);

  double halfWidth = focus_distance() * tan(horizontalfield_of_view() / 2.0);
  double shift =
      halfWidth * io_distance() /
      physicalscreen_width();  // * current window width / full screen width

  compute_model_view_matrix();
  if (leftBuffer) {
    model_view_matrix_[12] -= shift;
  } else {
    model_view_matrix_[12] += shift;
  }
  glLoadMatrixd(model_view_matrix_);
}

void Camera::get_projection_matrix(GLdouble m[16]) const {
  // May not be needed, but easier and more robust like this.
  compute_projection_matrix();
  for (int i = 0; i < 16; ++i) {
    m[i] = projection_matrix_[i];
  }
}

void Camera::get_model_view_matrix(GLdouble m[16]) const {
  // May not be needed, but easier like this.
  // Prevents from retrieving matrix in stereo mode -> overwrites shifted value.
  compute_model_view_matrix();
  for (int i = 0; i < 16; ++i) {
    m[i] = model_view_matrix_[i];
  }
}

/*! Fills \p m with the product of the ModelView and Projection matrices.

Calls get_model_view_matrix() and get_projection_matrix() and then fills \p
m with the product of these two matrices. */
void Camera::get_model_view_projection_matrix(GLdouble m[16]) const {
  GLdouble mv[16];
  GLdouble proj[16];
  get_model_view_matrix(mv);
  get_projection_matrix(proj);

  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      double sum = 0.0;
      for (int k = 0; k < 4; ++k) {
        sum += proj[i + 4 * k] * mv[k + 4 * j];
      }
      m[i + 4 * j] = sum;
    }
  }
}

void Camera::setscene_radius(const double radius) {
  if (radius <= 0.0) {
    std::cout << "Scene radius must be positive - Ignoring value" << std::endl;
    return;
  }

  scene_radius_ = radius;

  setfocus_distance(scene_radius() / tan(field_of_view() / 2.0));
}

/*! Similar to setscene_radius() and setscene_center(), but the scene limits are
defined by a (world
axis aligned) bounding box. */
void Camera::set_scene_bounding_box(const Eigen::Vector3d &min,
                                    const Eigen::Vector3d &max) {
  setscene_center((min + max) / 2.0);
  setscene_radius(0.5 * (max - min).norm());
}

void Camera::setscene_center(const Eigen::Vector3d &center) {
  scene_center_ = center;
  set_revolve_around_point(scene_center());
}

/*! setscene_center() to the result of point_under_pixel(\p pixel).

Returns \c true if a point_under_pixel() was found and scene_center() was
actually changed.

See also set_revolve_around_point_from_pixel(). See the point_under_pixel()
documentation. */
bool Camera::setscene_center_from_pixel(const Eigen::Vector2i &pixel) {
  bool found = false;
  Eigen::Vector3d point = point_under_pixel(pixel, &found);
  if (found) {
    setscene_center(point);
  }
  return found;
}

/*! Changes the revolve_around_point() to \p rap (defined in the world
 * coordinate system). */
void Camera::set_revolve_around_point(const Eigen::Vector3d &rap) {
  const double prevDist = fabs(cameracoordinates_of(revolve_around_point())(2));

  revolve_around_point_ = rap;

  // ortho_coef_ is used to compensate for changes of the revolve_around_point,
  // so that the image does
  // not change when the revolve_around_point is changed in ORTHOGRAPHIC mode.
  const double newDist = fabs(cameracoordinates_of(revolve_around_point())(2));
  // Prevents division by zero when rap is set to camera position
  if ((prevDist > 1E-9) && (newDist > 1E-9)) {
    ortho_coef_ *= prevDist / newDist;
  }
}

bool Camera::set_revolve_around_point_from_pixel(const Eigen::Vector2i &pixel) {
  bool found = false;
  Eigen::Vector3d point = point_under_pixel(pixel, &found);
  if (found) {
    set_revolve_around_point(point);
  }
  return found;
}

double Camera::pixelgl_ratio(const Eigen::Vector3d &position) const {
  switch (type()) {
    case Camera::PERSPECTIVE:
      return 2.0 * fabs((frame()->coordinates_of(position))(2)) *
             tan(field_of_view() / 2.0) / screen_height();
    case Camera::ORTHOGRAPHIC: {
      GLdouble w = 0.0;
      GLdouble h = 0.0;
      get_ortho_width_height(&w, &h);
      return 2.0 * h / screen_height();
    }
  }
  // Bad compilers complain
  return 1.0;
}

void Camera::setfov_to_fit_scene() {
  if (distance_to_scene_center() > sqrt(2.0) * scene_radius()) {
    setfield_of_view(2.0 * asin(scene_radius() / distance_to_scene_center()));
  } else {
    setfield_of_view(M_PI / 2.0f);
  }
}

Eigen::Vector3d Camera::point_under_pixel(const Eigen::Vector2i &pixel,
                                          bool *found) const {
  double depth = 0.0;
  // Qt uses upper corner for its origin while GL uses the lower corner.
  glReadPixels(pixel(0), screen_height() - 1 - pixel(1), 1, 1,
               GL_DEPTH_COMPONENT, GL_FLOAT, &depth);
  *found = depth < 1.0;
  Eigen::Vector3d point(pixel(0), pixel(1), depth);
  point = unprojectedcoordinates_of(point);
  return point;
}

void Camera::show_entire_scene() { fit_sphere(scene_center(), scene_radius()); }

void Camera::center_scene() {
  frame()->project_on_line(scene_center(), view_direction());
}

void Camera::look_at(const Eigen::Vector3d &target) {
  setview_direction(target - position());
}

void Camera::fit_sphere(const Eigen::Vector3d &center, double radius) {
  double distance = 0.0f;
  switch (type()) {
    case Camera::PERSPECTIVE: {
      const double yview = radius / sin(field_of_view() / 2.0);
      const double xview = radius / sin(horizontalfield_of_view() / 2.0);
      distance = std::max(xview, yview);
      break;
    }
    case Camera::ORTHOGRAPHIC: {
      distance = (center - revolve_around_point()).dot(view_direction()) +
                 (radius / ortho_coef_);
      break;
    }
  }
  Eigen::Vector3d newPos(center - distance * view_direction());
  frame()->set_position(newPos);
}

void Camera::fit_bounding_box(const Eigen::Vector3d &min,
                              const Eigen::Vector3d &max) {
  double diameter = std::max(fabs(max[1] - min[1]), fabs(max[0] - min[0]));
  diameter = std::max(fabs(max[2] - min[2]), diameter);
  fit_sphere(0.5 * (min + max), 0.5 * diameter);
}

/* Pan the camera */
void Camera::pan_by_mouse(int deltaX, int deltaY) {
  Eigen::Vector3d trans(static_cast<double>(deltaX),
                        static_cast<double>(-deltaY), 0.0);
  // Scale to fit the screen mouse displacement
  switch (type()) {
    case Camera::PERSPECTIVE:
      trans *= 2.0 * tan(field_of_view() / 2.0) *
               fabs((frame()->coordinates_of(revolve_around_point()))(2)) /
               screen_height();
      break;
    case Camera::ORTHOGRAPHIC: {
      GLdouble w = 0.0;
      GLdouble h = 0.0;
      get_ortho_width_height(&w, &h);
      trans[0] *= 2.0 * w / screen_width();
      trans[1] *= 2.0 * h / screen_height();
      break;
    }
  }
  frame()->translate(frame()->inverse_transform_of(-trans));
}

/* Rotate under the control of mouse*/
void Camera::rotate_by_mouse_from_qgwidget(int preX, int preY, int x, int y) {
  Eigen::Vector3d trans = projectedcoordinates_of(revolve_around_point());
  Eigen::Quaterniond rot =
      deformed_ball_quaternion(preX, preY, x, y, trans[0], trans[1]);
  frame()->rotate_around_point(rot, revolve_around_point());
}

Eigen::Quaterniond Camera::get_rotatation_by_mouse_from_qgwidget(int preX,
                                                                 int preY,
                                                                 int x, int y) {
  Eigen::Vector3d trans = projectedcoordinates_of(revolve_around_point());
  Eigen::Quaterniond rot =
      deformed_ball_quaternion(preX, preY, x, y, trans[0], trans[1]);
  return rot;
}

void Camera::rotate_by_mouse(int deltaX, int deltaY) {
  // Eigen::Vector3d p = revolve_around_point();
  // get the current angular parameters
  Eigen::Vector3d rightVec = right_vector();
  Eigen::Vector3d upVec = up_vector();

  Eigen::Vector3d tran = -rightVec * deltaX + upVec * deltaY;
  Eigen::Vector3d tgtPos = position() + tran * 0.1;
  Eigen::Vector3d tgtVec = tgtPos - revolve_around_point();
  tgtVec.normalize();

  Eigen::Vector3d curVec = position() - revolve_around_point();
  curVec.normalize();

  Eigen::Vector3d axis = curVec.cross(tgtVec);
  axis.normalize();
  double cosAngle = curVec.dot(tgtVec);
  double angle = 0.0;
  if (cosAngle > 1) {
    angle = 0;
  } else if (cosAngle < -1) {
    angle = 3.141592653;
  } else {
    angle = acos(cosAngle);
  }

  Eigen::AngleAxisd angleAxis(angle, axis);
  Eigen::Quaterniond rot(angleAxis);
  // frame()->rotate_around_point(rot, revolve_around_point());
  frame()->rotate(rot);
  frame()->set_position((position() - revolve_around_point()).norm() * tgtVec +
                        revolve_around_point());
  //    look_at(revolve_around_point());
}

void Camera::rotate(Eigen::Vector3d i_axis, double i_angle) {
  frame()->rotate_around_point(
      Eigen::Quaterniond(i_angle, i_axis(0), i_axis(1), i_axis(2)),
      revolve_around_point());
}

void Camera::setup_vector(const Eigen::Vector3d &up, bool noMove) {
  Eigen::Quaterniond q = Eigen::Quaterniond().FromTwoVectors(
      Eigen::Vector3d(0.0, 1.0, 0.0), frame()->transform_of(up));

  if (!noMove) {
    frame()->set_position(
        revolve_around_point() -
        (frame()->orientation() * q)
            ._transformVector(frame()->coordinates_of(revolve_around_point())));
  }

  frame()->rotate(q);
}

void Camera::set_orientation(double theta, double phi) {
  Eigen::Vector3d axis(0.0, 1.0, 0.0);
  const Eigen::Quaterniond rot1(theta, axis(0), axis(1), axis(2));
  axis = Eigen::Vector3d(-cos(theta), 0., sin(theta));
  const Eigen::Quaterniond rot2(phi, axis(0), axis(1), axis(2));
  set_orientation(rot1 * rot2);
}

void Camera::set_orientation(const Eigen::Quaterniond &q) {
  frame()->set_orientation(q);
}

static void setFromRotatedBasis(Eigen::Quaterniond *q, const Eigen::Vector3d &X,
                                const Eigen::Vector3d &Y,
                                const Eigen::Vector3d &Z) {
  Eigen::Matrix3d m;
  double normX = X.norm();
  double normY = Y.norm();
  double normZ = Z.norm();
  for (int i = 0; i < 3; ++i) {
    m(i, 0) = X[i] / normX;
    m(i, 1) = Y[i] / normY;
    m(i, 2) = Z[i] / normZ;
  }
  *q = Eigen::Quaterniond(m);
}

void Camera::setview_direction(const Eigen::Vector3d &direction) {
  if (direction.squaredNorm() < 1E-10) {
    return;
  }

  Eigen::Vector3d xAxis = direction.cross(up_vector());
  if (xAxis.squaredNorm() < 1E-10) {
    // target is aligned with up_vector, this means a rotation around X axis
    // X axis is then unchanged, let's keep it !
    xAxis = frame()->inverse_transform_of(Eigen::Vector3d(1.0, 0.0, 0.0));
  }

  Eigen::Quaterniond q;
  setFromRotatedBasis(&q, xAxis, xAxis.cross(direction), -direction);
  frame()->set_orientation(q);
}

static double det(double m00, double m01, double m02, double m10, double m11,
                  double m12, double m20, double m21, double m22) {
  return m00 * m11 * m22 + m01 * m12 * m20 + m02 * m10 * m21 - m20 * m11 * m02 -
         m10 * m01 * m22 - m00 * m21 * m12;
}

// Computes the index of element [i][j] in a \c double matrix[3][4].
static inline unsigned int ind(unsigned int i, unsigned int j) {
  return (i * 4 + j);
}

void Camera::set_from_model_view_matrix(const GLdouble *const modelViewMatrix) {
  // Get upper left (rotation) matrix
  Eigen::Matrix3d upperLeft;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      upperLeft(i, j) = modelViewMatrix[i * 4 + j];
    }
  }

  // Transform upperLeft into the associated Quaternion
  Eigen::Quaterniond q(upperLeft);
  set_orientation(q);
  set_position(-q._transformVector(Eigen::Vector3d(
      modelViewMatrix[12], modelViewMatrix[13], modelViewMatrix[14])));
}

void Camera::set_from_projection_matrix(const double matrix[12]) {
  // The 3 lines of the matrix are the normals to the planes x=0, y=0, z=0
  // in the camera CS. As we normalize them, we do not need the 4th coordinate.
  Eigen::Vector3d line_0(matrix[ind(0, 0)], matrix[ind(0, 1)],
                         matrix[ind(0, 2)]);
  Eigen::Vector3d line_1(matrix[ind(1, 0)], matrix[ind(1, 1)],
                         matrix[ind(1, 2)]);
  Eigen::Vector3d line_2(matrix[ind(2, 0)], matrix[ind(2, 1)],
                         matrix[ind(2, 2)]);

  line_0.normalize();
  line_1.normalize();
  line_2.normalize();

  // The camera position is at (0, 0, 0) in the camera CS so it is the
  // intersection of the 3 planes. It can be seen as the kernel
  // of the 3x4 projection matrix. We calculate it through 4 dimensional
  // vectorial product. We go directly into 3D that is to say we directly
  // divide the first 3 coordinates by the 4th one.

  // We derive the 4 dimensional vectorial product formula from the
  // computation of a 4x4 determinant that is developped according to
  // its 4th column. This implies some 3x3 determinants.
  const Eigen::Vector3d cam_pos =
      Eigen::Vector3d(
          det(matrix[ind(0, 1)], matrix[ind(0, 2)], matrix[ind(0, 3)],
              matrix[ind(1, 1)], matrix[ind(1, 2)], matrix[ind(1, 3)],
              matrix[ind(2, 1)], matrix[ind(2, 2)], matrix[ind(2, 3)]),

          -det(matrix[ind(0, 0)], matrix[ind(0, 2)], matrix[ind(0, 3)],
               matrix[ind(1, 0)], matrix[ind(1, 2)], matrix[ind(1, 3)],
               matrix[ind(2, 0)], matrix[ind(2, 2)], matrix[ind(2, 3)]),

          det(matrix[ind(0, 0)], matrix[ind(0, 1)], matrix[ind(0, 3)],
              matrix[ind(1, 0)], matrix[ind(1, 1)], matrix[ind(1, 3)],
              matrix[ind(2, 0)], matrix[ind(2, 1)], matrix[ind(2, 3)])) /

      (-det(matrix[ind(0, 0)], matrix[ind(0, 1)], matrix[ind(0, 2)],
            matrix[ind(1, 0)], matrix[ind(1, 1)], matrix[ind(1, 2)],
            matrix[ind(2, 0)], matrix[ind(2, 1)], matrix[ind(2, 2)]));

  // We compute the rotation matrix column by column.

  // GL Z axis is front facing.
  Eigen::Vector3d column_2 = -line_2;

  // X-axis is almost like line_0 but should be orthogonal to the Z axis.
  Eigen::Vector3d column_0 = column_2.cross(line_0).cross(column_2);
  column_0.normalize();

  // Y-axis is almost like line_1 but should be orthogonal to the Z axis.
  // Moreover line_1 is downward oriented as the screen CS.
  Eigen::Vector3d column_1 = -(column_2.cross(line_1).cross(column_2));
  column_1.normalize();

  Eigen::Matrix3d rot;
  rot(0, 0) = column_0[0];
  rot(1, 0) = column_0[1];
  rot(2, 0) = column_0[2];

  rot(0, 1) = column_1[0];
  rot(1, 1) = column_1[1];
  rot(2, 1) = column_1[2];

  rot(0, 2) = column_2[0];
  rot(1, 2) = column_2[1];
  rot(2, 2) = column_2[2];

  // We compute the field of view

  // line_1^column_0 -> vector of intersection line between
  // y_screen=0 and x_camera=0 plane.
  // column_2*(...)    -> cos of the angle between Z vector et y_screen=0 plane
  // * 2 -> field of view = 2 * half angle

  // We need some intermediate values.
  Eigen::Vector3d dummy = line_1.cross(column_0);
  dummy.normalize();
  double fov = acos(column_2.dot(dummy) * 2.0);

  // We set the camera.
  Eigen::Quaterniond q(rot);
  set_orientation(q);
  set_position(cam_pos);
  setfield_of_view(fov);
}

void Camera::get_cameracoordinates_of(const double src[3],
                                      double res[3]) const {
  Eigen::Vector3d r =
      cameracoordinates_of(Eigen::Vector3d(src[0], src[1], src[2]));
  for (int i = 0; i < 3; ++i) {
    res[i] = r[i];
  }
}

void Camera::get_worldcoordinates_of(const double src[3], double res[3]) const {
  Eigen::Vector3d r =
      worldcoordinates_of(Eigen::Vector3d(src[0], src[1], src[2]));
  for (int i = 0; i < 3; ++i) {
    res[i] = r[i];
  }
}

void Camera::get_viewport(GLint viewport[4]) const {
  viewport[0] = 0;
  viewport[1] = screen_height();
  viewport[2] = screen_width();
  viewport[3] = -screen_height();
}

Eigen::Vector3d Camera::projectedcoordinates_of(const Eigen::Vector3d &src,
                                                const Frame *frame) const {
  GLdouble x = 0.0;
  GLdouble y = 0.0;
  GLdouble z = 0.0;
  static GLint viewport[4];
  get_viewport(viewport);

  if (frame) {
    const Eigen::Vector3d tmp = frame->inverse_coordinates_of(src);
    gluProject(tmp(0), tmp(1), tmp(2), model_view_matrix_, projection_matrix_,
               viewport, &x, &y, &z);
  } else {
    gluProject(src(0), src(1), src(2), model_view_matrix_, projection_matrix_,
               viewport, &x, &y, &z);
  }
  return Eigen::Vector3d(x, y, z);
}

Eigen::Vector3d Camera::unprojectedcoordinates_of(const Eigen::Vector3d &src,
                                                  const Frame *frame) const {
  GLdouble x = 0.0;
  GLdouble y = 0.0;
  GLdouble z = 0.0;
  static GLint viewport[4];
  get_viewport(viewport);
  gluUnProject(src(0), src(1), src(2), model_view_matrix_, projection_matrix_,
               viewport, &x, &y, &z);
  if (frame) {
    return frame->coordinates_of(Eigen::Vector3d(x, y, z));
  } else {
    return Eigen::Vector3d(x, y, z);
  }
}

void Camera::get_projectedcoordinates_of(const double src[3], double res[3],
                                         const Frame *frame) const {
  Eigen::Vector3d r =
      projectedcoordinates_of(Eigen::Vector3d(src[0], src[1], src[2]), frame);
  for (int i = 0; i < 3; ++i) {
    res[i] = r[i];
  }
}

void Camera::get_unprojectedcoordinates_of(const double src[3], double res[3],
                                           const Frame *frame) const {
  Eigen::Vector3d r =
      unprojectedcoordinates_of(Eigen::Vector3d(src[0], src[1], src[2]), frame);
  for (int i = 0; i < 3; ++i) {
    res[i] = r[i];
  }
}

void Camera::draw(bool drawFarPlane, double scale) const {
  glPushMatrix();
  glMultMatrixd(frame()->world_matrix());

  // 0 is the upper left coordinates of the near corner, 1 for the far one
  Eigen::Vector3d points[2];

  points[0](2) = scale * znear();
  points[1](2) = scale * zfar();

  switch (type()) {
    case Camera::PERSPECTIVE: {
      points[0](1) = points[0](2) * std::tan(field_of_view() / 2.0);
      points[0](0) = points[0](1) * aspect_ratio();

      const double ratio = points[1](2) / points[0](2);

      points[1](1) = ratio * points[0](1);
      points[1](0) = ratio * points[0](0);
      break;
    }
    case Camera::ORTHOGRAPHIC: {
      GLdouble hw = 0.0;
      GLdouble hh = 0.0;
      get_ortho_width_height(&hw, &hh);
      points[0](0) = points[1](0) = scale * static_cast<double>(hw);
      points[0](1) = points[1](1) = scale * static_cast<double>(hh);
      break;
    }
  }

  const int farIndex = drawFarPlane ? 1 : 0;

  // Near and (optionally) far plane(s)
  glBegin(GL_QUADS);
  for (int i = farIndex; i >= 0; --i) {
    glNormal3f(0.0, 0.0, (i == 0) ? 1.0 : -1.0);
    glVertex3f(points[i](0), points[i](1), -points[i](2));
    glVertex3f(-points[i](0), points[i](1), -points[i](2));
    glVertex3f(-points[i](0), -points[i](1), -points[i](2));
    glVertex3f(points[i](0), -points[i](1), -points[i](2));
  }
  glEnd();

  // Up arrow
  const double arrowHeight = 1.5f * points[0](1);
  const double baseHeight = 1.2f * points[0](1);
  const double arrowHalfWidth = 0.5f * points[0](0);
  const double baseHalfWidth = 0.3f * points[0](0);

  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  // Base
  glBegin(GL_QUADS);
  glVertex3f(-baseHalfWidth, points[0](1), -points[0](2));
  glVertex3f(baseHalfWidth, points[0](1), -points[0](2));
  glVertex3f(baseHalfWidth, baseHeight, -points[0](2));
  glVertex3f(-baseHalfWidth, baseHeight, -points[0](2));
  glEnd();

  // Arrow
  glBegin(GL_TRIANGLES);
  glVertex3f(0.0f, arrowHeight, -points[0](2));
  glVertex3f(-arrowHalfWidth, baseHeight, -points[0](2));
  glVertex3f(arrowHalfWidth, baseHeight, -points[0](2));
  glEnd();

  // Frustum lines
  switch (type()) {
    case Camera::PERSPECTIVE:
      glBegin(GL_LINES);
      glVertex3f(0.0f, 0.0f, 0.0f);
      glVertex3f(points[farIndex](0), points[farIndex](1),
                 -points[farIndex](2));
      glVertex3f(0.0f, 0.0f, 0.0f);
      glVertex3f(-points[farIndex](0), points[farIndex](1),
                 -points[farIndex](2));
      glVertex3f(0.0f, 0.0f, 0.0f);
      glVertex3f(-points[farIndex](0), -points[farIndex](1),
                 -points[farIndex](2));
      glVertex3f(0.0f, 0.0f, 0.0f);
      glVertex3f(points[farIndex](0), -points[farIndex](1),
                 -points[farIndex](2));
      glEnd();
      break;
    case Camera::ORTHOGRAPHIC:
      if (drawFarPlane) {
        glBegin(GL_LINES);
        glVertex3f(points[0](0), points[0](1), -points[0](2));
        glVertex3f(points[1](0), points[1](1), -points[1](2));
        glVertex3f(-points[0](0), points[0](1), -points[0](2));
        glVertex3f(-points[1](0), points[1](1), -points[1](2));
        glVertex3f(-points[0](0), -points[0](1), -points[0](2));
        glVertex3f(-points[1](0), -points[1](1), -points[1](2));
        glVertex3f(points[0](0), -points[0](1), -points[0](2));
        glVertex3f(points[1](0), -points[1](1), -points[1](2));
        glEnd();
      }
  }

  glPopMatrix();
}

void Camera::get_frustum_planes_coefficients(GLdouble coef[6][4]) const {
  // Computed once and for all
  const Eigen::Vector3d pos = position();
  const Eigen::Vector3d viewDir = view_direction();
  const Eigen::Vector3d up = up_vector();
  const Eigen::Vector3d right = right_vector();
  const double posViewDir = pos.dot(viewDir);

  static Eigen::Vector3d normal[6];
  static GLdouble dist[6];

  switch (type()) {
    case Camera::PERSPECTIVE: {
      const double hhfov = horizontalfield_of_view() / 2.0;
      const double chhfov = cos(hhfov);
      const double shhfov = sin(hhfov);
      normal[0] = -shhfov * viewDir;
      normal[1] = normal[0] + chhfov * right;
      normal[0] = normal[0] - chhfov * right;

      normal[2] = -viewDir;
      normal[3] = viewDir;

      const double hfov = field_of_view() / 2.0;
      const double chfov = cos(hfov);
      const double shfov = sin(hfov);
      normal[4] = -shfov * viewDir;
      normal[5] = normal[4] - chfov * up;
      normal[4] = normal[4] + chfov * up;

      for (int i = 0; i < 2; ++i) {
        dist[i] = pos.dot(normal[i]);
      }
      for (int j = 4; j < 6; ++j) {
        dist[j] = pos.dot(normal[j]);
      }

      // Natural equations are:
      // dist[0, 1,4,5] = pos * normal[0, 1,4,5];
      // dist[2] = (pos + znear() * viewDir) * normal[2];
      // dist[3] = (pos + zfar()    * viewDir) * normal[3];

      // 2 times less computations using expanded/merged equations. Dir vectors
      // are normalized.
      const double posRightCosHH = chhfov * pos.dot(right);
      dist[0] = -shhfov * posViewDir;
      dist[1] = dist[0] + posRightCosHH;
      dist[0] = dist[0] - posRightCosHH;
      const double posUpCosH = chfov * pos.dot(up);
      dist[4] = -shfov * posViewDir;
      dist[5] = dist[4] - posUpCosH;
      dist[4] = dist[4] + posUpCosH;

      break;
    }
    case Camera::ORTHOGRAPHIC:
      normal[0] = -right;
      normal[1] = right;
      normal[4] = up;
      normal[5] = -up;

      GLdouble hw = 0.0;
      GLdouble hh = 0.0;

      get_ortho_width_height(&hw, &hh);
      dist[0] = (pos - hw * right).dot(normal[0]);
      dist[1] = (pos + hw * right).dot(normal[1]);
      dist[4] = (pos + hh * up).dot(normal[4]);
      dist[5] = (pos - hh * up).dot(normal[5]);
      break;
  }

  // Front and far planes are identical for both camera types.
  normal[2] = -viewDir;
  normal[3] = viewDir;
  dist[2] = -posViewDir - znear();
  dist[3] = posViewDir + zfar();

  for (int i = 0; i < 6; ++i) {
    coef[i][0] = GLdouble(normal[i](0));
    coef[i][1] = GLdouble(normal[i](1));
    coef[i][2] = GLdouble(normal[i](2));
    coef[i][3] = dist[i];
  }
}

static double projectOnBall(double x, double y) {
  // If you change the size value, change angle computation in
  // deformed_ball_quaternion().
  const double size = 1.0f;
  const double size2 = size * size;
  const double size_limit = size2 * 0.5;

  const double d = x * x + y * y;
  return d < size_limit ? sqrt(size2 - d) : size_limit / sqrt(d);
}

Eigen::Quaterniond Camera::deformed_ball_quaternion(int preX, int preY, int x,
                                                    int y, double cx,
                                                    double cy) {
  // Points on the deformed ball
  double px = (preX - cx) / screen_width();
  double py = (cy - preY) / screen_height();
  double dx = (x - cx) / screen_width();
  double dy = (cy - y) / screen_height();

  const Eigen::Vector3d p1(px, py, projectOnBall(px, py));
  const Eigen::Vector3d p2(dx, dy, projectOnBall(dx, dy));
  // Approximation of rotation angle
  // Should be divided by the projectOnBall size, but it is 1.0
  Eigen::Vector3d axis = p2.cross(p1);
  const double angle =
      2.0 *
      asin(sqrt(axis.squaredNorm() / p1.squaredNorm() / p2.squaredNorm()));
  axis.normalize();
  Eigen::AngleAxisd angleAxis(angle, axis);
  return Eigen::Quaterniond(angleAxis);
}

}  // namespace lowcostvisualizer
}  // namespace perception
}  // namespace apollo
