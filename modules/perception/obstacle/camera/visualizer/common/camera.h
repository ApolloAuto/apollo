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

#ifndef MODULES_PERCEPTION_OBSTACLE_VISUALIZER_CAMERA_H
#define MODULES_PERCEPTION_OBSTACLE_VISUALIZER_CAMERA_H

#include "GL/glu.h"
#include "modules/perception/obstacle/camera/visualizer/common/frame.h"

namespace apollo {
namespace perception {

class Camera {
 public:
  Camera();
  virtual ~Camera();

  Camera(const Camera& camera);
  Camera& operator=(const Camera& camera);

  enum Type { PERSPECTIVE, ORTHOGRAPHIC };

 public:
  Eigen::Vector3d position() const {
    return frame()->position();
  }
  Eigen::Vector3d up_vector() const {
    return frame()->inverse_transform_of(Eigen::Vector3d(0.0, 1.0, 0.0));
  }
  Eigen::Vector3d view_direction() const {
    return frame()->inverse_transform_of(Eigen::Vector3d(0.0, 0.0, -1.0));
  }
  Eigen::Vector3d right_vector() const {
    return frame()->inverse_transform_of(Eigen::Vector3d(1.0, 0.0, 0.0));
  }
  Eigen::Quaterniond orientation() const {
    return frame()->orientation();
  }

  void set_from_model_view_matrix(const double* const modelViewMatrix);
  void set_from_projection_matrix(const double matrix[12]);

  void set_position(const Eigen::Vector3d& pos) {
    frame()->set_position(pos);
  }
  void set_orientation(const Eigen::Quaterniond& q);
  void set_orientation(double theta, double phi);
  void setup_vector(const Eigen::Vector3d& up, bool noMove = true);
  void setview_direction(const Eigen::Vector3d& direction);

  void look_at(const Eigen::Vector3d& target);  // look from eye to target
  void show_entire_scene();
  void fit_sphere(const Eigen::Vector3d& center, double radius);
  void fit_bounding_box(const Eigen::Vector3d& min, const Eigen::Vector3d& max);

  void center_scene();
  void interpolate_to_zoom_on_pixel(const Eigen::Vector2i& pixel);
  void interpolate_to_fit_scene();
  void interpolate_to(const Frame& fr, double duration);

  void pan_by_mouse(int deltaX, int deltaY);
  void rotate_by_mouse(int deltaX, int deltaY);
  void rotate_by_mouse_from_qgwidget(int preX, int preY, int x, int y);
  Eigen::Quaterniond get_rotatation_by_mouse_from_qgwidget(int preX, int preY,
                                                           int x, int y);
  Eigen::Quaterniond deformed_ball_quaternion(int preX, int preY, int x, int y,
                                              double cx, double cy);
  void rotate(Eigen::Vector3d i_axis, double i_angle);

  Type type() const {
    return _type;
  }
  double field_of_view() const {
    return _field_of_view;
  }
  double horizontalfield_of_view() const {
    return 2.0 * atan(tan(field_of_view() / 2.0) * aspect_ratio());
  }
  double aspect_ratio() const {
    return static_cast<double>(_screen_width) /
           static_cast<double>(_screen_height);
  }
  int screen_width() const {
    return _screen_width;
  }
  int screen_height() const {
    return _screen_height;
  }
  void get_viewport(GLint viewport[4]) const;
  double pixelgl_ratio(const Eigen::Vector3d& position) const;
  double znear_coefficient() const {
    return _znear_coef;
  }
  double zclipping_coefficient() const {
    return _zclipping_coef;
  }

  virtual double znear() const;
  virtual double zfar() const;
  virtual void get_ortho_width_height(GLdouble* halfWidth,
                                      GLdouble* halfHeight) const;
  void get_frustum_planes_coefficients(GLdouble coef[6][4]) const;

  void set_type(Type type);
  void setfield_of_view(double fov) {
    _field_of_view = fov;
    setfocus_distance(scene_radius() / tan(fov / 2.0));
  }
  void set_hrizontalfield_of_view(double hfov) {
    setfield_of_view(2.0 * atan(tan(hfov / 2.0) / aspect_ratio()));
  }
  void setfov_to_fit_scene();
  void setaspect_ratio(double aspect) {
    setscreen_widthandheight(static_cast<int>(100.0 * aspect), 100);
  }
  void setscreen_widthandheight(int width, int height);
  void setznear_coefficient(double coef) {
    _znear_coef = coef;
  }
  void setzclipping_coefficient(double coef) {
    _zclipping_coef = coef;
  }

  double scene_radius() const {
    return _scene_radius;
  }
  Eigen::Vector3d scene_center() const {
    return _scene_center;
  }
  double distance_to_scene_center() const;

  void setscene_radius(double radius);
  void setscene_center(const Eigen::Vector3d& center);
  bool setscene_center_from_pixel(const Eigen::Vector2i& pixel);
  void set_scene_bounding_box(const Eigen::Vector3d& min,
                              const Eigen::Vector3d& max);
  void set_revolve_around_point(const Eigen::Vector3d& rap);
  bool set_revolve_around_point_from_pixel(const Eigen::Vector2i& pixel);

  Eigen::Vector3d revolve_around_point() const {
    return _revolve_around_point;
  }

  Frame* frame() const {
    return _frame;
  }
  void set_frame(Frame* const mcf);

  virtual void load_projection_matrix(bool reset = true) const;
  virtual void load_model_view_matrix(bool reset = true) const;
  void compute_projection_matrix() const;
  void compute_model_view_matrix() const;

  virtual void load_projection_matrix_stereo(bool leftBuffer = true) const;
  virtual void load_model_view_matrix_stereo(bool leftBuffer = true) const;

  void get_projection_matrix(double m[16]) const;
  void get_model_view_matrix(double m[16]) const;
  void get_model_view_projection_matrix(double m[16]) const;

  virtual void draw(bool drawFarPlane = true, double scale = 1.0) const;

  Eigen::Vector3d cameracoordinates_of(const Eigen::Vector3d& src) const {
    return frame()->coordinates_of(src);
  }
  Eigen::Vector3d worldcoordinates_of(const Eigen::Vector3d& src) const {
    return frame()->inverse_coordinates_of(src);
  }
  void get_cameracoordinates_of(const double src[3], double res[3]) const;
  void get_worldcoordinates_of(const double src[3], double res[3]) const;

  Eigen::Vector3d projectedcoordinates_of(const Eigen::Vector3d& src,
                                          const Frame* frame = NULL) const;
  Eigen::Vector3d unprojectedcoordinates_of(const Eigen::Vector3d& src,
                                            const Frame* frame = NULL) const;
  void get_projectedcoordinates_of(const double src[3], double res[3],
                                   const Frame* frame = NULL) const;
  void get_unprojectedcoordinates_of(const double src[3], double res[3],
                                     const Frame* frame = NULL) const;
  Eigen::Vector3d point_under_pixel(const Eigen::Vector2i& pixel,
                                    bool* found) const;

  double io_distance() const {
    return _io_distance;
  }
  double physical_distance_to_screen() const {
    return _physical_distance_to_screen;
  }
  double physicalscreen_width() const {
    return _physicalscreen_width;
  }
  double focus_distance() const {
    return _focus_distance;
  }
  void setio_distance(double distance) {
    _io_distance = distance;
  }
  void setphysical_distance_to_screen(double distance) {
    _physical_distance_to_screen = distance;
  }
  void set_physicalscreen_width(double width) {
    _physicalscreen_width = width;
  }
  void setfocus_distance(double distance) {
    _focus_distance = distance;
  }

 private:
  // Frame
  Frame* _frame;

  // Camera parameters
  int _screen_width;
  int _screen_height;     // size of the window, in pixels
  double _field_of_view;  // in radians
  Eigen::Vector3d _scene_center;
  Eigen::Vector3d _revolve_around_point;
  double _scene_radius;  // OpenGL units
  double _znear_coef;
  double _zclipping_coef;
  double _ortho_coef;
  Type _type;                               // PERSPECTIVE or ORTHOGRAPHIC
  mutable GLdouble _model_view_matrix[16];  // Buffered model view matrix.
  mutable GLdouble _projection_matrix[16];  // Buffered projection matrix.

  // Stereo parameters
  double _io_distance;                  // inter-ocular distance, in meters
  double _focus_distance;               // in scene units
  double _physical_distance_to_screen;  // in meters
  double _physicalscreen_width;         // in meters
};

}  // namespace perception
}  // namespace apollo

#endif  // QGLVIEWER_CAMERA_H
