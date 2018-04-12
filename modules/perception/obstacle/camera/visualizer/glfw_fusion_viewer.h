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

#ifndef MODULES_PERCEPTION_OBSTACLE_CAMERA_VISUALIZER_GLFW_FUSION_VIEWER_H_
#define MODULES_PERCEPTION_OBSTACLE_CAMERA_VISUALIZER_GLFW_FUSION_VIEWER_H_

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "Eigen/Dense"
#include "GL/glew.h"
#include "GLFW/glfw3.h"
#include "opencv2/opencv.hpp"

#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/obstacle/base/object.h"
#include "modules/perception/obstacle/camera/common/camera.h"
#include "modules/perception/obstacle/camera/visualizer/base_visualizer.h"
#include "modules/perception/obstacle/camera/visualizer/common/camera.h"
#include "modules/perception/obstacle/camera/visualizer/common/gl_raster_text.h"
#include "modules/perception/obstacle/camera/visualizer/frame_content.h"

namespace apollo {
namespace perception {
namespace lowcostvisualizer {

#define BUFFER_OFFSET(offset) (static_cast<GLvoid *>(offset))

typedef struct {
  GLfloat x;
  GLfloat y;
  GLfloat z;
} vec3;

template <typename T = float>
T get_poly_value(T a, T b, T c, T d, T x) {
  T y = d;
  T v = x;
  y += (c * v);
  v *= x;
  y += (b * v);
  v *= x;
  y += (a * v);
  return y;
}

class GLFWFusionViewer {
 public:
  GLFWFusionViewer();

  virtual ~GLFWFusionViewer();

  bool initialize();

  void set_frame_content(FrameContent *frame_content) {
    frame_content_ = frame_content;
  }

  void spin();

  void spin_once();

  void close();

  void set_background_color(Eigen::Vector3d i_bg_color) {
    bg_color_ = i_bg_color;
  }

  void set_camera_para(Eigen::Vector3d i_position, Eigen::Vector3d i_scn_center,
                       Eigen::Vector3d i_up_vector);

  void set_forward_dir(Eigen::Vector3d forward) { forward_dir_ = forward; }

  void set_main_car(const std::vector<Eigen::Vector3d> &main_car) {
    main_car_ = main_car;
  }

  // callback assistants
  void resize_framebuffer(int width, int height);

  void mouse_move(double xpos, double ypos);

  void mouse_wheel(double delta);

  void reset();

  void keyboard(int key);

  void resize_window(int width, int height);

  // callback functions
  static void framebuffer_size_callback(GLFWwindow *window, int width,
                                        int height);

  static void window_size_callback(GLFWwindow *window, int width, int height);

  // input related
  static void key_callback(GLFWwindow *window, int key, int scancode,
                           int action, int mods);

  static void mouse_button_callback(GLFWwindow *window, int button, int action,
                                    int mods);

  static void mouse_cursor_position_callback(GLFWwindow *window, double xpos,
                                             double ypos);

  static void mouse_scroll_callback(GLFWwindow *window, double xoffset,
                                    double yoffset);

  // error handling
  static void error_callback(int error, const char *description);

 private:
  bool window_init();

  bool camera_init();

  bool opengl_init();

  void pre_draw();

  void render();

 protected:
  vec3 get_velocity_src_position(const std::shared_ptr<Object> &object);

  // capture screen
  void capture_screen(const std::string &file_name);

  void draw_car_trajectory(FrameContent *content);
  void draw_trajectories(FrameContent *content);

  void drawHollowCircle(GLfloat x, GLfloat y, GLfloat radius);

  // for drawing camera 2d results
 protected:
  // @brief Get camera intrinsics with distortion coefficients from file
  bool get_camera_distort_intrinsics(const std::string &file_name,
                                     CameraDistort<double> *camera_distort);

  // @brief Project 3D point to 2D image using pin-hole camera model with
  // distortion
  bool project_point_undistort(Eigen::Matrix4d w2c, Eigen::Vector3d pc,
                               Eigen::Vector2d *p2d);

  void get_8points(float width, float height, float length,
                   std::vector<Eigen::Vector3d> *point);

  bool get_boundingbox(Eigen::Vector3d center, Eigen::Matrix4d w2c, float width,
                       float height, float length, Eigen::Vector3d dir,
                       float theta, std::vector<Eigen::Vector2d> *points);

  bool get_project_point(Eigen::Matrix4d w2c, Eigen::Vector3d pc,
                         Eigen::Vector2d *p2d);

  void draw_line2d(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2,
                   int line_width, int r, int g, int b, int offset_x,
                   int offset_y, int raw_image_width, int raw_image_height);

  void draw_camera_box2d(const std::vector<std::shared_ptr<Object>> &objects,
                         Eigen::Matrix4d w2c, int offset_x, int offset_y,
                         int image_width, int image_height);

  void draw_camera_box3d(
      const std::vector<std::shared_ptr<Object>> &camera_objects,
      const std::vector<std::shared_ptr<Object>> &segmented_objects,
      Eigen::Matrix4d w2c, int offset_x, int offset_y, int image_width,
      int image_height);

  void draw_rect2d(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2,
                   int line_width, int r, int g, int b, int offset_x,
                   int offset_y, int image_width, int image_height);

  void draw_8pts_box(const std::vector<Eigen::Vector2d> &points,
                     const Eigen::Vector3f &color, int offset_x, int offset_y,
                     int image_width, int image_height);

  bool draw_car_forward_dir();
  void draw_objects(const std::vector<std::shared_ptr<Object>> &objects,
                    const Eigen::Matrix4d &w2c, bool draw_cube,
                    bool draw_velocity, const Eigen::Vector3f &color,
                    bool use_class_color, bool use_track_color = true);

  void draw_3d_classifications(FrameContent *content, bool show_fusion);
  void draw_camera_box(const std::vector<std::shared_ptr<Object>> &objects,
                       Eigen::Matrix4d w2c, int offset_x, int offset_y,
                       int image_width, int image_height);

  void draw_objects2d(const std::vector<std::shared_ptr<Object>> &objects,
                      Eigen::Matrix4d w2c, std::string name, int offset_x,
                      int offset_y, int image_width, int image_height);

 private:
  bool init_;

  GLFWwindow *window_;
  Camera *pers_camera_;
  Eigen::Vector3d forward_dir_;
  std::vector<Eigen::Vector3d> main_car_;

  Eigen::Vector3d bg_color_;
  int win_width_;
  int win_height_;
  int mouse_prev_x_;
  int mouse_prev_y_;
  Eigen::Matrix4d mode_mat_;
  Eigen::Matrix4d view_mat_;

  FrameContent *frame_content_;
  unsigned char *rgba_buffer_;

  double vao_trans_x_;
  double vao_trans_y_;
  double vao_trans_z_;
  double _Rotate_x;
  double _Rotate_y;
  double _Rotate_z;
  bool show_box;
  bool show_velocity;
  bool show_polygon;
  bool show_text;

  void get_class_color(int cls, float rgb[3]);

  enum { circle, cube, cloud, polygon, NumVAOs_typs };  // {0, 1, 2, 3, 4}
  enum { vertices, colors, elements, NumVBOs };         // {0, 1, 2, 3}

  // cloud
  static const int VAO_cloud_num = 35;
  static const int VBO_cloud_num = 10000;
  GLuint VAO_cloud[VAO_cloud_num];
  GLuint buffers_cloud[VAO_cloud_num][NumVBOs];
  GLfloat cloudVerts[VBO_cloud_num][3];

  bool draw_cloud(FrameContent *content);

  // circle
  static const int VAO_circle_num = 4;
  static const int VBO_circle_num = 360;
  GLuint VAO_circle[VAO_circle_num];
  vec3 get_velocity_src_position(FrameContent *content, int id);

  // fusion association
  void draw_fusion_association(FrameContent *content);

  GLuint image_to_gl_texture(const cv::Mat &mat, GLenum min_filter,
                             GLenum mag_filter, GLenum wrap_filter);

  void draw_camera_frame(FrameContent *content);

  // @brief, draw 2d camera frame, show 2d or 3d classification
  void draw_camera_frame(FrameContent *content, bool show_3d_class);

  // @brief: draw lane objects in ego-car ground (vehicle) space
  void draw_lane_objects_ground();

  // @brief: draw lane objects in image space
  bool draw_lane_objects_image();

  bool use_class_color_ = true;

  bool capture_screen_ = false;
  bool capture_video_ = false;

  int scene_width_;
  int scene_height_;
  int image_width_;
  int image_height_;

  Eigen::Matrix<double, 3, 4> camera_intrinsic_;  // camera intrinsic

  bool show_fusion_;
  bool show_radar_pc_;
  bool show_camera_box2d_;  // show 2d bbox in camera frame
  bool show_camera_box3d_;  // show 3d bbox in camera frame
  bool show_camera_bdv_;
  bool show_associate_color_;  // show same color for both 3d pc bbox and camera
                               // bbox
  bool show_type_id_label_;
  bool show_lane_;
  bool draw_lane_objects_;

  static std::vector<std::vector<int>> s_color_table;
  std::shared_ptr<GLRasterText> raster_text_;

  LaneObjectsPtr lane_objects_;
  float lane_map_threshold_;

  LaneObjectsPtr lane_history_;
  //  std::vector<LaneObjects> Lane_history_buffer_;
  const std::size_t lane_history_buffer_size_ = 400;
  const std::size_t object_history_size_ = 5;
  Eigen::Matrix3f motion_matrix_;
  // pin-hole camera model with distortion
  std::shared_ptr<CameraDistort<double>> distort_camera_intrinsic_;

  // frame count
  int frame_count_;
  // object_trajectories
  std::map<int, std::vector<std::pair<float, float>>> object_trackjectories_;
  std::map<int, std::vector<double>> object_timestamps_;
};

}  // namespace lowcostvisualizer
}  // namespace perception
}  // namespace apollo

#endif
