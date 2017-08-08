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

#ifndef ADU_PERCEPTION_OBSTACLE_VISUALIZER_GLFW_VIEWER_H
#define ADU_PERCEPTION_OBSTACLE_VISUALIZER_GLFW_VIEWER_H

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "Eigen/Dense"
#include "camera.h"

namespace apollo {
namespace perception {
// 使用此宏来简化偏移量的表达形式
#define BUFFER_OFFSET(offset) ((GLvoid*)offset)

typedef struct {
  GLfloat x;
  GLfloat y;
  GLfloat z;
} vec3;

class FrameContent;

class GLFWViewer {
 public:
  explicit GLFWViewer();
  virtual ~GLFWViewer();

  bool initialize();

  void set_frame_content(FrameContent* frame_content) {
    _frame_content = frame_content;
  }
  void spin();
  void spin_once();
  void close();

  void set_background_color(Eigen::Vector3d i_bg_color) {
    _bg_color = i_bg_color;
  }
  void set_size(int w, int h);
  void set_camera_para(Eigen::Vector3d i_position, Eigen::Vector3d i_scn_center,
                       Eigen::Vector3d i_up_vector);
  void set_forward_dir(Eigen::Vector3d forward) {
    _forward_dir = forward;
  }

  // callback assistants
  void resize_framebuffer(int width, int height);
  void mouse_move(double xpos, double ypos);
  void mouse_wheel(double delta);
  void reset();
  void keyboard(int key);

  // callback functions
  static void framebuffer_size_callback(GLFWwindow* window, int width,
                                        int height);
  // input related
  static void key_callback(GLFWwindow* window, int key, int scancode,
                           int action, int mods);
  static void mouse_button_callback(GLFWwindow* window, int button, int action,
                                    int mods);
  static void mouse_cursor_position_callback(GLFWwindow* window, double xpos,
                                             double ypos);
  static void mouse_scroll_callback(GLFWwindow* window, double xoffset,
                                    double yoffset);
  // error handling
  static void error_callback(int error, const char* description);

 private:
  bool window_init();
  bool camera_init();
  bool opengl_init();
  void pre_draw();
  void render();

 private:
  bool _init;

  GLFWwindow* _window;
  Camera* _pers_camera;

  Eigen::Vector3d _forward_dir;

  Eigen::Vector3d _bg_color;
  int _win_width;
  int _win_height;
  int _mouse_prev_x;
  int _mouse_prev_y;
  Eigen::Matrix4d _mode_mat;
  Eigen::Matrix4d _view_mat;

  FrameContent* _frame_content;

  /***************************************************************************************/
  bool show_cloud;
  int _show_cloud_state;
  bool show_box;
  bool show_velocity;
  bool show_polygon;
  bool show_text;

  /***************************************************************************************/
  void get_class_color(int cls, float rgb[3]);
  enum { circle, cube, cloud, polygon, NumVAOs_typs };  //{0, 1, 2, 3, 4}
  enum { vertices, colors, elements, NumVBOs };         //{0, 1, 2, 3}

  // cloud
  static const int VAO_cloud_num = 35;
  static const int VBO_cloud_num = 10000;
  GLuint VAO_cloud[VAO_cloud_num];
  GLuint buffers_cloud[VAO_cloud_num][NumVBOs];
  GLfloat cloudVerts[VBO_cloud_num][3];
  bool draw_cloud(FrameContent* content);
  // circle
  static const int VAO_circle_num = 6;
  static const int VBO_circle_num = 256;
  GLuint VAO_circle[VAO_circle_num];
  bool draw_circle();
  bool draw_car_forward_dir();
  // objects
  bool draw_objects(FrameContent* content, bool draw_cube, bool draw_polygon,
                    bool draw_velocity);
  vec3 get_velocity_src_position(FrameContent* content, int id);
  // map_roi
  // bool show_map(FrameContent* content, bool show_map_roi, bool
  // show_map_boundary);
};

}  // namespace obstacle
}  // namespace perception
#endif
