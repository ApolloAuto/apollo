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

#ifndef MODULES_PERCEPTION_OBSTACLE_LIDAR_VISUALIZER_GLFW_VIEWER_H_
#define MODULES_PERCEPTION_OBSTACLE_LIDAR_VISUALIZER_GLFW_VIEWER_H_

#include <memory>

#include "Eigen/Dense"
#include "GL/glew.h"
#include "GLFW/glfw3.h"

#include "modules/perception/obstacle/lidar/visualizer/opengl_visualizer/camera.h"
#include "modules/perception/obstacle/lidar/visualizer/opengl_visualizer/frame_content.h"

namespace apollo {
namespace perception {
class GLFWViewer {
 public:
  GLFWViewer();
  virtual ~GLFWViewer();

  bool Initialize();

  void SetFrameContent(const FrameContent &frame_content) {
    frame_content_ = frame_content;
  }
  void Spin();
  void SpinOnce();
  void Close();

  void SetBackgroundColor(Eigen::Vector3d i_bg_color) {
    bg_color_ = i_bg_color;
  }
  void SetSize(int w, int h);
  void SetCameraPara(Eigen::Vector3d i_position, Eigen::Vector3d i_scn_center,
                     Eigen::Vector3d i_up_vector);
  void SetForwardDir(Eigen::Vector3d forward) { forward_dir_ = forward; }

  // callback assistants
  void ResizeFramebuffer(int width, int height);
  void MouseMove(double xpos, double ypos);
  void MouseWheel(double delta);
  void Reset();
  void Keyboard(int key);

  // callback functions
  static void FramebufferSizeCallback(GLFWwindow *window, int width,
                                      int height);
  static void KeyCallback(GLFWwindow *window, int key, int scancode, int action,
                          int mods);
  static void MouseButtonCallback(GLFWwindow *window, int button, int action,
                                  int mods);
  static void MouseCursorPositionCallback(GLFWwindow *window, double xpos,
                                          double ypos);
  static void MouseScrollCallback(GLFWwindow *window, double xoffset,
                                  double yoffset);
  static void ErrorCallback(int error, const char *description);

 private:
  bool WindowInit();
  bool CameraInit();
  bool OpenglInit();
  void PreDraw();
  void Render();

  void GetClassColor(int cls, float rgb[3]);
  void DrawCloud();
  void DrawCircle();
  void DrawCarForwardDir();
  void DrawObstacles();
  void DrawObstacle(const std::shared_ptr<Object> obj, bool show_cloud,
                    bool show_polygon, bool show_velocity, bool show_direction);
  void DrawOffsetVolumn(Eigen::Vector3d *polygon_points, double h,
                        int polygon_size);

 private:
  bool init_;

  GLFWwindow *window_;
  Camera *pers_camera_;

  FrameContent frame_content_;
  Eigen::Vector3d forward_dir_;
  Eigen::Vector3d scn_center_;
  Eigen::Vector3d bg_color_;

  Eigen::Matrix4d mode_mat_;
  Eigen::Matrix4d view_mat_;

  int win_width_;
  int win_height_;
  int mouse_prev_x_;
  int mouse_prev_y_;

  bool show_cloud_;
  int show_cloud_state_;
  bool show_velocity_;
  bool show_direction_;
  bool show_polygon_;

  enum class OBJ_Type {
    CIRCIE = 0,
    CUBE = 1,
    CLOUD = 2,
    POLYGON = 3,
    NUM_VAO_TYPE = 4
  };

  enum class VBO_Type {
    VBO_VERTICES = 0,
    VBO_COLORS = 1,
    VBO_ELEMENTS = 2,
    NUM_VBO_TYPE = 3
  };

  // cloud
  static const int kCloud_VAO_Num_ = 35;
  static const int kPoint_Num_Per_Cloud_VAO_ = 10000;
  GLuint cloud_VAO_buf_ids_[kCloud_VAO_Num_];
  // each VAO has NUM_VBO_TYPE VBOs
  GLuint cloud_VBO_buf_ids_[kCloud_VAO_Num_]
                           [static_cast<int>(VBO_Type::NUM_VBO_TYPE)];
  GLfloat cloud_verts_[kPoint_Num_Per_Cloud_VAO_][3];

  // circle
  static const int kCircle_VAO_Num_ = 3;
  static const int kPoint_Num_Per_Circle_VAO_ = 256;
  GLuint circle_VAO_buf_ids_[kCircle_VAO_Num_];
  GLuint circle_VBO_buf_ids_[kCircle_VAO_Num_]
                            [static_cast<int>(VBO_Type::NUM_VBO_TYPE)];
};

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_LIDAR_VISUALIZER_GLFW_VIEWER_H_
