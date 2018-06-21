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

#include "modules/perception/obstacle/lidar/visualizer/opengl_visualizer/glfw_viewer.h"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <vector>

#include "pcl/io/pcd_io.h"

#include "modules/common/log.h"
#include "modules/perception/obstacle/base/object.h"
#include "modules/perception/obstacle/lidar/visualizer/opengl_visualizer/arc_ball.h"
#include "modules/perception/obstacle/lidar/visualizer/opengl_visualizer/frame_content.h"

namespace apollo {
namespace perception {

#define BUFFER_OFFSET(offset) (reinterpret_cast<GLvoid *>(offset))

GLFWViewer::GLFWViewer() {
  init_ = false;
  window_ = nullptr;
  pers_camera_ = nullptr;
  forward_dir_ = Eigen::Vector3d::Zero();
  scn_center_ = Eigen::Vector3d::Zero();
  bg_color_ = Eigen::Vector3d::Zero();

  mode_mat_ = Eigen::Matrix4d::Identity();
  view_mat_ = Eigen::Matrix4d::Identity();

  win_width_ = 800;
  win_height_ = 600;
  mouse_prev_x_ = 0;
  mouse_prev_y_ = 0;

  show_cloud_ = true;
  show_cloud_state_ = 0;
  show_velocity_ = true;
  show_direction_ = false;
  show_polygon_ = false;
}

GLFWViewer::~GLFWViewer() {
  Close();
  if (pers_camera_) delete pers_camera_;
}

bool GLFWViewer::Initialize() {
  AINFO << "GLFWViewer::initialize()" << std::endl;
  if (init_) {
    AINFO << " GLFWViewer is already initialized !" << std::endl;
    return false;
  }

  if (!WindowInit()) {
    AINFO << " Failed to initialize the window !" << std::endl;
    return false;
  }

  if (!CameraInit()) {
    AINFO << " Failed to initialize the camera !" << std::endl;
    return false;
  }

  if (!OpenglInit()) {
    AINFO << " Failed to initialize opengl !" << std::endl;
    return false;
  }

  init_ = true;

  show_cloud_ = 1;
  show_velocity_ = 1;
  show_polygon_ = 0;
  return true;
}

void GLFWViewer::Spin() {
  while (!glfwWindowShouldClose(window_)) {
    glfwPollEvents();
    Render();
    glfwSwapBuffers(window_);
  }
  glfwDestroyWindow(window_);
}

void GLFWViewer::SpinOnce() {
  glfwPollEvents();
  Render();
  glfwSwapBuffers(window_);
}

void GLFWViewer::Close() { glfwTerminate(); }

void GLFWViewer::SetSize(int w, int h) {
  win_width_ = w;
  win_height_ = h;
}

void GLFWViewer::SetCameraPara(Eigen::Vector3d i_position,
                               Eigen::Vector3d i_scn_center,
                               Eigen::Vector3d i_up_vector) {
  pers_camera_->SetPosition(i_position);
  pers_camera_->LookAt(i_scn_center);
  pers_camera_->SetUpDirection(i_up_vector);
  view_mat_ = pers_camera_->GetViewMat();
  scn_center_ = i_scn_center;
}

bool GLFWViewer::WindowInit() {
  if (!glfwInit()) {
    AERROR << "Failed to initialize glfw !\n";
    return false;
  }

  window_ = glfwCreateWindow(win_width_, win_height_, "opengl_visualizer",
                             nullptr, nullptr);
  if (window_ == nullptr) {
    AERROR << "Failed to create glfw window!\n";
    glfwTerminate();
    return false;
  }

  glfwMakeContextCurrent(window_);
  glfwSwapInterval(1);
  glfwSetWindowUserPointer(window_, this);

  // set callback functions
  glfwSetFramebufferSizeCallback(window_, FramebufferSizeCallback);

  glfwSetKeyCallback(window_, KeyCallback);
  glfwSetMouseButtonCallback(window_, MouseButtonCallback);
  glfwSetCursorPosCallback(window_, MouseCursorPositionCallback);
  glfwSetScrollCallback(window_, MouseScrollCallback);

  glfwShowWindow(window_);
  return true;
}

bool GLFWViewer::CameraInit() {
  // perspective cameras
  pers_camera_ = new Camera;
  pers_camera_->SetScreenWidthHeight(win_width_, win_height_);
  pers_camera_->SetFov(45.0);
  pers_camera_->SetPosition(Eigen::Vector3d(0, 0, -30));

  return true;
}

bool GLFWViewer::OpenglInit() {
  glClearColor(bg_color_(0), bg_color_(1), bg_color_(2), 0.0);
  glClearDepth(1.0f);
  glShadeModel(GL_SMOOTH);
  glDepthFunc(GL_LEQUAL);
  // lighting
  GLfloat mat_shininess[] = {20.0};
  GLfloat light_position[] = {1.0, -1.0, 1.0, 0.0};
  GLfloat lmodel_ambient[] = {.5, .5, .5, 1.0};
  glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);
  glLightfv(GL_LIGHT0, GL_POSITION, light_position);
  glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lmodel_ambient);
  glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
  glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);
  glEnable(GL_LIGHT0);
  glEnable(GL_LIGHTING);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_NORMALIZE);
  glEnable(GL_COLOR_MATERIAL);

  // glew
  if (glewInit() != GLEW_OK) {
    AERROR << "Failed to initialize glew !";
    exit(EXIT_FAILURE);
  }

  // allocation of vbo
  // point cloud
  {
    GLfloat cloud_colors[kPoint_Num_Per_Cloud_VAO_][3];
    GLuint cloud_indices[kPoint_Num_Per_Cloud_VAO_];
    for (int i = 0; i < kPoint_Num_Per_Cloud_VAO_; i++) {
      cloud_colors[i][0] = 0.7;
      cloud_colors[i][1] = 0.7;
      cloud_colors[i][2] = 0.7;
      cloud_indices[i] = GLuint(i);
    }

    glGenVertexArrays(kCloud_VAO_Num_, cloud_VAO_buf_ids_);
    for (int i = 0; i < kCloud_VAO_Num_; i++) {
      glBindVertexArray(cloud_VAO_buf_ids_[i]);

      glGenBuffers(static_cast<int>(VBO_Type::NUM_VBO_TYPE),
                   cloud_VBO_buf_ids_[i]);
      glBindBuffer(
          GL_ARRAY_BUFFER,
          cloud_VBO_buf_ids_[i][static_cast<int>(VBO_Type::VBO_VERTICES)]);
      glBufferData(GL_ARRAY_BUFFER, sizeof(cloud_verts_), cloud_verts_,
                   GL_STREAM_DRAW);
      glVertexPointer(3, GL_FLOAT, 0, BUFFER_OFFSET(0));
      glEnableClientState(GL_VERTEX_ARRAY);

      glBindBuffer(
          GL_ARRAY_BUFFER,
          cloud_VBO_buf_ids_[i][static_cast<int>(VBO_Type::VBO_COLORS)]);
      glBufferData(GL_ARRAY_BUFFER, sizeof(cloud_colors), cloud_colors,
                   GL_STREAM_DRAW);
      glColorPointer(3, GL_FLOAT, 0, BUFFER_OFFSET(0));
      glEnableClientState(GL_COLOR_ARRAY);

      glBindBuffer(
          GL_ELEMENT_ARRAY_BUFFER,
          cloud_VBO_buf_ids_[i][static_cast<int>(VBO_Type::VBO_ELEMENTS)]);
      glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(cloud_indices),
                   cloud_indices, GL_STREAM_DRAW);
    }
  }
  // circle
  {
    GLfloat circle_verts[kPoint_Num_Per_Circle_VAO_][3];
    GLfloat circle_colors[kPoint_Num_Per_Circle_VAO_][3];
    GLuint circle_indices[kPoint_Num_Per_Circle_VAO_];

    for (int i = 0; i < kPoint_Num_Per_Circle_VAO_; ++i) {
      circle_verts[i][2] = -1.0;
      circle_colors[i][0] = 0.0;
      circle_colors[i][1] = 0.0;
      circle_colors[i][2] = 0.9;
      circle_indices[i] = GLuint(i);
    }

    float ang_interv =
        2 * M_PI / static_cast<float>(kPoint_Num_Per_Circle_VAO_);
    glGenVertexArrays(kCircle_VAO_Num_, circle_VAO_buf_ids_);
    for (int vao = 0; vao < kCircle_VAO_Num_; ++vao) {
      for (int i = 0; i < kPoint_Num_Per_Circle_VAO_; ++i) {
        float theta = i * ang_interv;
        circle_verts[i][0] = 20 * (vao + 1) * cos(theta);
        circle_verts[i][1] = 20 * (vao + 1) * sin(theta);
      }

      glBindVertexArray(circle_VAO_buf_ids_[vao]);

      glGenBuffers(static_cast<int>(VBO_Type::NUM_VBO_TYPE),
                   circle_VBO_buf_ids_[vao]);
      glBindBuffer(
          GL_ARRAY_BUFFER,
          circle_VBO_buf_ids_[vao][static_cast<int>(VBO_Type::VBO_VERTICES)]);
      glBufferData(GL_ARRAY_BUFFER, sizeof(circle_verts), circle_verts,
                   GL_STATIC_DRAW);
      glVertexPointer(3, GL_FLOAT, 0, BUFFER_OFFSET(0));
      glEnableClientState(GL_VERTEX_ARRAY);

      glBindBuffer(
          GL_ARRAY_BUFFER,
          circle_VBO_buf_ids_[vao][static_cast<int>(VBO_Type::VBO_COLORS)]);
      glBufferData(GL_ARRAY_BUFFER, sizeof(circle_colors), circle_colors,
                   GL_STATIC_DRAW);
      glColorPointer(3, GL_FLOAT, 0, BUFFER_OFFSET(0));
      glEnableClientState(GL_COLOR_ARRAY);

      glBindBuffer(
          GL_ELEMENT_ARRAY_BUFFER,
          circle_VBO_buf_ids_[vao][static_cast<int>(VBO_Type::VBO_ELEMENTS)]);
      glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(circle_indices),
                   circle_indices, GL_STATIC_DRAW);
    }
  }

  return true;
}

void GLFWViewer::PreDraw() {
  Eigen::Matrix4d e_proj_mat = pers_camera_->GetProjectionMat();

  // column major
  GLdouble proj_mat[16] = {
      e_proj_mat(0, 0), e_proj_mat(1, 0), e_proj_mat(2, 0), e_proj_mat(3, 0),
      e_proj_mat(0, 1), e_proj_mat(1, 1), e_proj_mat(2, 1), e_proj_mat(3, 1),
      e_proj_mat(0, 2), e_proj_mat(1, 2), e_proj_mat(2, 2), e_proj_mat(3, 2),
      e_proj_mat(0, 3), e_proj_mat(1, 3), e_proj_mat(2, 3), e_proj_mat(3, 3)};
  GLdouble mode_mat[16] = {
      mode_mat_(0, 0), mode_mat_(1, 0), mode_mat_(2, 0), mode_mat_(3, 0),
      mode_mat_(0, 1), mode_mat_(1, 1), mode_mat_(2, 1), mode_mat_(3, 1),
      mode_mat_(0, 2), mode_mat_(1, 2), mode_mat_(2, 2), mode_mat_(3, 2),
      mode_mat_(0, 3), mode_mat_(1, 3), mode_mat_(2, 3), mode_mat_(3, 3)};
  GLdouble view_mat[16] = {
      view_mat_(0, 0), view_mat_(1, 0), view_mat_(2, 0), view_mat_(3, 0),
      view_mat_(0, 1), view_mat_(1, 1), view_mat_(2, 1), view_mat_(3, 1),
      view_mat_(0, 2), view_mat_(1, 2), view_mat_(2, 2), view_mat_(3, 2),
      view_mat_(0, 3), view_mat_(1, 3), view_mat_(2, 3), view_mat_(3, 3)};

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glMultMatrixd(proj_mat);
  glMatrixMode(GL_MODELVIEW);
  glLoadMatrixd(mode_mat);
  glMultMatrixd(view_mat);

  GLfloat light_position[] = {1.0, -1.0, 1.0, 0.0};
  glLightfv(GL_LIGHT0, GL_POSITION, light_position);
}

void GLFWViewer::Render() {
  glClear(GL_COLOR_BUFFER_BIT);
  PreDraw();

  if (show_cloud_) DrawCloud();
  DrawObstacles();
  DrawCircle();
  DrawCarForwardDir();
}

void GLFWViewer::DrawCloud() {
  pcl_util::PointCloudPtr cloud;
  pcl_util::PointCloudPtr roi_cloud;

  if (show_cloud_state_ == 0) {  // only show original point cloud
    cloud = frame_content_.GetCloud();
  } else if (show_cloud_state_ == 1) {  // show roi
    roi_cloud = frame_content_.GetRoiCloud();
  } else {  // show both
    cloud = frame_content_.GetCloud();
    roi_cloud = frame_content_.GetRoiCloud();
  }

  // draw original point cloud
  if (cloud && !cloud->points.empty()) {
    glPointSize(1);
    int count = 0;
    int p_num = 0;
    int vao_num = (cloud->points.size() / kPoint_Num_Per_Cloud_VAO_) + 1;
    for (int vao = 0; vao < vao_num; vao++) {
      for (p_num = 0; p_num < kPoint_Num_Per_Cloud_VAO_; ++p_num) {
        cloud_verts_[p_num][0] = cloud->points[count].x;
        cloud_verts_[p_num][1] = cloud->points[count].y;
        cloud_verts_[p_num][2] = cloud->points[count].z;
        count++;

        if (count >= static_cast<int>(cloud->points.size())) {
          break;
        }
      }

      glBindVertexArray(cloud_VAO_buf_ids_[vao]);
      glBindBuffer(
          GL_ARRAY_BUFFER,
          cloud_VBO_buf_ids_[vao][static_cast<int>(VBO_Type::VBO_VERTICES)]);
      glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(cloud_verts_), cloud_verts_);
      glDrawElements(GL_POINTS, p_num, GL_UNSIGNED_INT, BUFFER_OFFSET(0));
      glBindVertexArray(0);

      if (count >= static_cast<int>(cloud->points.size())) {
        break;
      }
    }

    if (count < static_cast<int>(cloud->points.size())) {
      AINFO << "VAO_num * VBO_num < cloud->points.size()";
    }
  }

  // draw roi point cloud
  if (roi_cloud && !roi_cloud->points.empty()) {
    glPointSize(3);
    glColor3f(0, 0.8, 0);
    glBegin(GL_POINTS);
    for (const auto &point : roi_cloud->points) {
      glVertex3f(point.x, point.y, point.z);
    }
    glEnd();
  }
}

void GLFWViewer::DrawCircle() {
  Eigen::Matrix4d v2w_pose = frame_content_.GetPoseV2w();
  GLdouble mat[16] = {
      v2w_pose(0, 0), v2w_pose(1, 0), v2w_pose(2, 0), v2w_pose(3, 0),
      v2w_pose(0, 1), v2w_pose(1, 1), v2w_pose(2, 1), v2w_pose(3, 1),
      v2w_pose(0, 2), v2w_pose(1, 2), v2w_pose(2, 2), v2w_pose(3, 2),
      v2w_pose(0, 3), v2w_pose(1, 3), v2w_pose(2, 3), v2w_pose(3, 3)};

  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glMultMatrixd(mat);
  int vao = 0;
  for (vao = 0; vao < kCircle_VAO_Num_; vao++) {
    glBindVertexArray(circle_VAO_buf_ids_[vao]);
    glDrawElements(GL_LINE_LOOP, kPoint_Num_Per_Circle_VAO_, GL_UNSIGNED_INT,
                   BUFFER_OFFSET(0));
    glBindVertexArray(0);
  }
  glPopMatrix();
}

void GLFWViewer::DrawCarForwardDir() {
  glColor3f(1.0, 0.5, 0.17);
  glLineWidth(5);
  glBegin(GL_LINES);
  Eigen::Vector3d forward_vp = scn_center_ + forward_dir_ * 10;
  glVertex3f(scn_center_(0), scn_center_(1), scn_center_(2));
  glVertex3f(forward_vp(0), forward_vp(1), forward_vp(2));
  glEnd();
  glLineWidth(1);
}

void GLFWViewer::DrawObstacle(const std::shared_ptr<Object> obj,
                              bool show_cloud, bool show_polygon,
                              bool show_velocity, bool show_direction) {
  float type_color[3] = {0, 0, 0};
  // TODO(All): modify GetClassColor params
  GetClassColor(static_cast<int>(obj->type), type_color);
  if (show_polygon) {
    double h = obj->height;
    glColor3f(type_color[0], type_color[1], type_color[2]);
    glBegin(GL_LINE_LOOP);
    for (auto point : obj->polygon.points) {
      glVertex3f(point.x, point.y, point.z);
    }
    glEnd();

    glBegin(GL_LINE_LOOP);
    for (auto point : obj->polygon.points) {
      glVertex3f(point.x, point.y, point.z + h);
    }
    glEnd();

    glBegin(GL_LINES);
    for (auto point : obj->polygon.points) {
      glVertex3f(point.x, point.y, point.z);
      glVertex3f(point.x, point.y, point.z + h);
    }
    glEnd();
  } else {
    glColor3f(type_color[0], type_color[1], type_color[2]);
    Eigen::Vector3d dir(cos(obj->theta), sin(obj->theta), 0);
    Eigen::Vector3d odir(-dir[1], dir[0], 0);
    Eigen::Vector3d bottom_quad[4];
    double half_l = obj->length / 2;
    double half_w = obj->width / 2;
    double h = obj->height;
    bottom_quad[0] = obj->center - dir * half_l - odir * half_w;
    bottom_quad[1] = obj->center + dir * half_l - odir * half_w;
    bottom_quad[2] = obj->center + dir * half_l + odir * half_w;
    bottom_quad[3] = obj->center - dir * half_l + odir * half_w;

    DrawOffsetVolumn(bottom_quad, h, 4);
  }

  if (show_velocity) {
    glColor3f(1, 0, 0);
    const Eigen::Vector3d &center = obj->center;
    const Eigen::Vector3d &velocity = obj->velocity;
    Eigen::Vector3d dir(cos(obj->theta), sin(obj->theta), 0);
    Eigen::Vector3d start_point;
    if (dir.dot(velocity) < 0) {
      start_point = center - dir * (obj->length / 2);
    } else {
      start_point = center + dir * (obj->length / 2);
    }
    Eigen::Vector3d end_point = start_point + velocity;
    glBegin(GL_LINES);
    glVertex3f(start_point[0], start_point[1], start_point[2]);
    glVertex3f(end_point[0], end_point[1], end_point[2]);
    glEnd();
  }

  if (show_direction) {
    glColor3f(0, 0, 1);
    const Eigen::Vector3d &center = obj->center;
    Eigen::Vector3d dir(cos(obj->theta), sin(obj->theta), 0);
    Eigen::Vector3d odir(-dir[1], dir[0], 0);
    Eigen::Vector3d start_point =
        center + dir * (obj->length / 2) + odir * (obj->width / 2);
    Eigen::Vector3d end_point = start_point + dir * 3;
    glBegin(GL_LINES);
    glVertex3f(start_point[0], start_point[1], start_point[2]);
    glVertex3f(end_point[0], end_point[1], end_point[2]);
    glEnd();
  }
}

void GLFWViewer::DrawObstacles() {
  std::vector<std::shared_ptr<Object>> tracked_objects =
      frame_content_.GetTrackedObjects();
  for (std::size_t i = 0; i < tracked_objects.size(); i++) {
    DrawObstacle(tracked_objects[i], true, show_polygon_, show_velocity_,
                 show_direction_);
  }
}

void GLFWViewer::DrawOffsetVolumn(Eigen::Vector3d *polygon_points, double h,
                                  int polygon_size) {
  if (polygon_points == nullptr) {
    return;
  }

  glBegin(GL_LINE_LOOP);
  for (int i = 0; i < polygon_size; i++) {
    glVertex3d(polygon_points[i][0], polygon_points[i][1],
               polygon_points[i][2]);
  }
  glEnd();

  glBegin(GL_LINE_LOOP);
  for (int i = 0; i < polygon_size; i++) {
    glVertex3d(polygon_points[i][0], polygon_points[i][1],
               polygon_points[i][2] + h);
  }
  glEnd();

  glBegin(GL_LINES);
  for (int i = 0; i < polygon_size; i++) {
    glVertex3d(polygon_points[i][0], polygon_points[i][1],
               polygon_points[i][2]);
    glVertex3d(polygon_points[i][0], polygon_points[i][1],
               polygon_points[i][2] + h);
  }
  glEnd();
}

void GLFWViewer::GetClassColor(int cls, float rgb[3]) {
  switch (cls) {
    case 0:
      rgb[0] = 0.5;
      rgb[1] = 0;
      rgb[2] = 1;  // purple
      break;
    case 1:
      rgb[0] = 0;
      rgb[1] = 1;
      rgb[2] = 1;  // cryan
      break;
    case 2:
      rgb[0] = 1;
      rgb[1] = 1;
      rgb[2] = 0;  // yellow
      break;
    case 3:
      rgb[0] = 1;
      rgb[1] = 0.5;
      rgb[2] = 0.5;  // red
      break;
    case 4:
      rgb[0] = 0;
      rgb[1] = 0;
      rgb[2] = 1;  // blue
      break;
    case 5:
      rgb[0] = 0;
      rgb[1] = 1;
      rgb[2] = 0;  // green
      break;
    case 6:
      rgb[0] = 1;
      rgb[1] = 0.5;
      rgb[2] = 0;  // orange
      break;
    case 7:
      rgb[0] = 1;
      rgb[1] = 0;
      rgb[2] = 0;  // red
      break;
    default:
      rgb[0] = 1;
      rgb[1] = 1;
      rgb[2] = 1;  // white
      break;
  }
}

/************************callback functions************************/

void GLFWViewer::FramebufferSizeCallback(GLFWwindow *window, int width,
                                         int height) {
  void *user_data = glfwGetWindowUserPointer(window);
  if (user_data == nullptr) return;

  GLFWViewer *vis = static_cast<GLFWViewer *>(user_data);
  vis->ResizeFramebuffer(width, height);
}

void GLFWViewer::KeyCallback(GLFWwindow *window, int key, int scancode,
                             int action, int mods) {
  void *user_data = glfwGetWindowUserPointer(window);
  if (user_data == nullptr) return;
  if (action == GLFW_PRESS) {
    GLFWViewer *vis = static_cast<GLFWViewer *>(user_data);
    AINFO << "key_value: " << key;
    vis->Keyboard(key);
  }
}

void GLFWViewer::MouseButtonCallback(GLFWwindow *window, int button, int action,
                                     int mods) {}

void GLFWViewer::MouseCursorPositionCallback(GLFWwindow *window, double xpos,
                                             double ypos) {
  void *user_data = glfwGetWindowUserPointer(window);
  if (user_data == nullptr) return;

  GLFWViewer *vis = static_cast<GLFWViewer *>(user_data);
  vis->MouseMove(xpos, ypos);
}

void GLFWViewer::MouseScrollCallback(GLFWwindow *window, double xoffset,
                                     double yoffset) {
  void *user_data = glfwGetWindowUserPointer(window);
  if (user_data == nullptr) return;

  GLFWViewer *vis = static_cast<GLFWViewer *>(user_data);
  vis->MouseWheel(yoffset);
}

void GLFWViewer::ErrorCallback(int error, const char *description) {
  AINFO << "ERROR - " << error << "  " << description;
}

/************************callback assistants************************/
/*void GLFWViewer::resize_window(int width, int height){
}*/

void GLFWViewer::ResizeFramebuffer(int width, int height) {
  glViewport(0, 0, width, height);
  win_width_ = width;
  win_height_ = height;
}

void GLFWViewer::MouseMove(double xpos, double ypos) {
  int state_left = glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_LEFT);
  int state_right = glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_RIGHT);
  int x_delta = xpos - mouse_prev_x_;
  int y_delta = ypos - mouse_prev_y_;
  if (state_left == GLFW_PRESS) {
    Eigen::Vector3d obj_cen_screen = pers_camera_->PointOnScreen(scn_center_);
    Eigen::Quaterniond rot = ArcBall::RotateByMouse(
        static_cast<double>(mouse_prev_x_), static_cast<double>(mouse_prev_y_),
        xpos, ypos, obj_cen_screen(0), obj_cen_screen(1),
        static_cast<double>(win_width_), static_cast<double>(win_height_));

    Eigen::Matrix3d rot_mat = rot.inverse().toRotationMatrix();
    Eigen::Vector3d scn_center = scn_center_;
    Eigen::Vector4d scn_center_tmp(scn_center(0), scn_center(1), scn_center(2),
                                   1);
    scn_center_tmp = mode_mat_ * view_mat_ * scn_center_tmp;
    scn_center = scn_center_tmp.head(3);
    Eigen::Vector3d r_multi_scn_center = rot_mat * scn_center;
    Eigen::Vector3d t = scn_center - r_multi_scn_center;
    Eigen::Matrix4d cur_mat = Eigen::Matrix4d::Identity();
    cur_mat.topLeftCorner(3, 3) = rot_mat;
    cur_mat.topRightCorner(3, 1) = t;
    mode_mat_ = cur_mat * mode_mat_;
  } else if (state_right == GLFW_PRESS) {
    mode_mat_(0, 3) += 0.1 * x_delta;
    mode_mat_(1, 3) -= 0.1 * y_delta;
  }
  mouse_prev_x_ = xpos;
  mouse_prev_y_ = ypos;
}

void GLFWViewer::MouseWheel(double delta) { mode_mat_(2, 3) -= delta; }

void GLFWViewer::Reset() { mode_mat_ = Eigen::Matrix4d::Identity(); }

void GLFWViewer::Keyboard(int key) {
  switch (key) {
    case GLFW_KEY_R:  // 'R'
      Reset();
      break;
    case GLFW_KEY_P:  // 'P'
      show_polygon_ = !show_polygon_;
      break;
    case GLFW_KEY_V:  // 'V'
      show_velocity_ = !show_velocity_;
      break;
    case GLFW_KEY_D:  // d
      show_direction_ = !show_direction_;
      break;
    case GLFW_KEY_S:  // 'S'
      show_cloud_state_ = (show_cloud_state_ + 1) % 3;
      break;
  }
}

}  // namespace perception
}  // namespace apollo
