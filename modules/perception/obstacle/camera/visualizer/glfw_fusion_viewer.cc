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

#include "modules/perception/obstacle/camera/visualizer/glfw_fusion_viewer.h"

#include <yaml-cpp/yaml.h>
#include <boost/shared_ptr.hpp>

#include <cfloat>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "gflags/gflags.h"
#include "modules/perception/lib/config_manager/calibration_config_manager.h"
#include "modules/perception/obstacle/camera/visualizer/common/bmp.h"
#include "modules/perception/obstacle/camera/visualizer/common/gl_raster_text.h"
#include "modules/perception/obstacle/camera/visualizer/frame_content.h"

namespace apollo {
namespace perception {
namespace lowcostvisualizer {

using apollo::perception::CalibrationConfigManager;
using apollo::perception::CameraCalibrationPtr;

const double pace_zoom = 15;
const double My_PI = 3.14159265359;
DEFINE_bool(show_motion_track, false, "visualize motion/track info");

std::vector<std::vector<int>> GLFWFusionViewer::s_color_table = {
    std::vector<int>{0, 0, 128},   std::vector<int>{0, 0, 255},
    std::vector<int>{0, 128, 255}, std::vector<int>{0, 128, 128},
    std::vector<int>{0, 128, 0},   std::vector<int>{0, 255, 0},
    std::vector<int>{0, 255, 128}, std::vector<int>{0, 255, 255},
    std::vector<int>{255, 0, 0},   std::vector<int>{255, 0, 128},
    std::vector<int>{255, 0, 255}, std::vector<int>{255, 255, 0},
    std::vector<int>{255, 128, 0}};

GLFWFusionViewer::GLFWFusionViewer()
    : init_(false),
      window_(NULL),
      pers_camera_(NULL),
      bg_color_(0.0, 0.0, 0.0),
      win_width_(2560),
      win_height_(1440),
      mouse_prev_x_(0),
      mouse_prev_y_(0),
      frame_content_(NULL),
      rgba_buffer_(NULL),
      vao_trans_x_(0.0),
      vao_trans_y_(0.0),
      vao_trans_z_(0.0),
      _Rotate_x(0.0),
      _Rotate_y(0.0),
      _Rotate_z(0.0),
      show_box(1),
      show_velocity(1),
      show_text(0),
      capture_screen_(false),
      capture_video_(FLAGS_capture_screen),
      scene_width_(1280),
      scene_height_(720),
      image_width_(1280),
      image_height_(720),
      frame_count_(0) {
  mode_mat_ = Eigen::Matrix4d::Identity();
}

GLFWFusionViewer::~GLFWFusionViewer() {
  close();
  if (pers_camera_) {
    delete pers_camera_;
  }
  if (rgba_buffer_) {
    delete[] rgba_buffer_;
    rgba_buffer_ = nullptr;
  }
}

void GLFWFusionViewer::get_class_color(int cls, float rgb[3]) {
  switch (cls) {
    case 0:
      rgb[0] = 0.5;
      rgb[1] = 0;
      rgb[2] = 1;  // pink
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
      rgb[2] = 0.5;  // close to red
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
  }
}

bool GLFWFusionViewer::initialize() {
  AINFO << "GLFWFusionViewer::initialize()" << std::endl;
  if (init_) {
    AINFO << " GLFWFusionViewer is already initialized !" << std::endl;
    return false;
  }

  if (!window_init()) {
    AERROR << " Failed to initialize the window !" << std::endl;
    return false;
  }

  if (!camera_init()) {
    AERROR << " Failed to initialize the camera !" << std::endl;
    return false;
  }

  if (!opengl_init()) {
    AERROR << " Failed to initialize opengl !" << std::endl;
    return false;
  }

  if (FLAGS_show_radar_objects) {
    show_box = 1;
    show_velocity = 1;
    show_text = 1;
  }
  if (FLAGS_show_fused_objects) {
    show_box = 1;
    show_velocity = 1;
    show_text = 1;
  }

  // for camera visualization
  _show_camera_box2d = true;
  _show_camera_box3d = true;
  show_radar_pc_ = false;
  show_fusion_pc_ = false;
  show_associate_color_ = false;
  show_type_id_label_ = true;

  CalibrationConfigManager* config_manager =
      Singleton<CalibrationConfigManager>::get();
  CameraCalibrationPtr calibrator = config_manager->get_camera_calibration();
  camera_intrinsic_ = calibrator->get_camera_intrinsic();
  distort_camera_intrinsic_ = calibrator->get_camera_model();
  AINFO << " GLFWFusionViewer::initialize() config_manager" << std::endl;

  // Init Raster Text
  raster_text_ = std::make_shared<GLRasterText>();
  raster_text_->init();

  AINFO << " GLFWFusionViewer::initialize() Finished" << std::endl;
  init_ = true;
  return true;
}

void GLFWFusionViewer::spin() {
  while (!glfwWindowShouldClose(window_) && frame_content_) {
    glfwPollEvents();
    render();
    glfwSwapBuffers(window_);
  }
  glfwDestroyWindow(window_);
}

void GLFWFusionViewer::spin_once() {
  if (!frame_content_) {
    AWARN << "GLFWFusionViewer::spin_once : No frame content";
    return;
  }

  AINFO << "GLFWFusionViewer::spin_once()";
  glfwPollEvents();
  render();
  glfwSwapBuffers(window_);
}

void GLFWFusionViewer::close() { glfwTerminate(); }

void GLFWFusionViewer::set_camera_para(Eigen::Vector3d i_position,
                                       Eigen::Vector3d i_scn_center,
                                       Eigen::Vector3d i_up_vector) {
  pers_camera_->set_position(i_position);
  pers_camera_->setscene_center(i_scn_center);
  pers_camera_->setup_vector(i_up_vector);
  pers_camera_->look_at(i_scn_center);

  GLdouble v_mat[16];
  pers_camera_->get_model_view_matrix(v_mat);
  view_mat_ << v_mat[0], v_mat[4], v_mat[8], v_mat[12], v_mat[1], v_mat[5],
      v_mat[9], v_mat[13], v_mat[2], v_mat[6], v_mat[10], v_mat[14], v_mat[3],
      v_mat[7], v_mat[11], v_mat[15];
}

bool GLFWFusionViewer::window_init() {
  if (!glfwInit()) {
    std::cerr << "Failed to initialize glfw !\n";
    return false;
  }

  // window_ = glfwCreateWindow(win_width_, win_height_, "opengl_visualizer",
  // nullptr, nullptr);
  // glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);
  win_width_ = scene_width_ + image_width_;
  win_height_ =
      (image_height_ * 2 > win_height_) ? image_height_ * 2 : win_height_;
  window_ = glfwCreateWindow(win_width_, win_height_, "gl_camera_visualizer",
                             nullptr, nullptr);
  if (window_ == nullptr) {
    std::cerr << "Failed to create glfw window!\n";
    glfwTerminate();
    return false;
  }

  glfwMakeContextCurrent(window_);
  glfwSwapInterval(1);
  glfwSetWindowUserPointer(window_, this);

  // set callback functions
  glfwSetFramebufferSizeCallback(window_, framebuffer_size_callback);

  glfwSetKeyCallback(window_, key_callback);
  glfwSetMouseButtonCallback(window_, mouse_button_callback);
  glfwSetCursorPosCallback(window_, mouse_cursor_position_callback);
  glfwSetScrollCallback(window_, mouse_scroll_callback);

  glfwShowWindow(window_);
  return true;
}

bool GLFWFusionViewer::camera_init() {
  // perspective cameras
  pers_camera_ = new Camera;
  pers_camera_->set_type(Camera::Type::PERSPECTIVE);
  pers_camera_->setscene_radius(1000);
  pers_camera_->set_position(Eigen::Vector3d(0, 0, -30));
  pers_camera_->setscreen_widthandheight(scene_width_, scene_height_);
  pers_camera_->look_at(Eigen::Vector3d(0, 0, 0));
  double fov = 45 * (My_PI / 180.0);
  pers_camera_->setfield_of_view(fov);
  return true;
}

bool GLFWFusionViewer::opengl_init() {
  glViewport(0, 0, scene_width_, scene_height_);
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
  glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
  glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);
  glEnable(GL_LIGHT0);
  glEnable(GL_LIGHTING);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_NORMALIZE);
  glEnable(GL_COLOR_MATERIAL);

  GLenum err = glewInit();
  if (GLEW_OK != err) {
    fprintf(stderr, "GLEW init failed！\n");
    exit(EXIT_FAILURE);
  }
  /*********************************************************   gen cloud vao &
   * vbo   **********************************************************/
  {
    int i = 0;

    GLfloat cloudColors[VBO_cloud_num][3];
    GLuint cloudIndices[VBO_cloud_num];
    for (i = 0; i < VBO_cloud_num; i++) {
      cloudColors[i][0] = 0.7;
      cloudColors[i][1] = 0.7;
      cloudColors[i][2] = 0.7;
      cloudIndices[i] = (GLuint)i;
    }

    glGenVertexArrays(VAO_cloud_num, VAO_cloud);
    for (i = 0; i < VAO_cloud_num; i++) {
      glBindVertexArray(VAO_cloud[i]);
      // buffer object
      glGenBuffers(NumVBOs, buffers_cloud[i]);
      glBindBuffer(GL_ARRAY_BUFFER, buffers_cloud[i][vertices]);
      glBufferData(GL_ARRAY_BUFFER, sizeof(cloudVerts), cloudVerts,
                   GL_STREAM_DRAW);
      glVertexPointer(3, GL_FLOAT, 0, BUFFER_OFFSET(0));
      glEnableClientState(GL_VERTEX_ARRAY);

      glBindBuffer(GL_ARRAY_BUFFER, buffers_cloud[i][colors]);
      glBufferData(GL_ARRAY_BUFFER, sizeof(cloudColors), cloudColors,
                   GL_STREAM_DRAW);
      glColorPointer(3, GL_FLOAT, 0, BUFFER_OFFSET(0));
      glEnableClientState(GL_COLOR_ARRAY);

      glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, buffers_cloud[i][elements]);
      glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(cloudIndices), cloudIndices,
                   GL_STREAM_DRAW);
    }
  }

  /*********************************************************   gen circle vao &
   * vbo   ***************************************************************/
  {
    GLuint buffers_circle[VAO_circle_num][NumVBOs];
    GLfloat circleVerts[VBO_circle_num][3];
    GLfloat circleColors[VBO_circle_num][3];
    GLuint circleIndices[VBO_circle_num];

    float dTheta = 2 * 3.1415926f / static_cast<float>(VBO_circle_num);
    int i = 0;
    int vao = 0;
    for (i = 0; i < VBO_circle_num; i++) {
      circleVerts[i][2] = -1.0;

      circleColors[i][0] = 0.0;
      circleColors[i][1] = 1.0;
      circleColors[i][2] = 1;

      circleIndices[i] = (GLuint)i;
    }

    glGenVertexArrays(VAO_circle_num, VAO_circle);

    for (vao = 0; vao < VAO_circle_num; vao++) {
      for (i = 0; i < VBO_circle_num; i++) {
        float theta = static_cast<float>(i * dTheta);
        circleVerts[i][0] = 20 * (vao + 1) * cos(theta);
        circleVerts[i][1] = 20 * (vao + 1) * sin(theta);
      }
      glBindVertexArray(VAO_circle[vao]);
      glGenBuffers(NumVBOs, buffers_circle[vao]);
      glBindBuffer(GL_ARRAY_BUFFER, buffers_circle[vao][vertices]);
      glBufferData(GL_ARRAY_BUFFER, sizeof(circleVerts), circleVerts,
                   GL_STATIC_DRAW);
      glVertexPointer(3, GL_FLOAT, 0, BUFFER_OFFSET(0));
      glEnableClientState(GL_VERTEX_ARRAY);

      glBindBuffer(GL_ARRAY_BUFFER, buffers_circle[vao][colors]);
      glBufferData(GL_ARRAY_BUFFER, sizeof(circleColors), circleColors,
                   GL_STATIC_DRAW);
      glColorPointer(3, GL_FLOAT, 0, BUFFER_OFFSET(0));
      glEnableClientState(GL_COLOR_ARRAY);
      glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, buffers_circle[vao][elements]);
      glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(circleIndices),
                   circleIndices, GL_STATIC_DRAW);
    }
  }
  /*********************************************************   gen cube vao &
   * vbo   ***************************************************************/

  //    v6----- v5
  //   /|      /|
  //  v1------v0|
  //  | |     | |
  //  | |v7---|-|v4
  //  |/      |/
  //  v2------v3

  return true;
}

void GLFWFusionViewer::pre_draw() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  pers_camera_->load_projection_matrix();

  // column major
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
  glMatrixMode(GL_MODELVIEW);
  glLoadMatrixd(mode_mat);
  glMultMatrixd(view_mat);

  GLfloat light_position[] = {1.0, -1.0, 1.0, 0.0};
  glLightfv(GL_LIGHT0, GL_POSITION, light_position);
}

void GLFWFusionViewer::draw_fusion_association(FrameContent* content) {
  std::map<int, int> cam_track_id_2_ind;
  std::vector<ObjectPtr> cam_objects = content->get_camera_objects();
  std::vector<ObjectPtr> fusion_objects = content->get_fused_objects();
  for (size_t i = 0; i < cam_objects.size(); i++) {
    ObjectPtr obj = cam_objects[i];
    cam_track_id_2_ind[obj->track_id] = i;
  }
  glColor3f(1, 0, 0);
  glLineWidth(2);
  glBegin(GL_LINES);
  for (size_t i = 0; i < fusion_objects.size(); i++) {
    ObjectPtr obj = fusion_objects[i];
    if (obj->camera_supplement == nullptr) {
      continue;
    }
    int cam_track_id = obj->camera_supplement->local_track_id;
    std::map<int, int>::iterator it = cam_track_id_2_ind.find(cam_track_id);
    if (it != cam_track_id_2_ind.end()) {
      int cam_ind = it->second;
      const Eigen::Vector3d& cam_ct = cam_objects[cam_ind]->center;
      const Eigen::Vector3d& fused_ct = obj->center;
      glVertex3f(cam_ct[0], cam_ct[1], cam_ct[2]);
      glVertex3f(fused_ct[0], fused_ct[1], fused_ct[2]);
    }
  }
  glLineWidth(1);
  glColor4f(1.0, 1.0, 1.0, 1.0);
  glEnd();
  glFlush();
}

vec3 GLFWFusionViewer::get_velocity_src_position(const ObjectPtr& object) {
  vec3 velocity_src;
  vec3 center;
  vec3 direction;
  vec3 size;
  vec3 velocity;

  center.x = object->center[0];
  center.y = object->center[1];
  center.z = object->center[2];
  direction.x = object->direction[0];
  direction.y = object->direction[1];
  direction.z = object->direction[2];
  size.x = object->length;
  size.y = object->width;
  size.z = object->height;
  velocity.x = object->velocity[0];
  velocity.y = object->velocity[1];
  velocity.z = object->velocity[2];
  float cos_direction_velocity =
      (direction.x * direction.y + velocity.x * velocity.y) /
      sqrt(direction.x * direction.x + direction.y * direction.y) /
      sqrt(velocity.x * velocity.x + velocity.y * velocity.y);
  float cos_dir =
      direction.x / sqrt(direction.x * direction.x + direction.y * direction.y);
  float sin_dir = -1 * direction.y /
                  sqrt(direction.x * direction.x + direction.y * direction.y);
  float x1 = 0.0f;
  float y1 = 0.0f;
  float x2 = 0.0f;
  float y2 = 0.0f;
  float x11 = 0.0f;
  float y11 = 0.0f;
  float x22 = 0.0f;
  float y22 = 0.0f;
  if (abs(cos_direction_velocity) > 0.707) {  // <45 degree
    x1 = size.x / 2;
    y1 = 0;
    x2 = x1 * -1;
    y2 = 0;
  } else {
    x1 = 0;
    y1 = size.y / 2;
    x2 = 0;
    y2 = y1 * -1;
  }

  x11 = x1 * cos_dir + y1 * sin_dir + velocity.x;
  y11 = y1 * cos_dir - x1 * sin_dir + velocity.y;
  x22 = x2 * cos_dir + y2 * sin_dir + velocity.x;
  y22 = y2 * cos_dir - x2 * sin_dir + velocity.y;

  float dis1 = x11 * x11 + y11 * y11;
  float dis2 = x22 * x22 + y22 * y22;
  if (dis1 > dis2) {
    velocity_src.x = x11 - velocity.x + center.x;
    velocity_src.y = y11 - velocity.y + center.y;
  } else {
    velocity_src.x = x22 - velocity.x + center.x;
    velocity_src.y = y22 - velocity.y + center.y;
  }
  velocity_src.z = -1.0f;
  return velocity_src;
}

void GLFWFusionViewer::render() {
  glClear(GL_COLOR_BUFFER_BIT);

  frame_count_++;

  AINFO << "GLFWFusionViewer::render()";
  // 1. Bottom left, draw 3d detection and classification results (lidar tracked
  // objects), and lanes in ego-car ground space
  glViewport(0, 0, image_width_, image_height_);
  {
    pre_draw();
    glPushMatrix();
    glTranslatef(vao_trans_y_, vao_trans_x_, vao_trans_z_);
    glRotatef(_Rotate_x, 1, 0, 0);
    glRotatef(_Rotate_y, 0, 1, 0);
    bool show_fusion = false;
    draw_3d_classifications(frame_content_, show_fusion);
    draw_car_forward_dir();
    if (FLAGS_show_motion_track &&
        frame_content_->get_motion_buffer().size() > 0) {
          draw_car_trajectory(frame_content_);
    }
    glPopMatrix();
  }

  glViewport(0, 0, image_width_, image_height_);
  glPushMatrix();
  glTranslatef(vao_trans_y_, vao_trans_x_, vao_trans_z_);
  glRotatef(_Rotate_x, 1, 0, 0);
  glRotatef(_Rotate_y, 0, 1, 0);
  glPopMatrix();

  // 2. Bottom right
  glViewport(scene_width_, 0, image_width_, image_height_);
  glPushMatrix();
  glTranslatef(vao_trans_y_, vao_trans_x_, vao_trans_z_);
  glRotatef(_Rotate_x, 1, 0, 0);
  glRotatef(_Rotate_y, 0, 1, 0);
  glPopMatrix();

  // 3. Top left, draw 2d camera detection and classification results
  glViewport(0, scene_height_, image_width_, image_height_);
  draw_camera_frame(frame_content_, false);

  // 4. Top right, draw 2d detection and 3d classification results
  glViewport(scene_width_, scene_height_, image_width_, image_height_);
  draw_camera_frame(frame_content_);

  static int no_frame = 0;
  if (capture_video_ || capture_screen_) {
    double time_stamp = frame_content_->get_visualization_timestamp();
    char buffer[512];
    snprintf(buffer, sizeof(time_stamp), "./%.12f.bmp", time_stamp);
    std::string file_name = FLAGS_screen_output_dir + buffer;
    capture_screen(file_name);
    if (capture_screen_) {
      capture_screen_ = false;
    }
  }

  no_frame++;
}

/************************callback functions************************/

void GLFWFusionViewer::framebuffer_size_callback(GLFWwindow* window, int width,
                                                 int height) {
  void* user_data = glfwGetWindowUserPointer(window);
  if (user_data == NULL) {
    return;
  }

  GLFWFusionViewer* vis = static_cast<GLFWFusionViewer*>(user_data);
  vis->resize_framebuffer(width, height);
}

void GLFWFusionViewer::window_size_callback(GLFWwindow* window, int width,
                                            int height) {
  void* user_data = glfwGetWindowUserPointer(window);
  if (user_data == NULL) {
    return;
  }

  GLFWFusionViewer* vis = static_cast<GLFWFusionViewer*>(user_data);
  vis->resize_window(width, height);
}

void GLFWFusionViewer::key_callback(GLFWwindow* window, int key, int scancode,
                                    int action, int mods) {
  void* user_data = glfwGetWindowUserPointer(window);
  if (user_data == NULL) {
    return;
  }
  if (action == GLFW_PRESS) {
    GLFWFusionViewer* vis = static_cast<GLFWFusionViewer*>(user_data);
    AINFO << "key_value: " << key;
    vis->keyboard(key);
  }
}

void GLFWFusionViewer::mouse_button_callback(GLFWwindow* window, int button,
                                             int action, int mods) {}

void GLFWFusionViewer::mouse_cursor_position_callback(GLFWwindow* window,
                                                      double xpos,
                                                      double ypos) {
  void* user_data = glfwGetWindowUserPointer(window);
  if (user_data == NULL) {
    return;
  }

  GLFWFusionViewer* vis = static_cast<GLFWFusionViewer*>(user_data);
  vis->mouse_move(xpos, ypos);
}

void GLFWFusionViewer::mouse_scroll_callback(GLFWwindow* window, double xoffset,
                                             double yoffset) {
  void* user_data = glfwGetWindowUserPointer(window);
  if (user_data == NULL) {
    return;
  }

  GLFWFusionViewer* vis = static_cast<GLFWFusionViewer*>(user_data);
  vis->mouse_wheel(yoffset);
}

void GLFWFusionViewer::error_callback(int error, const char* description) {
  std::cout << "ERROR - " << error << "  " << description << "\n";
}

/************************callback assistants************************/
void GLFWFusionViewer::resize_window(int width, int height) {
  if (width == win_width_ && height == win_height_) {
    return;
  }
  if (rgba_buffer_ != nullptr) {
    delete[] rgba_buffer_;
    rgba_buffer_ = nullptr;
  }
  rgba_buffer_ = new unsigned char[width * height * 4];
  win_width_ = width;
  win_height_ = height;
  scene_width_ = win_width_ * 0.5;
  scene_height_ = win_height_ * 0.5;
  image_width_ = scene_width_;
  image_height_ = scene_height_;
  pers_camera_->setscreen_widthandheight(scene_width_, scene_height_);
}

void GLFWFusionViewer::resize_framebuffer(int width, int height) {
  if (width == win_width_ && height == win_height_) {
    return;
  }
  if (rgba_buffer_ != nullptr) {
    delete[] rgba_buffer_;
    rgba_buffer_ = nullptr;
  }
  rgba_buffer_ = new unsigned char[width * height * 4];
  win_width_ = width;
  win_height_ = height;
  scene_width_ = win_width_ * 0.5;
  scene_height_ = win_height_ * 0.5;
  image_width_ = scene_width_;
  image_height_ = scene_height_;
  pers_camera_->setscreen_widthandheight(scene_width_, scene_height_);
}

void GLFWFusionViewer::mouse_move(double xpos, double ypos) {
  int state_left = glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_LEFT);
  int state_right = glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_RIGHT);
  int x_delta = xpos - mouse_prev_x_;
  int y_delta = ypos - mouse_prev_y_;
  if (state_left == GLFW_PRESS) {
    Eigen::Quaterniond rot =
        pers_camera_->get_rotatation_by_mouse_from_qgwidget(
            mouse_prev_x_, mouse_prev_y_, xpos, ypos);
    Eigen::Matrix3d rot_mat = rot.inverse().toRotationMatrix();
    Eigen::Vector3d scn_center = pers_camera_->scene_center();
    Eigen::Vector4d scn_center_(scn_center(0), scn_center(1), scn_center(2), 1);
    scn_center_ = mode_mat_ * view_mat_ * scn_center_;
    scn_center = scn_center_.head(3);
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

void GLFWFusionViewer::mouse_wheel(double delta) { mode_mat_(2, 3) -= delta; }

void GLFWFusionViewer::reset() { mode_mat_ = Eigen::Matrix4d::Identity(); }

void GLFWFusionViewer::keyboard(int key) {
  switch (key) {
    case 82:  // 'R'
      reset();
      break;
    case 66:  // 'B'
      show_box = (show_box + 1) % 2;
      break;
    case 86:  // 'V'
      show_velocity = (show_velocity + 1) % 2;
      break;
    case 67:  // 'C'
      use_class_color_ = !use_class_color_;
      break;
    case 83:  // 'S'
      capture_screen_ = true;
      break;
    case 65:  // 'A'
      capture_video_ = !capture_video_;
      break;
    // for camera visualization
    case GLFW_KEY_I:
      show_type_id_label_ = !show_type_id_label_;
      break;
    case GLFW_KEY_Q:  // Q
      show_lane_ = !show_lane_;
      break;
    case GLFW_KEY_E:  // E
      draw_lane_objects_ = !draw_lane_objects_;
    case GLFW_KEY_F:  // F
      show_fusion_pc_ = !show_fusion_pc_;
      break;
    case GLFW_KEY_D:  // D
      show_radar_pc_ = !show_radar_pc_;
      break;
    case GLFW_KEY_2:  // 2
      _show_camera_box2d = !_show_camera_box2d;
      break;
    case GLFW_KEY_3:  // 3
      _show_camera_box3d = !_show_camera_box3d;
      break;
    case GLFW_KEY_0:  // 3
      show_associate_color_ = !show_associate_color_;
    default:
      break;
  }
}

void GLFWFusionViewer::capture_screen(const std::string& file_name) {
  if (rgba_buffer_ == nullptr) {
    rgba_buffer_ = new unsigned char[4 * win_width_ * win_height_];
    if (rgba_buffer_ == nullptr) {
      AERROR << "Failed to create screen capture buffer \n";
      return;
    }
  }
  glReadPixels(0, 0, win_width_, win_height_, GL_BGRA, GL_UNSIGNED_BYTE,
               rgba_buffer_);

  save_rgba_image_to_bmp<unsigned char>(rgba_buffer_, win_width_, win_height_,
                                        file_name.c_str());
}

GLuint GLFWFusionViewer::image_to_gl_texture(const cv::Mat& mat,
                                             GLenum min_filter,
                                             GLenum mag_filter,
                                             GLenum wrap_filter) {
  // Generate a number for our texture_id's unique handle
  GLuint texture_id;
  glGenTextures(1, &texture_id);

  // Bind to our texture handle
  glBindTexture(GL_TEXTURE_2D, texture_id);

  // Catch silly-mistake texture interpolation method for magnification
  if (mag_filter == GL_LINEAR_MIPMAP_LINEAR ||
      mag_filter == GL_LINEAR_MIPMAP_NEAREST ||
      mag_filter == GL_NEAREST_MIPMAP_LINEAR ||
      mag_filter == GL_NEAREST_MIPMAP_NEAREST) {
    std::cout << "You can't use MIPMAPs for magnification - setting filter to "
                 "GL_LINEAR"
              << std::endl;
    mag_filter = GL_LINEAR;
  }

  // Set texture interpolation methods for minification and magnification
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, min_filter);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, mag_filter);

  // Set texture clamping method
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, wrap_filter);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, wrap_filter);

  // Set incoming texture format
  GLenum input_color_format = GL_BGR;
  if (mat.channels() == 1) {
    input_color_format = GL_LUMINANCE;
  } else if (mat.channels() == 3) {
    input_color_format = GL_BGR;
  } else if (mat.channels() == 4) {
    input_color_format = GL_BGRA;
  } else {
    // Unknown color format
  }

  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

  // Create the texture
  glTexImage2D(GL_TEXTURE_2D,  // Type of texture
               0,       // Pyramid level (for mip-mapping) - 0 is the top level
               GL_RGB,  // Internal colour format to convert to
               mat.cols,  // Image width  i.e. 640 for Kinect in standard mode
               mat.rows,  // Image height i.e. 480 for Kinect in standard mode
               0,         // Border width in pixels (can either be 1 or 0)
               input_color_format,  // Input image format (i.e. GL_RGB, GL_RGBA,
                                    // GL_BGR etc.)
               GL_UNSIGNED_BYTE,    // Image data type
               mat.ptr());          // The actual image data itself

  // If we're using mipmaps then generate them. Note: This requires OpenGL 3.0
  // or higher
  if (min_filter == GL_LINEAR_MIPMAP_LINEAR ||
      min_filter == GL_LINEAR_MIPMAP_NEAREST ||
      min_filter == GL_NEAREST_MIPMAP_LINEAR ||
      min_filter == GL_NEAREST_MIPMAP_NEAREST) {
    glGenerateMipmap(GL_TEXTURE_2D);
  }

  return texture_id;
}

void GLFWFusionViewer::draw_camera_frame(FrameContent* content) {
  AINFO << "GLFWFusionViewer::draw_camera_frame";
  cv::Mat image_mat_src = content->get_camera_image().clone();
  if (image_mat_src.empty()) {
    AWARN << "GLFWFusionViewer::draw_camera_frame : No image found";
    return;
  }
  int image_width = image_mat_src.cols;
  int image_height = image_mat_src.rows;

  glMatrixMode(GL_PROJECTION);  // Operate on projection matrix
  glLoadIdentity();
  glOrtho(scene_width_, scene_width_ + image_width_, image_height_, 0.0, 0.0,
          100.0);

  glMatrixMode(GL_MODELVIEW);  // Operate on model-view matrix
  glLoadIdentity();

  glEnable(GL_TEXTURE_2D);
  GLuint image_tex = image_to_gl_texture(image_mat_src, GL_LINEAR_MIPMAP_LINEAR,
                                         GL_LINEAR, GL_CLAMP);

  /* Draw a quad */
  glBegin(GL_QUADS);
  glTexCoord2i(0, 0);
  glVertex2i(scene_width_, 0);
  glTexCoord2i(0, 1);
  glVertex2i(scene_width_, image_height_);
  glTexCoord2i(1, 1);
  glVertex2i(scene_width_ + image_width_, image_height_);
  glTexCoord2i(1, 0);
  glVertex2i(scene_width_ + image_width_, 0);
  glEnd();

  glDeleteTextures(1, &image_tex);
  glDisable(GL_TEXTURE_2D);

  // -----------------------------
  Eigen::Matrix4d camera_to_world_pose = content->get_camera_to_world_pose();

  Eigen::Matrix4d v2c = camera_to_world_pose.inverse();

  int offset_x = scene_width_;
  int offset_y = 0;
  if (_show_camera_box2d || _show_camera_box3d) {
    std::vector<ObjectPtr> camera_objects;
    camera_objects = content->get_camera_objects();
    draw_camera_box(camera_objects, v2c, offset_x, offset_y, image_width,
                    image_height);
  }

  if (show_fusion_pc_) {
    std::vector<ObjectPtr> objects;
    objects = content->get_fused_objects();
    draw_objects2d(objects, v2c, "fusion", offset_x, offset_y, image_width,
                   image_height);
  }

  if (show_radar_pc_) {
    std::vector<ObjectPtr> objects;
    objects = content->get_radar_objects();
    draw_objects2d(objects, v2c, "radar", offset_x, offset_y, image_width,
                   image_height);
  }
}

void GLFWFusionViewer::draw_camera_frame(FrameContent* content,
                                         bool show_3d_class) {
  cv::Mat image_mat_src = content->get_camera_image().clone();
  if (image_mat_src.empty()) {
    return;
  }
  int image_width = image_mat_src.cols;
  int image_height = image_mat_src.rows;

  glMatrixMode(GL_PROJECTION);  // Operate on projection matrix
  glLoadIdentity();
  glOrtho(scene_width_, scene_width_ + image_width_, image_height_, 0.0, 0.0,
          100.0);

  glMatrixMode(GL_MODELVIEW);  // Operate on model-view matrix
  glLoadIdentity();

  glEnable(GL_TEXTURE_2D);
  GLuint image_tex = image_to_gl_texture(image_mat_src, GL_LINEAR_MIPMAP_LINEAR,
                                         GL_LINEAR, GL_CLAMP);

  /* Draw a quad */
  glBegin(GL_QUADS);
  glTexCoord2i(0, 0);
  glVertex2i(scene_width_, 0);
  glTexCoord2i(0, 1);
  glVertex2i(scene_width_, image_height_);
  glTexCoord2i(1, 1);
  glVertex2i(scene_width_ + image_width_, image_height_);
  glTexCoord2i(1, 0);
  glVertex2i(scene_width_ + image_width_, 0);
  glEnd();

  glDeleteTextures(1, &image_tex);
  glDisable(GL_TEXTURE_2D);

  // -----------------------------
  Eigen::Matrix4d camera_to_world_pose = content->get_camera_to_world_pose();

  Eigen::Matrix4d v2c = camera_to_world_pose.inverse();

  int offset_x = scene_width_;
  int offset_y = 0;

  std::vector<ObjectPtr> camera_objects;
  camera_objects = content->get_camera_objects();
  // show 2d detection and classification
  if (!show_3d_class) {
    draw_camera_box2d(camera_objects, v2c, offset_x, offset_y, image_width,
                      image_height);
  } else {  // show 3d class
    std::vector<ObjectPtr> fused_objects;
    fused_objects = content->get_fused_objects();
    draw_camera_box3d(camera_objects, fused_objects, v2c, offset_x, offset_y,
                      image_width, image_height);
  }
}

bool GLFWFusionViewer::project_point_undistort(Eigen::Matrix4d v2c,
                                               Eigen::Vector3d pc,
                                               Eigen::Vector2d* p2d) {
  Eigen::Vector3d pc3d =
      (v2c * Eigen::Vector4d(pc[0], pc[1], pc[2], 1)).head(3);
  if (pc3d[2] < 0) {
    return false;
  }

  Eigen::Matrix<double, 2, 1> pc2d_mat;
  Eigen::Matrix<double, 3, 1> pc3d_mat;
  for (size_t i = 0; i < 3; ++i) {
    pc3d_mat(i, 0) = pc3d[i];
  }

  pc2d_mat = distort_camera_intrinsic_->project(pc3d_mat);
  (*p2d)[0] = pc2d_mat(0, 0);
  (*p2d)[1] = pc2d_mat(1, 0);

  return true;
}

void GLFWFusionViewer::get_8points(float width, float height, float length,
                                   std::vector<Eigen::Vector3d>* points) {
  points->clear();
  points->push_back(Eigen::Vector3d(-width / 2.0, 0, length / 2.0));
  points->push_back(Eigen::Vector3d(width / 2.0, 0, length / 2.0));
  points->push_back(Eigen::Vector3d(width / 2.0, 0, -length / 2.0));
  points->push_back(Eigen::Vector3d(-width / 2.0, 0, -length / 2.0));
  points->push_back(Eigen::Vector3d(-width / 2.0, -height, length / 2.0));
  points->push_back(Eigen::Vector3d(width / 2.0, -height, length / 2.0));
  points->push_back(Eigen::Vector3d(width / 2.0, -height, -length / 2.0));
  points->push_back(Eigen::Vector3d(-width / 2.0, -height, -length / 2.0));
}

bool GLFWFusionViewer::get_boundingbox(Eigen::Vector3d center,
                                       Eigen::Matrix4d v2c, float width,
                                       float height, float length,
                                       Eigen::Vector3d dir, float theta,
                                       std::vector<Eigen::Vector2d>* points) {
  // Eigen::Vector3d dir_world = (v2c * Eigen::Vector4d(dir[0], dir[1], dir[2],
  // 0)).head(3);
  Eigen::Matrix3d r = Eigen::Matrix3d::Identity();
  r << cos(theta), 0, sin(theta), 0, 1, 0, -sin(theta), 0, cos(theta);
  std::vector<Eigen::Vector3d> obj_points;
  get_8points(width, height, length, &obj_points);
  std::vector<Eigen::Vector3d> camera_points;
  camera_points.resize(8);
  if (points->size() != obj_points.size()) {
    AERROR << "cannot get 8 obj points";
    return false;
  }
  for (int i = 0; i < 8; i++) {
    camera_points[i] =
        camera_intrinsic_.block(0, 0, 3, 3) * (center + r * obj_points[i]);
    (*points)[i] = camera_points[i].head(2) / camera_points[i].z();
  }
  return true;
}

bool GLFWFusionViewer::get_project_point(Eigen::Matrix4d v2c,
                                         Eigen::Vector3d pc,
                                         Eigen::Vector2d* p2d) {
  Eigen::Vector3d pc3d =
      (v2c * Eigen::Vector4d(pc[0], pc[1], pc[2], 1)).head(3);
  if (pc3d[2] < 0) {
    return false;
  }
  Eigen::Vector3d pv = camera_intrinsic_.block(0, 0, 3, 3) * (pc3d);
  *p2d = pv.head(2) / pv.z();
  return true;
}

void GLFWFusionViewer::draw_line2d(const Eigen::Vector2d& p1,
                                   const Eigen::Vector2d& p2, int line_width,
                                   int r, int g, int b, int offset_x,
                                   int offset_y, int image_width,
                                   int image_height) {
  double x1 = offset_x + 1.0 * p1.x() * image_width_ / image_width;
  double y1 = offset_y + 1.0 * p1.y() * image_height_ / image_height;
  double x2 = offset_x + 1.0 * p2.x() * image_width_ / image_width;
  double y2 = offset_y + 1.0 * p2.y() * image_height_ / image_height;

  glColor3ub(r, g, b);
  glLineWidth(line_width);
  glBegin(GL_LINES);
  glVertex2i(x1, y1);
  glVertex2i(x2, y2);
  glEnd();

  glColor4f(1, 1, 1, 1);  // reset the color to white
  glLineWidth(1);
}

void GLFWFusionViewer::draw_camera_box2d(const std::vector<ObjectPtr>& objects,
                                         Eigen::Matrix4d v2c, int offset_x,
                                         int offset_y, int image_width,
                                         int image_height) {
  for (auto obj : objects) {
    Eigen::Vector3d center = obj->center;
    Eigen::Vector2d center2d;
    get_project_point(v2c, center, &center2d);
    ADEBUG << "camera obj " << obj->track_id << " center: " << center2d[0]
           << " " << center2d[1];

    float theta = obj->theta;
    float width = obj->width;
    float height = obj->height;
    float length = obj->length;

    std::vector<Eigen::Vector2d> points;
    points.resize(8);

    Eigen::Vector3d tc = center.head(3);
    get_boundingbox(tc, v2c, width, height, length, obj->direction, theta,
                    &points);

    // auto box3d_color = s_color_table[0];
    // if (obj->camera_supplement != nullptr) {
    //     box3d_color = s_color_table[obj->track_id % s_color_table.size()];
    // }
    // if (_show_camera_box3d) {
    //     draw_8pts_box(points,
    //             Eigen::Vector3f(box3d_color[0], box3d_color[1],
    //             box3d_color[2]),
    //             offset_x, offset_y, image_width, image_height);
    // }

    if (_show_camera_box2d) {
      if (obj->camera_supplement != nullptr) {
        // use class color
        float rgb[3];
        auto tmp_color = s_color_table[obj->track_id % s_color_table.size()];
        rgb[0] = tmp_color[0] / 255.0;
        rgb[1] = tmp_color[1] / 255.0;
        rgb[2] = tmp_color[2] / 255.0;

        if (use_class_color_) {
          get_class_color(static_cast<unsigned>(obj->type), rgb);
        }
        // get_class_color(obj->type, rgb);
        int box2d_color[3];
        for (size_t i = 0; i < 3; ++i) {
          box2d_color[i] = static_cast<int>(255 * rgb[i]);
        }
        if (obj->b_cipv) {
          AINFO << "draw_camera_box2d This is CIPV, obj->track_id: "
                << obj->track_id;
          box2d_color[0] = 255;
          box2d_color[1] = 0;
          box2d_color[2] = 0;
        }
        // test track id
        // auto track_id_color = s_color_table[obj->track_id %
        // s_color_table.size()];
        // for (size_t i = 0; i < 3; ++i) {
        //     box2d_color[i] = track_id_color[i];
        // }

        auto upper_left_pt = obj->camera_supplement->upper_left;
        auto lower_right_pt = obj->camera_supplement->lower_right;
        draw_rect2d(upper_left_pt, lower_right_pt, 2, box2d_color[0],
                    box2d_color[1], box2d_color[2], offset_x, offset_y,
                    image_width, image_height);
        // Draw texts using OpenGL
        // distance
        if (show_type_id_label_) {
          std::string c = std::to_string(
              static_cast<int>((sqrt(tc.x() * tc.x() + tc.z() * tc.z()))));
          glColor3ub(box2d_color[0], box2d_color[1], box2d_color[2]);
          double x_txt = points[7].x();
          double y_txt = points[7].y() - 8;
          int xi_txt = offset_x + x_txt * image_width_ / image_width;
          int yi_txt = offset_y + y_txt * image_height_ / image_height;

          glRasterPos2i(xi_txt, yi_txt);
          raster_text_->print_string(c.c_str());
          glColor4f(1.0f, 1.0f, 1.0f, 1.0f);  // reset the color to white

          // type
          std::string obj_type_str;
          std::string id = std::to_string(obj->track_id);
          switch (obj->type) {
            case ObjectType::PEDESTRIAN:
              obj_type_str = "PED";
              break;
            case ObjectType::BICYCLE:
              obj_type_str = "CYC";
              break;
            case ObjectType::VEHICLE:
              obj_type_str = "CAR";
              break;
            default:
              break;
          }
          obj_type_str = obj_type_str + "(" + id + ")";
          glColor3ub(box2d_color[0], box2d_color[1], box2d_color[2]);
          double x_type = (upper_left_pt.x() + lower_right_pt.x()) / 2.0;
          double y_type = (upper_left_pt.y() + lower_right_pt.y()) / 2.0;
          int xi_type = offset_x + x_type * image_width_ / image_width;
          int yi_type = offset_y + y_type * image_height_ / image_height;

          glRasterPos2i(xi_type, yi_type);
          raster_text_->print_string(obj_type_str.c_str());
        }
        glColor4f(1.0f, 1.0f, 1.0f, 1.0f);  // reset the color to white
      }
    }
  }
}

void GLFWFusionViewer::draw_camera_box3d(
    const std::vector<ObjectPtr>& camera_objects,
    const std::vector<ObjectPtr>& fused_objects, Eigen::Matrix4d v2c,
    int offset_x, int offset_y, int image_width, int image_height) {
  std::map<int, int> cam_track_id_2_ind;
  for (size_t i = 0; i < camera_objects.size(); ++i) {
    auto obj = camera_objects[i];
    cam_track_id_2_ind[obj->track_id] = i;
  }

  for (auto fused_obj : fused_objects) {
    if (fused_obj->camera_supplement == nullptr) {
      continue;
    }
    int cam_track_id = fused_obj->camera_supplement->local_track_id;
    auto it = cam_track_id_2_ind.find(cam_track_id);

    auto fused_type = fused_obj->type;
    if (it != cam_track_id_2_ind.end()) {
      int cam_ind = it->second;
      auto obj = camera_objects[cam_ind];
      Eigen::Vector3d center = obj->center;
      Eigen::Vector2d center2d;
      get_project_point(v2c, center, &center2d);
      AINFO << "camera obj " << obj->track_id << " center: " << center2d[0]
            << " " << center2d[1];

      float theta = obj->theta;
      float width = obj->width;
      float height = obj->height;
      float length = obj->length;

      std::vector<Eigen::Vector2d> points;
      points.resize(8);
      Eigen::Vector3d tc =
          (v2c * Eigen::Vector4d(center[0], center[1], center[2], 1)).head(3);

      get_boundingbox(tc, v2c, width, height, length, obj->direction, theta,
                      &points);

      // use 3d class color
      float rgb[3];
      get_class_color(static_cast<unsigned>(fused_type), rgb);
      int box3d_color[3];
      for (size_t i = 0; i < 3; ++i) {
        box3d_color[i] = static_cast<int>(255 * rgb[i]);
      }

      if (_show_camera_box3d) {
        draw_8pts_box(points, Eigen::Vector3f(box3d_color[0], box3d_color[1],
                                              box3d_color[2]),
                      offset_x, offset_y, image_width, image_height);
      }
    }
  }

  // for (auto obj : camera_objects) {
  //     Eigen::Vector3d center = obj->center;
  //     Eigen::Vector2d center2d;
  //     get_project_point(v2c, center, &center2d);
  //     ADEBUG << "camera obj " << obj->track_id << " center: " << center2d[0]
  //                 << " " << center2d[1];

  //     float theta = obj->theta;
  //     float width = obj->width;
  //     float height = obj->height;
  //     float length = obj->length;

  //     std::vector<Eigen::Vector2d> points;
  //     points.resize(8);
  //     Eigen::Vector3d tc = (v2c * Eigen::Vector4d(center[0], center[1],
  //     center[2], 1)).head(
  //             3);

  //     get_boundingbox(tc, v2c, width, height, length, obj->direction, theta,
  //     &points);

  //     // use class color
  //     float rgb[3];
  //     get_class_color(obj->type, rgb);
  //     int box3d_color[3];
  //     for (size_t i = 0; i < 3; ++i) {
  //         box3d_color[i] = static_cast<int>(255 * rgb[i]);
  //     }

  //     if (_show_camera_box3d) {
  //         draw_8pts_box(points,
  //                 Eigen::Vector3f(box3d_color[0], box3d_color[1],
  //                 box3d_color[2]),
  //                 offset_x, offset_y, image_width, image_height);
  //     }
  // }
}

void GLFWFusionViewer::draw_rect2d(const Eigen::Vector2d& p1,
                                   const Eigen::Vector2d& p2, int line_width,
                                   int r, int g, int b, int offset_x,
                                   int offset_y, int image_width,
                                   int image_height) {
  double x1 = offset_x + 1.0 * p1.x() * image_width_ / image_width;
  double y1 = offset_y + 1.0 * p1.y() * image_height_ / image_height;
  double x2 = offset_x + 1.0 * p2.x() * image_width_ / image_width;
  double y2 = offset_y + 1.0 * p2.y() * image_height_ / image_height;

  glColor3ub(r, g, b);
  glLineWidth(line_width);
  glBegin(GL_LINES);
  glVertex2i(x1, y1);
  glVertex2i(x1, y2);

  glVertex2i(x1, y2);
  glVertex2i(x2, y2);

  glVertex2i(x2, y2);
  glVertex2i(x2, y1);

  glVertex2i(x2, y1);
  glVertex2i(x1, y1);
  glEnd();

  glColor4f(1.0f, 1.0f, 1.0f, 1.0f);  // reset the color to white
  glLineWidth(1);
}

bool GLFWFusionViewer::draw_car_forward_dir() {
  glColor3f(1.0, 0.5, 0.17);
  glLineWidth(3);
  glBegin(GL_LINES);
  Eigen::Vector3d center = pers_camera_->scene_center();
  Eigen::Vector3d forward_vp = center + forward_dir_ * 5;
  glVertex3f(center(0), center(1), center(2));
  glVertex3f(forward_vp(0), forward_vp(1), forward_vp(2));
  glEnd();
  glLineWidth(1);
  glColor4f(1.0, 1.0, 1.0, 1.0);
  return true;
}

void GLFWFusionViewer::draw_objects(const std::vector<ObjectPtr>& objects,
                                    const Eigen::Matrix4d& c2w, bool draw_cube,
                                    bool draw_velocity,
                                    const Eigen::Vector3f& color,
                                    bool use_class_color) {
  if (show_associate_color_) {
    use_class_color = false;
  }

  float rgb[3];
  if (!use_class_color) {
    rgb[0] = color[0];
    rgb[1] = color[1];
    rgb[2] = color[2];
  }
  // draw main car
  {
    glColor3f((GLfloat)255.0, (GLfloat)0.0, (GLfloat)0.0);
    glBegin(GL_LINE_STRIP);
    for (size_t i = 0; i < 5; ++i) {
      size_t index = i % 4;
      glVertex3f(main_car_[index][0], main_car_[index][1], main_car_[index][2]);
    }
    glEnd();
    glFlush();
  }
  if (draw_cube) {
    float verts[8][3];
    int i = 0;
    vec3 center;     // x,y,z
    vec3 direction;  // x,y,z
    vec3 size;       // len wid hei
    int indices[16] = {0, 1, 2, 3, 4, 5, 6, 7, 4, 3, 0, 5, 6, 1, 2, 7};
    for (i = 0; i < static_cast<int>(objects.size()); i++) {
      center.x = objects[i]->center[0];
      center.y = objects[i]->center[1];
      center.z = objects[i]->center[2];
      Eigen::Vector3d tc;
      tc[0] = center.x;
      tc[1] = center.y;
      tc[2] = center.z;
      direction.x = objects[i]->direction[0];
      direction.y = objects[i]->direction[1];
      direction.z = objects[i]->direction[2];
      size.x = objects[i]->length;
      size.y = objects[i]->width;
      size.z = objects[i]->height;
      if (size.x < 1.0e-2 && size.y < 1.0e-2 && size.z < 1.0e-2) {
        size.x = 0.1;
        size.y = 0.1;
        size.z = 0.1;
      }
      float x1 = size.x / 2;
      float x2 = 0 - x1;
      float y1 = size.y / 2;
      float y2 = 0 - y1;
      double len = sqrt(direction.x * direction.x + direction.y * direction.y);
      float cos_theta = direction.x / len;
      float sin_theta = -direction.y / len;
      // set x y
      verts[0][0] = verts[5][0] = x1 * cos_theta + y1 * sin_theta + tc[0];
      verts[0][1] = verts[5][1] = y1 * cos_theta - x1 * sin_theta + tc[1];

      verts[3][0] = verts[4][0] = x1 * cos_theta + y2 * sin_theta + tc[0];
      verts[3][1] = verts[4][1] = y2 * cos_theta - x1 * sin_theta + tc[1];

      verts[1][0] = verts[6][0] = x2 * cos_theta + y1 * sin_theta + tc[0];
      verts[1][1] = verts[6][1] = y1 * cos_theta - x2 * sin_theta + tc[1];

      verts[2][0] = verts[7][0] = x2 * cos_theta + y2 * sin_theta + tc[0];
      verts[2][1] = verts[7][1] = y2 * cos_theta - x2 * sin_theta + tc[1];

      // set z
      // verts[0][2] = verts[1][2] = verts[2][2] = verts[3][2] = tc[2] + size.z
      // / 2;
      // verts[4][2] = verts[5][2] = verts[6][2] = verts[7][2] = tc[2] - size.z
      // / 2;
      verts[0][2] = verts[1][2] = verts[2][2] = verts[3][2] = 0.0;
      verts[4][2] = verts[5][2] = verts[6][2] = verts[7][2] = 0.0;

      // draw same color with 2d camera bbox
      auto tmp_color =
          s_color_table[objects[i]->track_id % s_color_table.size()];
      rgb[0] = tmp_color[0];
      rgb[1] = tmp_color[1];
      rgb[2] = tmp_color[2];
      if (use_class_color) {
        get_class_color(static_cast<unsigned>(objects[i]->type), rgb);
      }

      if (objects[i]->b_cipv) {
        AINFO << "objects[i]->track_id: " << objects[i]->track_id;
        rgb[0] = 1;
        rgb[1] = 0;
        rgb[2] = 0;
      }

      glColor3f((GLfloat)rgb[0], (GLfloat)rgb[1], (GLfloat)rgb[2]);
      glBegin(GL_LINE_STRIP);
      int j = 0;
      for (j = 0; j < 16; j++) {
        glVertex3f((GLfloat)verts[indices[j]][0], (GLfloat)verts[indices[j]][1],
                   (GLfloat)verts[indices[j]][2]);
      }
      glEnd();
      glFlush();

      glRasterPos2i(tc[0], tc[1]);
      raster_text_->print_string(std::to_string(objects[i]->track_id).c_str());

      if (objects[i]->b_cipv) {
        glRasterPos2i(tc[0] + 3, tc[1]);
        raster_text_->print_string(std::string("cipv").c_str());
      }
      AINFO << objects[i]->ToString();
    }
  }

  draw_velocity = false;
  if (draw_velocity) {
    int i = 0;
    vec3 velocity_src;
    vec3 velocity_dst;
    float rgb[3] = {1, 1, 0};
    for (i = 0; i < static_cast<int>(objects.size()); i++) {
      velocity_src = get_velocity_src_position(objects[i]);
      velocity_dst.x = velocity_src.x + objects[i]->velocity[0];
      velocity_dst.y = velocity_src.y + objects[i]->velocity[1];
      velocity_dst.z = -1.0f;

      // draw same color with 2d camera bbox
      auto tmp_color =
          s_color_table[objects[i]->track_id % s_color_table.size()];
      rgb[0] = tmp_color[0];
      rgb[1] = tmp_color[1];
      rgb[2] = tmp_color[2];

      if (use_class_color) {
        get_class_color(static_cast<unsigned>(objects[i]->type), rgb);
      }
      glColor3f((GLfloat)rgb[0], (GLfloat)rgb[1], (GLfloat)rgb[2]);
      glBegin(GL_LINES);
      glVertex3f((GLfloat)velocity_src.x, (GLfloat)velocity_src.y,
                 (GLfloat)velocity_src.z);
      glVertex3f((GLfloat)velocity_dst.x, (GLfloat)velocity_dst.y,
                 (GLfloat)velocity_dst.z);
      glEnd();
      glFlush();
    }
  }
  glColor4f(1.0, 1.0, 1.0, 1.0);  // reset to white color
}

bool GLFWFusionViewer::draw_objects(FrameContent* content, bool draw_cube,
                                    bool draw_velocity) {
  Eigen::Matrix4d c2v = content->get_camera_to_world_pose();

  if (FLAGS_show_camera_objects) {
    Eigen::Vector3f cam_color(1, 1, 0);
    std::vector<ObjectPtr> objects = content->get_camera_objects();
    draw_objects(objects, c2v, draw_cube, draw_velocity, cam_color,
                 use_class_color_);
  }
  if (FLAGS_show_radar_objects) {
    Eigen::Vector3f radar_color(0, 1, 0);
    std::vector<ObjectPtr> objects = content->get_radar_objects();
    draw_objects(objects, c2v, draw_cube, draw_velocity, radar_color, false);
  }
  if (FLAGS_show_fused_objects) {
    Eigen::Vector3f fused_color(1, 0, 1);
    std::vector<ObjectPtr> objects = content->get_fused_objects();
    draw_objects(objects, c2v, draw_cube, draw_velocity, fused_color,
                 use_class_color_);
  }

  if (FLAGS_show_fusion_association) {
    draw_fusion_association(content);
  }
  return true;
}

void drawHollowCircle(GLfloat x, GLfloat y, GLfloat radius) {
    int i = 0;
    int lineAmount = 100;  // # of triangles used to draw circle

    // GLfloat radius = 0.8f;
    GLfloat twicePi = 2.0f * My_PI;

    glBegin(GL_LINE_LOOP);
//    glColor3f((GLfloat) 255.0, (GLfloat) 255.0, (GLfloat) 0.0);

    for (i = 0; i <= lineAmount; i++) {
        glVertex2f(
                (x + (radius * cos(i *  twicePi / lineAmount))),
                (y + (radius* sin(i * twicePi / lineAmount))));
    }
    glColor4f(1.0, 1.0, 1.0, 1.0);
    glEnd();
}

void GLFWFusionViewer::draw_car_trajectory(FrameContent* content) {
    const MotionBuffer& motion_buffer = content->get_motion_buffer();
    // Eigen::Vector3d center_d = pers_camera_->scene_center();
    Eigen::Vector3f center;
//    center << center_d(0,0),
//              center_d(1,0),
//              1.0;

    center <<  10,
            10,
            1.0;

//    std::cout << "GLViewer motion_buffer.size() : "
//              << motion_buffer.size() << std::endl;

    Eigen::Vector3f point = center;
    for (int i = motion_buffer.size() - 1; i >= 0; i--) {
        Eigen::Matrix3f tmp = motion_buffer[i].motion;
        Eigen::Matrix2f rotat2d = tmp.block(0, 0, 2, 2);
        Eigen::Vector2f trans = tmp.block(0, 2, 2, 1);

        // [R, T]^-1 = [R^t, -R^t*T]
        tmp.block(0, 0, 2, 2) = rotat2d.transpose();
        tmp.block(0, 2, 2, 1) = -rotat2d.transpose() * trans;

        point = tmp * center;

        point[0] = 2*center[0] - point[0];
        point[1] = 2*center[1] - point[1];
//        point = 2*center - point;
//        std::cout << "trajectory points: (" << point(0,0) << ", "
//                  << point(1,0) << ", " << point(2,0)
//                  << "); ";
        drawHollowCircle(point(0), point(1), 1);
        glFlush();
    }
//    std::cout <<  std::endl;
}

void GLFWFusionViewer::draw_trajectories(FrameContent* content) {
    std::vector<ObjectPtr> objects = content->get_camera_objects();
    double time_stamp = frame_content_->get_visualization_timestamp();

//    double camera_timestamp = content->get_camera_timestamp();
//    double content_motion_timestamp = content->get_motion_timestamp();
//    std::cout << "content object and motion timestamp: " <<
//        std::to_string(camera_timestamp) << " "
//              << std::to_string(content_motion_timestamp) <<std::endl;

    const MotionBuffer &motion_buffer = content->get_motion_buffer();
    int motion_size = motion_buffer.size();
    if (motion_size > 0) {
        // auto &motion_mat = motion_buffer[motion_buffer.size() - 1].motion;
//        Eigen::Matrix3f tmp = motion_mat;
//        Eigen::Matrix2f rotat2d = tmp.block(0, 0, 2, 2);
//        Eigen::Vector2f trans   = tmp.block(0, 2, 2, 1);
//        //[R, T]^-1 = [R^t, -R^t*T]
//        tmp.block(0, 0, 2, 2) = rotat2d.transpose();
//        tmp.block(0, 2, 2, 1) = -rotat2d.transpose() * trans;
//        std::cout << "current motion :" << tmp <<std::endl;
//        std::cout << "inversed by: " << motion_mat<<std::endl;


        std::map<int, std::vector<std::pair<float, float> > >
          tmp_object_trackjectories;
        std::map<int, std::vector<double>> tmp_object_timestamps;
        std::swap(object_trackjectories_, tmp_object_trackjectories);
        std::swap(object_timestamps_, tmp_object_timestamps);

        for (auto obj : objects) {
            int cur_id = obj->track_id;
            for (auto point : tmp_object_trackjectories[cur_id]) {
                    object_trackjectories_[cur_id].push_back(point);
            }
            for (auto ts : tmp_object_timestamps[cur_id]) {
                object_timestamps_[cur_id].push_back(ts);
            }
            object_trackjectories_[cur_id].push_back(
                        std::make_pair(obj->center[2], -obj->center[0]));
            object_timestamps_[cur_id].push_back(time_stamp);
        }

        glColor3f(1.0, 0.5, 0.17);
        for (auto &trackjectory : object_trackjectories_) {
            if (trackjectory.second.size() > 1) {
                glLineWidth(1);
                glBegin(GL_LINE_STRIP);
////                logging
                // int cur_id = trackjectory.first;
//                auto &timestamps = object_timestamps[cur_id];
//                std::cout<< "track: " << cur_id << " with size " <<
//                  trackjectory.second.size() <<": ";

//                std::cout<<"motion time: ";
//              for(int it = motion_buffer.size()-1, count=0;
//                  it >= 0; it--, count++) {
//                    if(count>=10) {
//                        break;
//                    }
//                    std::cout<<std::to_string(motion_buffer[it].time_t) <<" ";
////                    std::cout<<motion_buffer[it].motion<<std::endl;
//                }
//                std::cout<<std::endl;

//                std::cout <<"object time: ";
//                for(int it = timestamps.size() - 1, count = 0;
//                      it > 0; it--, count++) {
//                    if (count>=10 ) {
//                        break;
//                    }
//                    std::cout<<std::to_string(timestamps[it]) <<" ";
//                }
//                std::cout<<std::endl;
                for (std::size_t it = trackjectory.second.size() - 1, count = 0;
                      it > 0; it--, count++) {
                    if (count >= 10 || count > motion_buffer.size()) {
                        continue;
                    }

                    Eigen::Vector3f pt, proj_pt;
                    pt <<   trackjectory.second[it].first,
                            trackjectory.second[it].second,
                            1.0;
                    if (it == trackjectory.second.size() - 1) {
                        proj_pt = pt;
                    } else {
                        auto &motion_mat =
                          motion_buffer[motion_size - count].motion;
                        auto tmp = motion_mat.inverse();
                        proj_pt = tmp * pt;
                    }
                    proj_pt[0] = 2 * pt[0] - proj_pt[0];
                    proj_pt[1] = 2 * pt[1] - proj_pt[1];

                    glVertex2f(proj_pt[0], proj_pt[1]);
//                  drawHollowCircle(proj_pt[0], proj_pt[1], 0.7);
//                  drawHollowCircle(2*pt[0]-proj_pt[0],
//                    2*pt[1]-proj_pt[1], 0.4);
                }
                glEnd();
                glLineWidth(1);
            }
        }
        glColor4f(1.0, 1.0, 1.0, 1.0);
    }
}

void GLFWFusionViewer::draw_3d_classifications(FrameContent* content,
                                               bool show_fusion) {
  Eigen::Matrix4d c2v = content->get_camera_to_world_pose();

  if (show_fusion) {
    if (!FLAGS_show_fused_objects) {
      return;
    } else {
      Eigen::Vector3f fused_color(1, 0, 1);
      bool draw_cube = true;
      bool draw_velocity = true;
      std::vector<ObjectPtr> objects = content->get_fused_objects();
      draw_objects(objects, c2v, draw_cube, draw_velocity, fused_color,
                   use_class_color_);
    }

    if (FLAGS_show_fusion_association) {
      draw_fusion_association(content);
    }
  } else {
    if (!FLAGS_show_camera_objects && !FLAGS_show_radar_objects) {
      return;
    } else {
      if (FLAGS_show_camera_objects) {
        Eigen::Vector3f cam_color(1, 1, 0);
        bool draw_cube = true;
        bool draw_velocity = true;
        std::vector<ObjectPtr> objects = content->get_camera_objects();
        if (FLAGS_show_motion_track &&
            content->get_motion_buffer().size() > 0) {
              draw_trajectories(content);
        }

        draw_objects(objects, c2v, draw_cube, draw_velocity, cam_color,
                     use_class_color_);
      }
      if (FLAGS_show_radar_objects) {
        Eigen::Vector3f radar_color(1, 1, 1);
        bool draw_cube = true;
        bool draw_velocity = true;
        std::vector<ObjectPtr> objects = content->get_radar_objects();
        draw_objects(objects, c2v, draw_cube, draw_velocity, radar_color,
                     false);
      }
    }
  }
}

void GLFWFusionViewer::draw_camera_box(const std::vector<ObjectPtr>& objects,
                                       Eigen::Matrix4d v2c, int offset_x,
                                       int offset_y, int image_width,
                                       int image_height) {
  for (auto obj : objects) {
    Eigen::Vector3d center = obj->center;
    Eigen::Vector2d center2d;
    get_project_point(v2c, center, &center2d);
    AINFO << "camera obj " << obj->track_id << " center: " << center[0] << " "
          << center[1];

    float theta = obj->theta;
    float width = obj->width;
    float height = obj->height;
    float length = obj->length;

    std::vector<Eigen::Vector2d> points;
    points.resize(8);
    Eigen::Vector3d tc =
        (v2c * Eigen::Vector4d(center[0], center[1], center[2], 1)).head(3);
    get_boundingbox(tc, v2c, width, height, length, obj->direction, theta,
                    &points);

    auto box3d_color = s_color_table[0];
    if (obj->camera_supplement != nullptr) {
      box3d_color = s_color_table[obj->track_id % s_color_table.size()];
    }

    if (_show_camera_box3d) {
      draw_8pts_box(points, Eigen::Vector3f(box3d_color[0], box3d_color[1],
                                            box3d_color[2]),
                    offset_x, offset_y, image_width, image_height);
    }

    // TODO(All) fix the code after continue
    continue;
    /*
    if (_show_camera_box2d) {
      if (obj->camera_supplement != nullptr) {
        auto box2d_color = s_color_table[obj->track_id % s_color_table.size()];
        auto upper_left_pt = obj->camera_supplement->upper_left;
        auto lower_right_pt = obj->camera_supplement->lower_right;
        draw_rect2d(upper_left_pt, lower_right_pt, 2, box2d_color[0],
                    box2d_color[1], box2d_color[2], offset_x, offset_y,
                    image_width, image_height);
        // Draw texts using OpenGL
        // distance
        std::string c = std::to_string(
            static_cast<int>((sqrt(tc.x() * tc.x() + tc.z() * tc.z()))));
        auto dis_txt_color =
            s_color_table[obj->track_id % s_color_table.size()];
        glColor3ub(dis_txt_color[0], dis_txt_color[1], dis_txt_color[2]);
        double x_txt = points[7].x();
        double y_txt = points[7].y() - 8;
        int xi_txt = offset_x + x_txt * image_width_ / image_width;
        int yi_txt = offset_y + y_txt * image_height_ / image_height;

        glRasterPos2i(xi_txt, yi_txt);
        raster_text_->print_string(c.c_str());
        glColor4f(1.0f, 1.0f, 1.0f, 1.0f);  // reset the color to white

        // type
        std::string obj_type_str;
        switch (obj->type) {
          case ObjectType::PEDESTRIAN:
            obj_type_str = "PED";
            break;
          case ObjectType::BICYCLE:
            obj_type_str = "CYC";
            break;
          case ObjectType::VEHICLE:
            obj_type_str = "CAR";
            break;
          default:
            break;
        }
        auto type_txt_color =
            s_color_table[obj->track_id % s_color_table.size()];
        glColor3ub(type_txt_color[0], type_txt_color[1], type_txt_color[2]);
        double x_type = (upper_left_pt.x() + lower_right_pt.x()) / 2.0;
        double y_type = (upper_left_pt.y() + lower_right_pt.y()) / 2.0;
        int xi_type = offset_x + x_type * image_width_ / image_width;
        int yi_type = offset_y + y_type * image_height_ / image_height;

        glRasterPos2i(xi_type, yi_type);
        raster_text_->print_string(obj_type_str.c_str());
        glColor4f(1.0f, 1.0f, 1.0f, 1.0f);  // reset the color to white
      }
    }
    */
  }
}

void GLFWFusionViewer::draw_objects2d(const std::vector<ObjectPtr>& objects,
                                      Eigen::Matrix4d v2c, std::string name,
                                      int offset_x, int offset_y,
                                      int image_width, int image_height) {
  if (name == "radar") {
    // LOG(INFO)<<objects.size();
    for (auto obj : objects) {
      Eigen::Vector3d center = obj->center;
      Eigen::Vector2d center2d;
      get_project_point(v2c, center, &center2d);
      if ((center2d[0] > image_width) || (center2d[1] > image_height) ||
          (center2d[0] < 0) || (center2d[1] < 0)) {
        continue;
      }
      float x = offset_x + 1.0 * center2d[0] * image_width_ / image_width;
      float y = offset_y + 1.0 * center2d[1] * image_height_ / image_height;
      float radius = 5.0 * image_height_ / image_height;
      float x1 = x - radius;
      float x2 = x + radius;
      float y1 = y - radius;
      float y2 = y + radius;

      if (obj->b_cipv) {
        AINFO << "draw_objects2d This is CIPV, obj->track_id: "
              << obj->track_id;
        glColor3ub(255, 0, 0);
      } else {
        glColor3ub(0, 0, 0);
      }

      glLineWidth(4);
      glBegin(GL_LINES);
      glVertex2i(x1, y1);
      glVertex2i(x1, y2);

      glVertex2i(x1, y2);
      glVertex2i(x2, y2);

      glVertex2i(x2, y2);
      glVertex2i(x2, y1);

      glVertex2i(x2, y1);
      glVertex2i(x1, y1);
      glEnd();
    }
    glLineWidth(1);

    return;
  }
}

void GLFWFusionViewer::draw_8pts_box(const std::vector<Eigen::Vector2d>& points,
                                     const Eigen::Vector3f& color, int offset_x,
                                     int offset_y, int image_width,
                                     int image_height) {
  if (points.size() != 8) {
    return;
  }
  Eigen::Vector2d p1 = points[0];
  Eigen::Vector2d p2 = points[1];
  Eigen::Vector2d p3 = points[2];
  Eigen::Vector2d p4 = points[3];
  Eigen::Vector2d p5 = points[4];
  Eigen::Vector2d p6 = points[5];
  Eigen::Vector2d p7 = points[6];
  Eigen::Vector2d p8 = points[7];

  int color_bottom[3] = {128, 32, 32};  // bottom edges' color
  int color_top[3] = {0, 255, 155};     // top edges' color
  int color_side[3] = {0, 255, 55};     // side edges' color

  for (size_t i = 0; i < 3; ++i) {
    color_top[i] = color[i];
    color_side[i] = color[i];
  }

  draw_line2d(p1, p2, 2, color_bottom[0], color_bottom[1], color_bottom[2],
              offset_x, offset_y, image_width, image_height);
  draw_line2d(p2, p3, 2, color_bottom[0], color_bottom[1], color_bottom[2],
              offset_x, offset_y, image_width, image_height);
  draw_line2d(p3, p4, 2, color_bottom[0], color_bottom[1], color_bottom[2],
              offset_x, offset_y, image_width, image_height);
  draw_line2d(p4, p1, 2, color_bottom[0], color_bottom[1], color_bottom[2],
              offset_x, offset_y, image_width, image_height);

  draw_line2d(p5, p6, 2, color_top[0], color_top[1], color_top[2], offset_x,
              offset_y, image_width, image_height);
  draw_line2d(p6, p7, 2, color_top[0], color_top[1], color_top[2], offset_x,
              offset_y, image_width, image_height);
  draw_line2d(p7, p8, 2, color_top[0], color_top[1], color_top[2], offset_x,
              offset_y, image_width, image_height);
  draw_line2d(p8, p5, 2, color_top[0], color_top[1], color_top[2], offset_x,
              offset_y, image_width, image_height);

  draw_line2d(p5, p1, 2, color_side[0], color_side[1], color_side[2], offset_x,
              offset_y, image_width, image_height);
  draw_line2d(p3, p7, 2, color_side[0], color_side[1], color_side[2], offset_x,
              offset_y, image_width, image_height);
  draw_line2d(p4, p8, 2, color_side[0], color_side[1], color_side[2], offset_x,
              offset_y, image_width, image_height);
  draw_line2d(p6, p2, 2, color_side[0], color_side[1], color_side[2], offset_x,
              offset_y, image_width, image_height);
}

}  // namespace lowcostvisualizer
}  // namespace perception
}  // namespace apollo
