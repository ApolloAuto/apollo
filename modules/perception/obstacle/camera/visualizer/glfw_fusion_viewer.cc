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

#include <fstream>
#include <iomanip>
#include <iostream>
#include <cfloat>
#include <sstream>
#include <string>
#include <vector>
#include <map>

#include "modules/perception/lib/config_manager/calibration_config_manager.h"
#include "modules/perception/obstacle/camera/visualizer/common/bmp.h"
#include "modules/perception/obstacle/camera/visualizer/common/gl_raster_text.h"
#include "modules/perception/obstacle/camera/visualizer/frame_content.h"

namespace apollo {
namespace perception {
namespace lowcostvisualizer {

using ::apollo::perception::CalibrationConfigManager;
using ::apollo::perception::CameraCalibrationPtr;

const double pace_zoom = 15;
const double My_PI = 3.14159265359;

std::vector<std::vector<int>> GLFWFusionViewer::s_color_table = {
    std::vector<int>{0, 0, 128},   std::vector<int>{0, 0, 255},
    std::vector<int>{0, 128, 255}, std::vector<int>{0, 128, 128},
    std::vector<int>{0, 128, 0},   std::vector<int>{0, 255, 0},
    std::vector<int>{0, 255, 128}, std::vector<int>{0, 255, 255},
    std::vector<int>{255, 0, 0},   std::vector<int>{255, 0, 128},
    std::vector<int>{255, 0, 255}, std::vector<int>{255, 255, 0},
    std::vector<int>{255, 128, 0}};

GLFWFusionViewer::GLFWFusionViewer()
    : _init(false),
      _window(NULL),
      _pers_camera(NULL),
      _bg_color(0.0, 0.0, 0.0),
      _win_width(2560),
      _win_height(1440),
      _mouse_prev_x(0),
      _mouse_prev_y(0),
      _frame_content(NULL),
      _rgba_buffer(NULL),
      _vao_trans_x(0.0),
      _vao_trans_y(0.0),
      _vao_trans_z(0.0),
      _Rotate_x(0.0),
      _Rotate_y(0.0),
      _Rotate_z(0.0),
      show_box(1),
      show_velocity(1),
      show_text(0),
      _capture_screen(false),
      _capture_video(FLAGS_capture_screen),
      _scene_width(1280),
      _scene_height(720),
      _image_width(1280),
      _image_height(720),
      _frame_count(0) {
  _mode_mat = Eigen::Matrix4d::Identity();
}

GLFWFusionViewer::~GLFWFusionViewer() {
  close();
  if (_pers_camera) {
    delete _pers_camera;
  }
  if (_rgba_buffer) {
    delete[] _rgba_buffer;
    _rgba_buffer = nullptr;
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
  if (_init) {
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
  _show_radar_pc = false;
  _show_fusion_pc = false;
  _show_associate_color = false;
  _show_type_id_label = true;

  CalibrationConfigManager* config_manager =
      Singleton<CalibrationConfigManager>::get();
  CameraCalibrationPtr calibrator = config_manager->get_camera_calibration();
  _camera_intrinsic = calibrator->get_camera_intrinsic();
  _distort_camera_intrinsic = calibrator->get_camera_model();

  // Init Raster Text
  _raster_text = std::make_shared<GLRasterText>();
  _raster_text->init();

  _init = true;
  return true;
}

void GLFWFusionViewer::spin() {
  while (!glfwWindowShouldClose(_window) && _frame_content) {
    glfwPollEvents();
    render();
    glfwSwapBuffers(_window);
  }
  glfwDestroyWindow(_window);
}

void GLFWFusionViewer::spin_once() {
  if (!_frame_content) {  // if _frame_content may be always guarantteed, remove
                          // this line.
    return;
  }

  glfwPollEvents();
  render();
  glfwSwapBuffers(_window);
}

void GLFWFusionViewer::close() {
  glfwTerminate();
}

void GLFWFusionViewer::set_camera_para(Eigen::Vector3d i_position,
                                       Eigen::Vector3d i_scn_center,
                                       Eigen::Vector3d i_up_vector) {
  _pers_camera->set_position(i_position);
  _pers_camera->setscene_center(i_scn_center);
  _pers_camera->setup_vector(i_up_vector);
  _pers_camera->look_at(i_scn_center);

  GLdouble v_mat[16];
  _pers_camera->get_model_view_matrix(v_mat);
  _view_mat << v_mat[0], v_mat[4], v_mat[8], v_mat[12], v_mat[1], v_mat[5],
      v_mat[9], v_mat[13], v_mat[2], v_mat[6], v_mat[10], v_mat[14], v_mat[3],
      v_mat[7], v_mat[11], v_mat[15];
}

bool GLFWFusionViewer::window_init() {
  if (!glfwInit()) {
    std::cerr << "Failed to initialize glfw !\n";
    return false;
  }

  // _window = glfwCreateWindow(_win_width, _win_height, "opengl_visualizer",
  // nullptr, nullptr);
  // glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);
  _win_width = _scene_width + _image_width;
  _win_height =
      (_image_height * 2 > _win_height) ? _image_height * 2 : _win_height;
  _window = glfwCreateWindow(_win_width, _win_height, "gl_camera_visualizer",
                             nullptr, nullptr);
  if (_window == nullptr) {
    std::cerr << "Failed to create glfw window!\n";
    glfwTerminate();
    return false;
  }

  glfwMakeContextCurrent(_window);
  glfwSwapInterval(1);
  glfwSetWindowUserPointer(_window, this);

  // set callback functions
  glfwSetFramebufferSizeCallback(_window, framebuffer_size_callback);

  glfwSetKeyCallback(_window, key_callback);
  glfwSetMouseButtonCallback(_window, mouse_button_callback);
  glfwSetCursorPosCallback(_window, mouse_cursor_position_callback);
  glfwSetScrollCallback(_window, mouse_scroll_callback);

  glfwShowWindow(_window);
  return true;
}

bool GLFWFusionViewer::camera_init() {
  // perspective cameras
  _pers_camera = new Camera;
  _pers_camera->set_type(Camera::Type::PERSPECTIVE);
  _pers_camera->setscene_radius(1000);
  _pers_camera->set_position(Eigen::Vector3d(0, 0, -30));
  _pers_camera->setscreen_widthandheight(_scene_width, _scene_height);
  _pers_camera->look_at(Eigen::Vector3d(0, 0, 0));
  double fov = 45 * (My_PI / 180.0);
  _pers_camera->setfield_of_view(fov);
  return true;
}

bool GLFWFusionViewer::opengl_init() {
  glViewport(0, 0, _scene_width, _scene_height);
  glClearColor(_bg_color(0), _bg_color(1), _bg_color(2), 0.0);
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
  _pers_camera->load_projection_matrix();

  // column major
  GLdouble mode_mat[16] = {
      _mode_mat(0, 0), _mode_mat(1, 0), _mode_mat(2, 0), _mode_mat(3, 0),
      _mode_mat(0, 1), _mode_mat(1, 1), _mode_mat(2, 1), _mode_mat(3, 1),
      _mode_mat(0, 2), _mode_mat(1, 2), _mode_mat(2, 2), _mode_mat(3, 2),
      _mode_mat(0, 3), _mode_mat(1, 3), _mode_mat(2, 3), _mode_mat(3, 3)};
  GLdouble view_mat[16] = {
      _view_mat(0, 0), _view_mat(1, 0), _view_mat(2, 0), _view_mat(3, 0),
      _view_mat(0, 1), _view_mat(1, 1), _view_mat(2, 1), _view_mat(3, 1),
      _view_mat(0, 2), _view_mat(1, 2), _view_mat(2, 2), _view_mat(3, 2),
      _view_mat(0, 3), _view_mat(1, 3), _view_mat(2, 3), _view_mat(3, 3)};
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
  if (abs(cos_direction_velocity) > 0.707)  {  // <45 degree
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

  _frame_count++;

  // 1. Bottom left, draw 3d detection and classification results (lidar tracked
  // objects), and lanes in ego-car ground space
  glViewport(0, 0, _image_width, _image_height);
  {
    pre_draw();
    glPushMatrix();
    glTranslatef(_vao_trans_y, _vao_trans_x, _vao_trans_z);
    glRotatef(_Rotate_x, 1, 0, 0);
    glRotatef(_Rotate_y, 0, 1, 0);
    bool show_fusion = false;
    draw_3d_classifications(_frame_content, show_fusion);
    draw_car_forward_dir();
    glPopMatrix();
  }

  glViewport(0, 0, _image_width, _image_height);
  glPushMatrix();
  glTranslatef(_vao_trans_y, _vao_trans_x, _vao_trans_z);
  glRotatef(_Rotate_x, 1, 0, 0);
  glRotatef(_Rotate_y, 0, 1, 0);
  glPopMatrix();

  // 2. Bottom right
  glViewport(_scene_width, 0, _image_width, _image_height);
  glPushMatrix();
  glTranslatef(_vao_trans_y, _vao_trans_x, _vao_trans_z);
  glRotatef(_Rotate_x, 1, 0, 0);
  glRotatef(_Rotate_y, 0, 1, 0);
  glPopMatrix();

  // 3. Top left, draw 2d camera detection and classification results
  glViewport(0, _scene_height, _image_width, _image_height);
  draw_camera_frame(_frame_content, false);

  // 4. Top right, draw 2d detection and 3d classification results
  glViewport(_scene_width, _scene_height, _image_width, _image_height);
  draw_camera_frame(_frame_content);

  static int no_frame = 0;
  if (_capture_video || _capture_screen) {
    double time_stamp = _frame_content->get_visualization_timestamp();
    char buffer[512];
    snprintf(buffer, sizeof(time_stamp), "./%.12f.bmp", time_stamp);
    std::string file_name = FLAGS_screen_output_dir + buffer;
    capture_screen(file_name);
    if (_capture_screen) {
      _capture_screen = false;
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
  if (width == _win_width && height == _win_height) {
    return;
  }
  if (_rgba_buffer != nullptr) {
    delete[] _rgba_buffer;
    _rgba_buffer = nullptr;
  }
  _rgba_buffer = new unsigned char[width * height * 4];
  _win_width = width;
  _win_height = height;
  _scene_width = _win_width * 0.5;
  _scene_height = _win_height * 0.5;
  _image_width = _scene_width;
  _image_height = _scene_height;
  _pers_camera->setscreen_widthandheight(_scene_width, _scene_height);
}

void GLFWFusionViewer::resize_framebuffer(int width, int height) {
  if (width == _win_width && height == _win_height) {
    return;
  }
  if (_rgba_buffer != nullptr) {
    delete[] _rgba_buffer;
    _rgba_buffer = nullptr;
  }
  _rgba_buffer = new unsigned char[width * height * 4];
  _win_width = width;
  _win_height = height;
  _scene_width = _win_width * 0.5;
  _scene_height = _win_height * 0.5;
  _image_width = _scene_width;
  _image_height = _scene_height;
  _pers_camera->setscreen_widthandheight(_scene_width, _scene_height);
}

void GLFWFusionViewer::mouse_move(double xpos, double ypos) {
  int state_left = glfwGetMouseButton(_window, GLFW_MOUSE_BUTTON_LEFT);
  int state_right = glfwGetMouseButton(_window, GLFW_MOUSE_BUTTON_RIGHT);
  int x_delta = xpos - _mouse_prev_x;
  int y_delta = ypos - _mouse_prev_y;
  if (state_left == GLFW_PRESS) {
    Eigen::Quaterniond rot =
        _pers_camera->get_rotatation_by_mouse_from_qgwidget(
            _mouse_prev_x, _mouse_prev_y, xpos, ypos);
    Eigen::Matrix3d rot_mat = rot.inverse().toRotationMatrix();
    Eigen::Vector3d scn_center = _pers_camera->scene_center();
    Eigen::Vector4d scn_center_(scn_center(0), scn_center(1), scn_center(2), 1);
    scn_center_ = _mode_mat * _view_mat * scn_center_;
    scn_center = scn_center_.head(3);
    Eigen::Vector3d r_multi_scn_center = rot_mat * scn_center;
    Eigen::Vector3d t = scn_center - r_multi_scn_center;
    Eigen::Matrix4d cur_mat = Eigen::Matrix4d::Identity();
    cur_mat.topLeftCorner(3, 3) = rot_mat;
    cur_mat.topRightCorner(3, 1) = t;
    _mode_mat = cur_mat * _mode_mat;
  } else if (state_right == GLFW_PRESS) {
    _mode_mat(0, 3) += 0.1 * x_delta;
    _mode_mat(1, 3) -= 0.1 * y_delta;
  }

  _mouse_prev_x = xpos;
  _mouse_prev_y = ypos;
}

void GLFWFusionViewer::mouse_wheel(double delta) {
  _mode_mat(2, 3) -= delta;
}

void GLFWFusionViewer::reset() {
  _mode_mat = Eigen::Matrix4d::Identity();
}

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
      _use_class_color = !_use_class_color;
      break;
    case 83:  // 'S'
      _capture_screen = true;
      break;
    case 65:  // 'A'
      _capture_video = !_capture_video;
      break;
    // for camera visualization
    case GLFW_KEY_I:
      _show_type_id_label = !_show_type_id_label;
      break;
    case GLFW_KEY_Q:  // Q
      _show_lane = !_show_lane;
      break;
    case GLFW_KEY_E:  // E
      _draw_lane_objects = !_draw_lane_objects;
    case GLFW_KEY_F:  // F
      _show_fusion_pc = !_show_fusion_pc;
      break;
    case GLFW_KEY_D:  // D
      _show_radar_pc = !_show_radar_pc;
      break;
    case GLFW_KEY_2:  // 2
      _show_camera_box2d = !_show_camera_box2d;
      break;
    case GLFW_KEY_3:  // 3
      _show_camera_box3d = !_show_camera_box3d;
      break;
    case GLFW_KEY_0:  // 3
      _show_associate_color = !_show_associate_color;
    default:
      break;
  }
}

void GLFWFusionViewer::capture_screen(const std::string& file_name) {
  if (_rgba_buffer == nullptr) {
    _rgba_buffer = new unsigned char[4 * _win_width * _win_height];
    if (_rgba_buffer == nullptr) {
      AERROR << "Failed to create screen capture buffer \n";
      return;
    }
  }
  glReadPixels(0, 0, _win_width, _win_height, GL_BGRA, GL_UNSIGNED_BYTE,
               _rgba_buffer);

  save_rgba_image_to_bmp<unsigned char>(_rgba_buffer, _win_width, _win_height,
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
  cv::Mat image_mat_src = content->get_camera_image().clone();
  if (image_mat_src.empty()) {
    return;
  }
  int image_width = image_mat_src.cols;
  int image_height = image_mat_src.rows;

  glMatrixMode(GL_PROJECTION);  // Operate on projection matrix
  glLoadIdentity();
  glOrtho(_scene_width, _scene_width + _image_width, _image_height, 0.0, 0.0,
          100.0);

  glMatrixMode(GL_MODELVIEW);  // Operate on model-view matrix
  glLoadIdentity();

  glEnable(GL_TEXTURE_2D);
  GLuint image_tex = image_to_gl_texture(image_mat_src, GL_LINEAR_MIPMAP_LINEAR,
                                         GL_LINEAR, GL_CLAMP);

  /* Draw a quad */
  glBegin(GL_QUADS);
  glTexCoord2i(0, 0);
  glVertex2i(_scene_width, 0);
  glTexCoord2i(0, 1);
  glVertex2i(_scene_width, _image_height);
  glTexCoord2i(1, 1);
  glVertex2i(_scene_width + _image_width, _image_height);
  glTexCoord2i(1, 0);
  glVertex2i(_scene_width + _image_width, 0);
  glEnd();

  glDeleteTextures(1, &image_tex);
  glDisable(GL_TEXTURE_2D);

  // -----------------------------
  Eigen::Matrix4d camera_to_world_pose = content->get_camera_to_world_pose();

  Eigen::Matrix4d v2c = camera_to_world_pose.inverse();

  int offset_x = _scene_width;
  int offset_y = 0;
  if (_show_camera_box2d || _show_camera_box3d) {
    std::vector<ObjectPtr> camera_objects;
    camera_objects = content->get_camera_objects();
    draw_camera_box(camera_objects, v2c, offset_x, offset_y, image_width,
                    image_height);
  }

  if (_show_fusion_pc) {
    std::vector<ObjectPtr> objects;
    objects = content->get_fused_objects();
    draw_objects2d(objects, v2c, "fusion", offset_x, offset_y, image_width,
                   image_height);
  }

  if (_show_radar_pc) {
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
  glOrtho(_scene_width, _scene_width + _image_width, _image_height, 0.0, 0.0,
          100.0);

  glMatrixMode(GL_MODELVIEW);  // Operate on model-view matrix
  glLoadIdentity();

  glEnable(GL_TEXTURE_2D);
  GLuint image_tex = image_to_gl_texture(image_mat_src, GL_LINEAR_MIPMAP_LINEAR,
                                         GL_LINEAR, GL_CLAMP);

  /* Draw a quad */
  glBegin(GL_QUADS);
  glTexCoord2i(0, 0);
  glVertex2i(_scene_width, 0);
  glTexCoord2i(0, 1);
  glVertex2i(_scene_width, _image_height);
  glTexCoord2i(1, 1);
  glVertex2i(_scene_width + _image_width, _image_height);
  glTexCoord2i(1, 0);
  glVertex2i(_scene_width + _image_width, 0);
  glEnd();

  glDeleteTextures(1, &image_tex);
  glDisable(GL_TEXTURE_2D);

  // -----------------------------
  Eigen::Matrix4d camera_to_world_pose = content->get_camera_to_world_pose();

  Eigen::Matrix4d v2c = camera_to_world_pose.inverse();

  int offset_x = _scene_width;
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

  pc2d_mat = _distort_camera_intrinsic->project(pc3d_mat);
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
        _camera_intrinsic.block(0, 0, 3, 3) * (center + r * obj_points[i]);
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
  Eigen::Vector3d pv = _camera_intrinsic.block(0, 0, 3, 3) * (pc3d);
  *p2d = pv.head(2) / pv.z();
  return true;
}

void GLFWFusionViewer::draw_line2d(const Eigen::Vector2d& p1,
                                   const Eigen::Vector2d& p2, int line_width,
                                   int r, int g, int b, int offset_x,
                                   int offset_y, int image_width,
                                   int image_height) {
  double x1 = offset_x + 1.0 * p1.x() * _image_width / image_width;
  double y1 = offset_y + 1.0 * p1.y() * _image_height / image_height;
  double x2 = offset_x + 1.0 * p2.x() * _image_width / image_width;
  double y2 = offset_y + 1.0 * p2.y() * _image_height / image_height;

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
    Eigen::Vector3d tc =
        (v2c * Eigen::Vector4d(center[0], center[1], center[2], 1)).head(3);

    tc = center.head(3);
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

        if (_use_class_color) {
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
        if (_show_type_id_label) {
          std::string c =
              std::to_string(static_cast<int>(
                             (sqrt(tc.x() * tc.x() + tc.z() * tc.z()))));
          glColor3ub(box2d_color[0], box2d_color[1], box2d_color[2]);
          double x_txt = points[7].x();
          double y_txt = points[7].y() - 8;
          int xi_txt = offset_x + x_txt * _image_width / image_width;
          int yi_txt = offset_y + y_txt * _image_height / image_height;

          glRasterPos2i(xi_txt, yi_txt);
          _raster_text->print_string(c.c_str());
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
          int xi_type = offset_x + x_type * _image_width / image_width;
          int yi_type = offset_y + y_type * _image_height / image_height;

          glRasterPos2i(xi_type, yi_type);
          _raster_text->print_string(obj_type_str.c_str());
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
  double x1 = offset_x + 1.0 * p1.x() * _image_width / image_width;
  double y1 = offset_y + 1.0 * p1.y() * _image_height / image_height;
  double x2 = offset_x + 1.0 * p2.x() * _image_width / image_width;
  double y2 = offset_y + 1.0 * p2.y() * _image_height / image_height;

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
  Eigen::Vector3d center = _pers_camera->scene_center();
  Eigen::Vector3d forward_vp = center + _forward_dir * 5;
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
  if (_show_associate_color) {
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
      glVertex3f(_main_car[index][0], _main_car[index][1], _main_car[index][2]);
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
      _raster_text->print_string(std::to_string(objects[i]->track_id).c_str());

      if (objects[i]->b_cipv) {
        glRasterPos2i(tc[0] + 3, tc[1]);
        _raster_text->print_string(std::string("cipv").c_str());
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
                 _use_class_color);
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
                 _use_class_color);
  }

  if (FLAGS_show_fusion_association) {
    draw_fusion_association(content);
  }
  return true;
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
                   _use_class_color);
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
        draw_objects(objects, c2v, draw_cube, draw_velocity, cam_color,
                     _use_class_color);
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

    continue;
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
        std::string c =
            std::to_string(static_cast<int>(
                           (sqrt(tc.x() * tc.x() + tc.z() * tc.z()))));
        auto dis_txt_color =
            s_color_table[obj->track_id % s_color_table.size()];
        glColor3ub(dis_txt_color[0], dis_txt_color[1], dis_txt_color[2]);
        double x_txt = points[7].x();
        double y_txt = points[7].y() - 8;
        int xi_txt = offset_x + x_txt * _image_width / image_width;
        int yi_txt = offset_y + y_txt * _image_height / image_height;

        glRasterPos2i(xi_txt, yi_txt);
        _raster_text->print_string(c.c_str());
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
        int xi_type = offset_x + x_type * _image_width / image_width;
        int yi_type = offset_y + y_type * _image_height / image_height;

        glRasterPos2i(xi_type, yi_type);
        _raster_text->print_string(obj_type_str.c_str());
        glColor4f(1.0f, 1.0f, 1.0f, 1.0f);  // reset the color to white
      }
    }
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
      float x = offset_x + 1.0 * center2d[0] * _image_width / image_width;
      float y = offset_y + 1.0 * center2d[1] * _image_height / image_height;
      float radius = 5.0 * _image_height / image_height;
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
