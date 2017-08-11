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

class GLFWViewer{
public:
    explicit GLFWViewer();
    virtual ~GLFWViewer();

    bool Initialize();
 
    void SetFrameContent(FrameContent* frame_content){ frame_content_ = frame_content;}
    void Spin();
    void SpinOnce();
    void Close();

    void SetBackgroundColor(Eigen::Vector3d i_bg_color) { bg_color_ = i_bg_color;}
    void SetSize(int w, int h);
    void SetCameraPara(Eigen::Vector3d i_position, Eigen::Vector3d i_scn_center, Eigen::Vector3d i_up_vector);
    void SetForwardDir(Eigen::Vector3d forward){ forward_dir_ = forward;}
    
    //callback assistants
    void ResizeFramebuffer(int width, int height);
    void MouseMove(double xpos, double ypos);
    void MouseWheel(double delta);
    void Reset();
    void Keyboard(int key);

    //callback functions
    static void FramebufferSizeCallback(GLFWwindow* window, int width, int height);
    //input related
    static void KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);
    static void MouseButtonCallback(GLFWwindow* window, int button, int action, int mods);
    static void MouseCursorPositionCallback(GLFWwindow* window, double xpos, double ypos);
    static void MouseScrollCallback(GLFWwindow* window, double xoffset, double yoffset);
    //error handling
    static void ErrorCallback(int error, const char* description);

private:
    bool WindowInit();
    bool CameraInit();
    bool OpenglInit();
    void PreDraw();
    void Render();
private:
    bool init_;

    GLFWwindow* window_;
    Camera* pers_camera_; 
	
    Eigen::Vector3d forward_dir_;

    int win_width_;
    int win_height_;
    int mouse_prev_x_;
    int mouse_prev_y_;
    Eigen::Matrix4d mode_mat_;
    Eigen::Matrix4d view_mat_;
    Eigen::Vector3d bg_color_;
 
    /***************************************************************************************/
    bool show_cloud_;
    int  show_cloud_state_;
    bool show_box_;
    bool show_velocity_;
    bool show_polygon_;
    bool show_text_;
    
    FrameContent* frame_content_;
    /***************************************************************************************/
    void GetClassColor(int cls, float rgb[3]);
    enum { circle, cube, cloud, polygon,  NumVAOs_typs };   //{0, 1, 2, 3, 4}
    enum { vertices, colors, elements, NumVBOs };   //{0, 1, 2, 3}
         
    //cloud
    static const int VAO_cloud_num = 35;
    static const int VBO_cloud_num = 10000;
    GLuint   VAO_cloud[VAO_cloud_num];
    GLuint   buffers_cloud[VAO_cloud_num][NumVBOs];
    GLfloat  cloudVerts[VBO_cloud_num][3]; 
    bool DrawCloud(FrameContent* content);
    //circle
    static const int VAO_circle_num = 6;
    static const int VBO_circle_num = 256;
    GLuint VAO_circle[VAO_circle_num];
    void DrawCircle();
    void DrawCarForwardDir();
    //objects
    void DrawObjects(FrameContent* content , bool draw_cube, bool draw_polygon, bool draw_velocity);
    vec3 GetVelocitySrcPosition(FrameContent* content, int id);
    //map_roi 
    //bool show_map(FrameContent* content, bool show_map_roi, bool show_map_boundary);
};

}  // namespace obstacle
}  // namespace perception
#endif
