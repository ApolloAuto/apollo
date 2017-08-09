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

#include "glfw_viewer.h"

#include <iomanip>
#include <fstream>
#include <iostream>
#include <sstream>
#include "modules/common/log.h"

#include <pcl/io/pcd_io.h>

#include "frame_content.h"
#include "modules/perception/obstacle/base/object.h"
//#include "obstacle/visualizer/pcl_render_util.h"

namespace apollo {
namespace perception {


const double My_PI = 3.14159265359;

GLFWViewer::GLFWViewer() :
_init(false),
_window(NULL),
_pers_camera(NULL),
_bg_color(0.0,0.0,0.0),
_win_width(800),
_win_height(600),
_mouse_prev_x(0),
_mouse_prev_y(0),
_frame_content(NULL),
show_cloud(1),
_show_cloud_state(0),
show_box(1),
show_velocity(1),
show_polygon(0),
show_text(0){
    _mode_mat = Eigen::Matrix4d::Identity();
}

GLFWViewer::~GLFWViewer(){
    close();
    if (_pers_camera)
        delete _pers_camera;
}
void GLFWViewer:: get_class_color(int cls, float rgb[3])
{
 	switch (cls) {
        case 0:
            rgb[0] = 0.5; rgb[1] = 0; rgb[2] = 1; //紫
            break;
        case 1:
            rgb[0] = 0; rgb[1] = 1; rgb[2] = 1; //青
            break;
        case 2:
             rgb[0] = 1; rgb[1] = 1; rgb[2] = 0; //黄
            break;
        case 3:
             rgb[0] = 1; rgb[1] = 0.5; rgb[2] = 0.5;  //赤
            break;
        case 4:
             rgb[0] = 0; rgb[1] = 0; rgb[2] = 1;  //蓝
            break;
        case 5:
             rgb[0] = 0; rgb[1] = 1; rgb[2] = 0;  //绿
            break;
        case 6:
             rgb[0] = 1; rgb[1] = 0.5; rgb[2] = 0;  //橙
            break;
        case 7:
             rgb[0] = 1; rgb[1] = 0; rgb[2] = 0;  //赤
            break;
    }
}
bool GLFWViewer::initialize(){
	AINFO << "GLFWViewer::initialize()"<<std::endl;
    if (_init) {
        AINFO <<" GLFWViewer is already initialized !"<<std::endl;
        return false;
    }

    if (!window_init()){
        AINFO << " Failed to initialize the window !"<<std::endl;
        return false;
    }

    if (!camera_init()){
        AINFO << " Failed to initialize the camera !"<<std::endl;
        return false;
    }

    if (!opengl_init()){
        AINFO << " Failed to initialize opengl !"<<std::endl;
        return false;
    }

    _init = true;

   	show_cloud = 1;
    show_box = 1;
    show_velocity = 1;
    show_polygon = 0;
    show_text = 1;
    return true;
}

void GLFWViewer::spin(){
    while (!glfwWindowShouldClose(_window) ) {//&& _frame_content){
        glfwPollEvents();
        render();
        glfwSwapBuffers(_window);
    }
    glfwDestroyWindow(_window);
}

void GLFWViewer::spin_once(){
    if (!_frame_content) // if _frame_content may be always guarantteed, remove this line.
        return;
    
    glfwPollEvents();
    render();
    glfwSwapBuffers(_window);
}

void GLFWViewer::close(){
    glfwTerminate();
}

void GLFWViewer::set_size(int w, int h){
    _win_width = w;
    _win_height = h;
}

void GLFWViewer::set_camera_para(Eigen::Vector3d i_position,
                                 Eigen::Vector3d i_scn_center,
                                 Eigen::Vector3d i_up_vector){
    _pers_camera->set_position(i_position);
    _pers_camera->setscene_center(i_scn_center);
    _pers_camera->set_revolve_around_point(i_scn_center);
    _pers_camera->setup_vector(i_up_vector);
    _pers_camera->look_at(i_scn_center); 

    GLdouble v_mat[16];
    _pers_camera->get_model_view_matrix(v_mat);
    _view_mat<<v_mat[0],v_mat[4],v_mat[8], v_mat[12],
               v_mat[1],v_mat[5],v_mat[9], v_mat[13],
               v_mat[2],v_mat[6],v_mat[10],v_mat[14],
               v_mat[3],v_mat[7],v_mat[11],v_mat[15];
}

bool GLFWViewer::window_init(){
    if (!glfwInit()){
        std::cerr << "Failed to initialize glfw !\n";
        return false;
    }

    _window = glfwCreateWindow(_win_width, _win_height, "opengl_visualizer", nullptr, nullptr);
    if (_window == nullptr){
        std::cerr << "Failed to create glfw window!\n";
        glfwTerminate();
        return false;
    }

    glfwMakeContextCurrent(_window);
    glfwSwapInterval(1);
    glfwSetWindowUserPointer(_window, this);

    //set callback functions
    glfwSetFramebufferSizeCallback(_window, framebuffer_size_callback);

    glfwSetKeyCallback(_window, key_callback);
    glfwSetMouseButtonCallback(_window, mouse_button_callback);
    glfwSetCursorPosCallback(_window, mouse_cursor_position_callback);
    glfwSetScrollCallback(_window, mouse_scroll_callback);

    glfwShowWindow(_window);
    return true;
}

bool GLFWViewer::camera_init(){
    //perspective cameras
    _pers_camera = new Camera;
    _pers_camera->set_type(Camera::Type::PERSPECTIVE);
    _pers_camera->setscene_radius(1000);
    _pers_camera->set_position(Eigen::Vector3d(0,0,-30));
    _pers_camera->setscreen_widthandheight(_win_width, _win_height);
    _pers_camera->look_at(Eigen::Vector3d(0,0,0));
    double fov = 45*(My_PI/180.0);
    _pers_camera->setfield_of_view(fov);

    return true;
}

bool GLFWViewer::opengl_init(){
    glClearColor(_bg_color(0), _bg_color(1), _bg_color(2), 0.0);
    glClearDepth(1.0f);
    glShadeModel(GL_SMOOTH);
    glDepthFunc(GL_LEQUAL);
    //lighting
    GLfloat mat_shininess[]={20.0};
    GLfloat light_position[]={1.0,-1.0,1.0,0.0};
    GLfloat lmodel_ambient[]={.5,.5,.5,1.0};
    glMaterialfv(GL_FRONT,GL_SHININESS,mat_shininess);
    glLightfv(GL_LIGHT0,GL_POSITION,light_position);
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT,lmodel_ambient);
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
    glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHTING);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_NORMALIZE);
    glEnable(GL_COLOR_MATERIAL);
    /********************************************************************************************************************************************/
    GLenum err = glewInit();
    if (GLEW_OK != err)
    {
	fprintf(stderr, "GLEW初始化失败！\n");
	exit(EXIT_FAILURE);
     }
    /*********************************************************   gen cloud vao & vbo   **********************************************************/
    {

	int i=0;

        GLfloat  cloudColors[VBO_cloud_num][3];
        GLuint  cloudIndices[VBO_cloud_num];
	for (i = 0; i<VBO_cloud_num; i++)
	{
	    cloudColors[i][0] = 0.7;
	    cloudColors[i][1] = 0.7;
	    cloudColors[i][2] = 0.7;
	    cloudIndices[i] = (GLuint)i;
	}
	glGenVertexArrays(VAO_cloud_num, VAO_cloud);
	for (i = 0; i < VAO_cloud_num; i++){
		glBindVertexArray(VAO_cloud[i]);
		//修改缓冲对象里的数据
		glGenBuffers(NumVBOs, buffers_cloud[i]);
		glBindBuffer(GL_ARRAY_BUFFER, buffers_cloud[i][vertices]);
		glBufferData(GL_ARRAY_BUFFER, sizeof(cloudVerts), cloudVerts, GL_STREAM_DRAW);
		glVertexPointer(3, GL_FLOAT, 0, BUFFER_OFFSET(0));
		glEnableClientState(GL_VERTEX_ARRAY);

		glBindBuffer(GL_ARRAY_BUFFER, buffers_cloud[i][colors]);
		glBufferData(GL_ARRAY_BUFFER, sizeof(cloudColors), cloudColors, GL_STREAM_DRAW);
		glColorPointer(3, GL_FLOAT, 0, BUFFER_OFFSET(0));
		glEnableClientState(GL_COLOR_ARRAY);

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, buffers_cloud[i][elements]);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(cloudIndices), cloudIndices, GL_STREAM_DRAW);
	}

    }
    /*********************************************************   gen circle vao & vbo   ***************************************************************/
    {
	GLuint   buffers_circle[VAO_circle_num][NumVBOs];
        GLfloat  circleVerts[VBO_circle_num][3];
        GLfloat  circleColors[VBO_circle_num][3];
        GLuint   circleIndices[VBO_circle_num];

	float dTheta = 2*3.1415926f / (float)VBO_circle_num;
	int i = 0;
        int vao = 0;
	for (i = 0; i<VBO_circle_num; i++)
	{
		circleVerts[i][2] = -1.0;

		circleColors[i][0] = 0.0;
		circleColors[i][1] = 0.9;
		circleColors[i][2] = 0.9;

		circleIndices[i] = (GLuint)i;
	}



	glGenVertexArrays(VAO_circle_num, VAO_circle);

	for (vao = 0; vao < VAO_circle_num; vao++){

		for (i = 0; i<VBO_circle_num; i++)
		{
		    float theta = (float)i*dTheta;
		    circleVerts[i][0] = 20*(vao+1)*cos(theta);
	   	    circleVerts[i][1] = 20*(vao+1)*sin(theta);
		}
		glBindVertexArray(VAO_circle[vao]);
		glGenBuffers(NumVBOs, buffers_circle[vao]);
		glBindBuffer(GL_ARRAY_BUFFER, buffers_circle[vao][vertices]);
		glBufferData(GL_ARRAY_BUFFER, sizeof(circleVerts), circleVerts, GL_STATIC_DRAW);
		glVertexPointer(3, GL_FLOAT, 0, BUFFER_OFFSET(0));
		glEnableClientState(GL_VERTEX_ARRAY);

		glBindBuffer(GL_ARRAY_BUFFER, buffers_circle[vao][colors]);
		glBufferData(GL_ARRAY_BUFFER, sizeof(circleColors), circleColors, GL_STATIC_DRAW);
		glColorPointer(3, GL_FLOAT, 0, BUFFER_OFFSET(0));
		glEnableClientState(GL_COLOR_ARRAY);

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, buffers_circle[vao][elements]);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(circleIndices), circleIndices, GL_STATIC_DRAW);
	}

     }
/*********************************************************   gen cube vao & vbo   ***************************************************************/

	//    v6----- v5
	//   /|      /|
	//  v1------v0|
	//  | |     | |
	//  | |v7---|-|v4
	//  |/      |/
	//  v2------v3


    return true;
}

void GLFWViewer::pre_draw(){
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    _pers_camera->load_projection_matrix();
    
    //column major
    GLdouble mode_mat[16] = {_mode_mat(0, 0),_mode_mat(1, 0),_mode_mat(2, 0),_mode_mat(3,0),
                             _mode_mat(0, 1),_mode_mat(1, 1),_mode_mat(2, 1),_mode_mat(3,1),
                             _mode_mat(0, 2),_mode_mat(1, 2),_mode_mat(2, 2),_mode_mat(3,2),
                             _mode_mat(0, 3),_mode_mat(1, 3),_mode_mat(2, 3),_mode_mat(3,3)};
    GLdouble view_mat[16] = {_view_mat(0, 0),_view_mat(1, 0),_view_mat(2, 0),_view_mat(3,0),
                             _view_mat(0, 1),_view_mat(1, 1),_view_mat(2, 1),_view_mat(3,1),
                             _view_mat(0, 2),_view_mat(1, 2),_view_mat(2, 2),_view_mat(3,2),
                             _view_mat(0, 3),_view_mat(1, 3),_view_mat(2, 3),_view_mat(3,3)};
    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixd(mode_mat);
    glMultMatrixd(view_mat);

    GLfloat light_position[]={1.0,-1.0,1.0,0.0};
    glLightfv(GL_LIGHT0,GL_POSITION,light_position);
}

bool GLFWViewer::draw_cloud(FrameContent* content)
{
	pcl_util::PointCloudPtr cloud;
    pcl_util::PointCloudPtr roi_cloud;

    cloud = content->get_cloud();
    //roi_cloud = content->get_roi_cloud();

	//draw original point cloud
    if (cloud && !cloud->points.empty()) {
        glPointSize(1);
        size_t i = 0;
        int vao = 0;
        int vbo = 0;
        int vao_num = (cloud->points.size() / VBO_cloud_num) + 1;
        for (vao = 0; vao< vao_num; vao++) {
            for (vbo = 0; vbo < VBO_cloud_num; vbo++){
                cloudVerts[vbo][0] = cloud->points[i].x;
                cloudVerts[vbo][1] = cloud->points[i].y;
                cloudVerts[vbo][2] = cloud->points[i].z;
                i++;
                if (i >= cloud->points.size()) break;
            }
            glBindVertexArray(VAO_cloud[vao]);
            glBindBuffer(GL_ARRAY_BUFFER, buffers_cloud[vao][vertices]);
            glBufferSubData( GL_ARRAY_BUFFER, 0, sizeof(cloudVerts), cloudVerts);
            glDrawElements(GL_POINTS, vbo, GL_UNSIGNED_INT, BUFFER_OFFSET(0));
            glBindVertexArray(0);
            if (i >= cloud->points.size()) break;
        }
        if (i < cloud->points.size()) AINFO <<"vao*vbo num < cloud->points.size()";
    }

    //draw roi point cloud
    /*if (roi_cloud && !roi_cloud->points.empty()) {
        glPointSize(3);
        glColor3f(0,0.8,0);
        glBegin(GL_POINTS);
        for (const auto& point : roi_cloud->points){
            glVertex3f(point.x, point.y, point.z);
        }
        glEnd();
    }*/

    return true;
}

void GLFWViewer::draw_circle() {
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    Eigen::Matrix4d v2w_pose ;//= _frame_content->get_pose_v2w();
    GLdouble mat[16] = {v2w_pose(0, 0),v2w_pose(1, 0),v2w_pose(2, 0),v2w_pose(3,0),
                        v2w_pose(0, 1),v2w_pose(1, 1),v2w_pose(2, 1),v2w_pose(3,1),
                        v2w_pose(0, 2),v2w_pose(1, 2),v2w_pose(2, 2),v2w_pose(3,2),
                        v2w_pose(0, 3),v2w_pose(1, 3),v2w_pose(2, 3),v2w_pose(3,3)};
    glMultMatrixd(mat);
    int vao = 0;
    for (vao = 0; vao < VAO_circle_num; vao++) {
        glBindVertexArray(VAO_circle[vao]);
        glDrawElements(GL_LINE_LOOP, VBO_circle_num, GL_UNSIGNED_INT, BUFFER_OFFSET(0));
        glBindVertexArray(0);
    }
    glPopMatrix();
}

void GLFWViewer::draw_car_forward_dir() {
    glColor3f(1.0,0.5,0.17);
    glLineWidth(5);
    glBegin(GL_LINES);
    Eigen::Vector3d center = _pers_camera->scene_center();
    Eigen::Vector3d forward_vp = center + _forward_dir*10;
    glVertex3f(center(0), center(1), center(2));
    glVertex3f(forward_vp(0), forward_vp(1), forward_vp(2));
    glEnd();
    glLineWidth(1);
}

void GLFWViewer::draw_objects(FrameContent* content , bool draw_cube, bool draw_polygon, bool draw_velocity)
{
	std::vector<ObjectPtr> objects;
    std::vector<ObjectPtr> tracked_objects;

   	tracked_objects = content->get_tracked_objects();

    std::cout<<"tracked objects size "<<tracked_objects.size()<<std::endl;

    for (size_t i = 0; i < tracked_objects.size(); ++i) {
        objects.push_back(tracked_objects[i]);
    }

	float rgb[3];
	if (draw_cube){
        std::cout<<"draw cube enabled with objects size"<< objects.size()<<std::endl;
		float verts[8][3];
		vec3 center;//x,y,z
		vec3 direction;//x,y,z
		vec3 size;//len wid hei
		int indices[16] = {0,1,2,3,4,5,6,7,4,3,0,5,6,1,2,7};
		for (size_t i = 0; i < objects.size(); i++)
		{
			center.x = objects[i]->center[0];        
            center.y = objects[i]->center[1];        
            center.z = objects[i]->center[2];
			direction.x = objects[i]->direction[0];  
            direction.y = objects[i]->direction[1];  
            direction.z = objects[i]->direction[2];
			size.x = objects[i]->length;             
            size.y = objects[i]->width;              
            size.z = objects[i]->height;

            std::cout<<"objects info "<<objects[i]->ToString()<<std::endl;

			float x1 = size.x/2;
            float x2 = 0 - x1;
			float y1 = size.y/2;
            float y2 = 0 - y1;
		    float cos_theta = direction.x / sqrt(direction.x * direction.x  + direction.y * direction.y);
			float sin_theta = -1 * direction.y / sqrt(direction.x * direction.x  + direction.y * direction.y);
			//set x y

			verts[0][0] = verts[5][0] = x1 * cos_theta + y1 * sin_theta + center.x;
			verts[0][1] = verts[5][1] = y1 * cos_theta - x1 * sin_theta + center.y;

			verts[3][0] = verts[4][0] = x1 * cos_theta + y2 * sin_theta + center.x;
			verts[3][1] = verts[4][1] = y2 * cos_theta - x1 * sin_theta + center.y;

			verts[1][0] = verts[6][0] = x2 * cos_theta + y1 * sin_theta + center.x;
			verts[1][1] = verts[6][1] = y1 * cos_theta - x2 * sin_theta + center.y;

			verts[2][0] = verts[7][0] = x2 * cos_theta + y2 * sin_theta + center.x;
			verts[2][1] = verts[7][1] = y2 * cos_theta - x2 * sin_theta + center.y;

			//set z
			verts[0][2] = verts[1][2] = verts[2][2] = verts[3][2] = center.z + size.z/2;
			verts[4][2] = verts[5][2] = verts[6][2] = verts[7][2] = center.z - size.z/2;

			get_class_color(objects[i]->type, rgb);
			glColor3f((GLfloat)rgb[0],(GLfloat)rgb[1],(GLfloat)rgb[2]);
			glBegin(GL_LINE_STRIP);
			int j = 0;
			for (j = 0; j<16; j++){
				glVertex3f((GLfloat)verts[indices[j]][0],(GLfloat)verts[indices[j]][1],(GLfloat)verts[indices[j]][2]);   }
			glEnd();
			glFlush();
		}
	}
	if (draw_polygon){
		for (size_t i = 0; i < objects.size(); i++)
		{
			get_class_color(objects[i]->type, rgb);
			glColor3f((GLfloat)rgb[0],(GLfloat)rgb[1],(GLfloat)rgb[2]);
			glBegin(GL_LINE_LOOP);

			for (size_t j = 0; j< objects[i]->polygon.size(); j++) {
				glVertex3f((GLfloat)objects[i]->polygon.points[j].x,(GLfloat)objects[i]->polygon.points[j].y,-1.0); }
			glEnd();
			glFlush();
		}

	}
	if (draw_velocity){
		vec3 velocity_src;
		vec3 velocity_dst;
		float rgb[3]= {1,1,0};
		for (size_t i = 0; i < objects.size(); i++)
		{
			velocity_src = get_velocity_src_position(content, i);
			velocity_dst.x = velocity_src.x + objects[i]->velocity[0];
			velocity_dst.y = velocity_src.y + objects[i]->velocity[1];
			velocity_dst.z = -1.0f;
			get_class_color(objects[i]->type, rgb);
			glColor3f((GLfloat)rgb[0],(GLfloat)rgb[1],(GLfloat)rgb[2]);
			glBegin(GL_LINES);
			glVertex3f((GLfloat)velocity_src.x,(GLfloat)velocity_src.y,(GLfloat)velocity_src.z);
			glVertex3f((GLfloat)velocity_dst.x,(GLfloat)velocity_dst.y,(GLfloat)velocity_dst.z);
			glEnd();
			glFlush();
		}
	}
}

vec3 GLFWViewer::get_velocity_src_position(FrameContent* content, int i)
{
 	vec3 velocity_src;
	vec3 center;
	vec3 direction;
	vec3 size;
	vec3 velocity;
	std::vector<ObjectPtr> objects;
    std::vector<ObjectPtr> tracked_objects;
	
   	tracked_objects = content->get_tracked_objects();
   	
    for (size_t i = 0; i < tracked_objects.size(); ++i) {
        objects.push_back(tracked_objects[i]);
    }

	center.x = objects[i]->center[0];        
    center.y = objects[i]->center[1];        
    center.z = objects[i]->center[2];
	direction.x = objects[i]->direction[0];  
    direction.y = objects[i]->direction[1];  
    direction.z = objects[i]->direction[2];
	size.x = objects[i]->length;             
    size.y = objects[i]->width;              
    size.z = objects[i]->height;
	velocity.x = objects[i]->velocity[0];    
    velocity.y = objects[i]->velocity[1];    
    velocity.z = objects[i]->velocity[2];
	float cos_direction_velocity = (direction.x * direction.y + velocity.x * velocity.y)
				     / sqrt(direction.x * direction.x  + direction.y * direction.y)
				     / sqrt(velocity.x * velocity.x  + velocity.y * velocity.y);
	float cos_dir= direction.x / sqrt(direction.x * direction.x  + direction.y * direction.y);
	float sin_dir = -1 * direction.y / sqrt(direction.x * direction.x  + direction.y * direction.y);
	float x1 = 0.0f;
    float y1 = 0.0f;
    float x2 = 0.0f;
    float y2 = 0.0f;
	float x11 = 0.0f;
    float y11 = 0.0f;
    float x22 = 0.0f;
    float y22 = 0.0f;
	if (abs(cos_direction_velocity)>0.707)//<45dgree
	{
		x1 = size.x / 2;      y1 = 0;
		x2 = x1 * -1;         y2 = 0;
	}
	else
	{
		x1 = 0; y1 = size.y / 2;
		x2 = 0; y2 = y1*-1;
	}

	x11 = x1 * cos_dir + y1 * sin_dir + velocity.x;
	y11 = y1 * cos_dir - x1 * sin_dir + velocity.y;
	x22 = x2 * cos_dir + y2 * sin_dir + velocity.x;
	y22 = y2 * cos_dir - x2 * sin_dir + velocity.y;

	float dis1 = x11*x11+y11*y11;
	float dis2 = x22*x22+y22*y22;
	if (dis1>dis2)
	{
		velocity_src.x = x11 - velocity.x + center.x;
		velocity_src.y = y11 - velocity.y + center.y;
	}
	else
	{
		velocity_src.x = x22 - velocity.x + center.x;
		velocity_src.y = y22 - velocity.y + center.y;
	}
	velocity_src.z = -1.0f;
	return velocity_src;
}


/*bool GLFWViewer::show_map(FrameContent* content, bool show_map_roi, bool show_map_boundary)
{
	int i = 0;
	int j = 0;

	if (show_map_roi){
		std::vector<pcl_util::PointDCloudPtr> map_roi = content->get_map_roi();
		for (i = 0; i < map_roi.size(); i++) {
			glColor3f(0.0,0.0,1.0);
			glBegin(GL_POINTS);
			for (j = 0; j< map_roi[i]->points.size(); j++){

				glVertex3f((GLfloat)map_roi[i]->points[j].x,(GLfloat)map_roi[i]->points[j].y,(GLfloat)map_roi[i]->points[j].z);

			}
   			glEnd();
			glFlush();

			glColor3f(1.0,0.0,0.0);
			glBegin(GL_LINE_LOOP);
			for (j = 0; j< map_roi[i]->points.size(); j++){

				glVertex3f((GLfloat)map_roi[i]->points[j].x,(GLfloat)map_roi[i]->points[j].y,(GLfloat)map_roi[i]->points[j].z);
			}
			glEnd();
			glFlush();
		}
	}
	if (show_map_boundary){
		std::vector<adu::perception::onboard::RoadBoundary> map_boundary = content->get_map_boundary();
		for (i = 0; i < map_boundary.size(); i++) {
			glColor3f(1.0, 1.0, 0.0);
			glBegin(GL_LINE_STRIP);
			for (j = 0; j< map_boundary[i].left_boundary.size(); j++){
				glVertex3f((GLfloat)map_boundary[i].left_boundary[j].x,(GLfloat)map_boundary[i].left_boundary[j].y,(GLfloat)map_boundary[i].left_boundary[j].z);
			}
			glEnd();
			glFlush();
			glColor3f(0.0,1.0,0.0);
			glBegin(GL_LINE_STRIP);
			for (j = 0; j< map_boundary[i].right_boundary.size(); j++){
				glVertex3f((GLfloat)map_boundary[i].right_boundary[j].x,(GLfloat)map_boundary[i].right_boundary[j].y,(GLfloat)map_boundary[i].right_boundary[j].z);
			}
			glEnd();
			glFlush();
		}
	}
}*/

void GLFWViewer::render(){
 	glClear( GL_COLOR_BUFFER_BIT);
    pre_draw();

	/*if (FLAGS_enable_hdmap_input) show_map(_frame_content, FLAGS_show_map_roi, FLAGS_show_map_boundary); */
	if (show_cloud) draw_cloud(_frame_content);
	draw_objects(_frame_content, show_box, show_polygon, show_velocity);
	draw_circle();
    draw_car_forward_dir();
}

/************************callback functions************************/

void GLFWViewer::framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    void* user_data = glfwGetWindowUserPointer(window);
    if (user_data == NULL)
        return;

    GLFWViewer* vis = static_cast<GLFWViewer*>(user_data);
    vis->resize_framebuffer(width,height);
}

void GLFWViewer::key_callback(GLFWwindow* window,
                              int key,
                              int scancode,
                              int action,
                              int mods) {
	void* user_data = glfwGetWindowUserPointer(window);
    	if (user_data == NULL) return;
	if (action == GLFW_PRESS)
	{
		GLFWViewer* vis = static_cast<GLFWViewer*>(user_data);
		AINFO << "key_value: "<<key;
		vis->keyboard(key);
	}
}

void GLFWViewer::mouse_button_callback(GLFWwindow* window,
                                       int button,
                                       int action,
                                       int mods){
}

void GLFWViewer::mouse_cursor_position_callback(GLFWwindow* window,
                                                double xpos,
                                                double ypos){
    void* user_data = glfwGetWindowUserPointer(window);
    if (user_data == NULL)
        return;

    GLFWViewer* vis = static_cast<GLFWViewer*>(user_data);
    vis->mouse_move(xpos,ypos);
}

void GLFWViewer::mouse_scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
    void* user_data = glfwGetWindowUserPointer(window);
    if (user_data == NULL)
        return;

    GLFWViewer* vis = static_cast<GLFWViewer*>(user_data);
    vis->mouse_wheel(yoffset);
}


void GLFWViewer::error_callback(int error,
                                const char* description){
    std::cout << "ERROR - " << error << "  " << description << "\n";
}

/************************callback assistants************************/
/*void GLFWViewer::resize_window(int width, int height){
}*/


void GLFWViewer::resize_framebuffer(int width, int height){
    glViewport(0, 0, width, height);
    _pers_camera->setscreen_widthandheight(width, height);
}

void GLFWViewer::mouse_move(double xpos, double ypos) {
	int state_left = glfwGetMouseButton(_window, GLFW_MOUSE_BUTTON_LEFT);
	int state_right = glfwGetMouseButton(_window, GLFW_MOUSE_BUTTON_RIGHT);
	int x_delta = xpos - _mouse_prev_x;
	int y_delta = ypos - _mouse_prev_y;
    if (state_left == GLFW_PRESS) {
        Eigen::Quaterniond rot = _pers_camera->get_rotatation_by_mouse_from_qgwidget(
                                                      _mouse_prev_x, _mouse_prev_y, xpos, ypos);
        Eigen::Matrix3d rot_mat = rot.inverse().toRotationMatrix();
        Eigen::Vector3d scn_center = _pers_camera->scene_center();
        Eigen::Vector4d scn_center_(scn_center(0),scn_center(1), scn_center(2), 1);
        scn_center_ = _mode_mat*_view_mat*scn_center_;
        scn_center = scn_center_.head(3);
        Eigen::Vector3d r_multi_scn_center = rot_mat*scn_center;
        Eigen::Vector3d t = scn_center - r_multi_scn_center;
        Eigen::Matrix4d cur_mat = Eigen::Matrix4d::Identity();
        cur_mat.topLeftCorner(3,3) = rot_mat;
        cur_mat.topRightCorner(3,1) = t;
        _mode_mat = cur_mat*_mode_mat;
    }
    else if (state_right == GLFW_PRESS) {
        _mode_mat(0,3) += 0.1*x_delta;
        _mode_mat(1,3) -= 0.1*y_delta;
    }
    _mouse_prev_x = xpos;
    _mouse_prev_y = ypos;
}

void GLFWViewer::mouse_wheel(double delta) {
    _mode_mat(2,3) -= delta;
}

void GLFWViewer::reset(){
	_mode_mat = Eigen::Matrix4d::Identity();
}
void GLFWViewer::keyboard(int key){
	switch (key){
		case GLFW_KEY_R: //'R'
			reset();
			break;
		case GLFW_KEY_B: //'B'
			show_box = (show_box+1)%2;
			break;
		case GLFW_KEY_P: //'P'
			show_polygon = (show_polygon+1)%2;
			break;
		case GLFW_KEY_V: //'V'
			show_velocity = (show_velocity+1)%2;
			break;
        case GLFW_KEY_S: //'S'
            _show_cloud_state = (_show_cloud_state+1)%3;
            break;
 	}
}

} // namespace perception
} // namespace adu
