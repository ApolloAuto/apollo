/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#pragma once

#include <opencv2/opencv.hpp>

#include <map>
#include <string>
#include <vector>

#include "modules/perception/camera/app/cipv_camera.h"
#include "modules/perception/camera/common/camera_frame.h"
#include "modules/perception/camera/common/util.h"
#include "modules/perception/camera/tools/offline/transform_server.h"
#include "modules/perception/proto/motion_service.pb.h"

namespace apollo {
namespace perception {
namespace camera {

class Visualizer {
 public:
  bool Init(const std::vector<std::string> &camera_names,
            TransformServer *tf_server);
  bool Init_all_info_single_camera(
      const std::vector<std::string> &camera_names,
      const std::string &visual_camera,
      const std::map<std::string, Eigen::Matrix3f> &intrinsic_map,
      const std::map<std::string, Eigen::Matrix4d> &extrinsic_map,
      const Eigen::Matrix4d &ex_lidar2imu,
      const double pitch_adj,
      const double yaw_adj,
      const double roll_adj,
      const int image_height,
      const int image_width);
  bool adjust_angles(const std::string &camera_name, const double pitch_adj,
                     const double yaw_adj, const double roll_adj);
  bool SetDirectory(const std::string &path);
  void ShowResult(const cv::Mat &img, const CameraFrame &frame);
  void Draw2Dand3D(const cv::Mat &img, const CameraFrame &frame);
  void ShowResult_all_info_single_camera(
      const cv::Mat &img,
      const CameraFrame &frame,
      const base::MotionBufferPtr motion_buffer,
      const Eigen::Affine3d &world2camera);
  void Draw2Dand3D_all_info_single_camera(
      const std::string &camera_name,
      const cv::Mat &img,
      const CameraFrame &frame,
      const Eigen::Matrix3d &intrinsic,
      const Eigen::Matrix4d &extrinsic,
      const Eigen::Affine3d &world2camera,
      const base::MotionBufferPtr motion_buffer);
  bool DrawTrajectories(const base::ObjectPtr &object,
                        const base::MotionBufferPtr motion_buffer);
  cv::Point world_point_to_bigimg(const Eigen::Vector2d &p);
  cv::Point world_point_to_bigimg(const Eigen::Vector4f &p);
  Eigen::Vector2d image2ground(const std::string &camera_name, cv::Point p_img);
  cv::Point ground2image(const std::string &camera_name,
                         Eigen::Vector2d p_ground);
  std::string type_to_string(const apollo::perception::base::ObjectType type);
  std::string sub_type_to_string(
      const apollo::perception::base::ObjectSubType type);
  Eigen::Matrix3d homography_im2car(std::string camera_name) {
    return homography_image2ground_[camera_name];
  }
  void Set_ROI(int input_offset_y, int crop_height, int crop_width) {
    roi_start_ = input_offset_y;
    roi_height_ = crop_height;
    roi_width_ = crop_width;
  }
  bool euler_to_quaternion(Eigen::Vector4d *quaternion,
                           const double pitch_radian, const double yaw_radian,
                           const double roll_radian);
  bool save_manual_calibration_parameter(const std::string &camera_name,
                                         const double pitch_adj_degree,
                                         const double yaw_adj_degree,
                                         const double roll_adj_degree);
  bool save_extrinsic_in_yaml(const std::string &camera_name,
                              const Eigen::Matrix4d &extrinsic,
                              const Eigen::Vector4d &quaternion,
                              const double pitch_radian,
                              const double yaw_radian,
                              const double roll_radian);
  double regularize_angle(const double angle);
  bool copy_backup_file(const std::string &filename);
  bool key_handler(const std::string &camera_name, const int key);
  bool reset_key();
  void draw_range_circle();
  void draw_selected_image_boundary(const int width, int const height,
                                    cv::Mat *image);

  bool write_out_img_ = false;
  bool cv_imshow_img_ = true;
  // homograph between image and ground plane
  std::map<std::string, Eigen::Matrix3d> homography_image2ground_;
  std::map<std::string, Eigen::Matrix3d> homography_ground2image_;

 private:
  std::map<std::string, cv::Mat> camera_image_;
  cv::Mat world_image_;
  TransformServer *tf_server_;
  std::string path_;
  double last_timestamp_ = 0.0;
  int image_width_ = 1920;
  int image_height_ = 1080;
  int wide_pixel_ = 800;
  double scale_ratio_ = 0.6;
  int small_h_ = 0;
  int small_w_ = 0;
  int world_h_ = 0;
  int m2pixel_ = 15;  // 6;
  double fov_cut_ratio_ = 0.55;
  double degree_to_radian_factor_ = M_PI / 180.0;
  double radian_to_degree_factor_ = 180.0 / M_PI;

  std::map<std::string, double> pitch_adj_degree_;
  std::map<std::string, double> yaw_adj_degree_;
  std::map<std::string, double> roll_adj_degree_;

  double max_pitch_degree_ = 5.0;
  double min_pitch_degree_ = -5.0;
  double max_yaw_degree_ = 5.0;
  double min_yaw_degree_ = -5.0;
  double max_roll_degree_ = 5.0;
  double min_roll_degree_ = -5.0;

  cv::Point p_fov_1_;
  cv::Point p_fov_2_;
  cv::Point p_fov_3_;
  cv::Point p_fov_4_;
  int roi_height_ = 768;
  int roi_start_ = 312;
  int roi_width_ = 1920;

  std::map<std::string, Eigen::Vector2d> vp1_;
  std::map<std::string, Eigen::Vector2d> vp2_;

  std::vector<std::string> camera_names_;
  std::string visual_camera_ = "front_6mm";
  // map for store params
  std::map<std::string, Eigen::Matrix3f> intrinsic_map_;
  std::map<std::string, Eigen::Matrix4d> extrinsic_map_;
  Eigen::Matrix4d ex_lidar2imu_;
  std::map<std::string, Eigen::Matrix4d> ex_camera2lidar_;
  Eigen::Matrix4d ex_camera2imu_;
  Eigen::Matrix4d ex_imu2camera_;
  Eigen::Matrix4d ex_car2camera_;
  Eigen::Matrix4d ex_camera2car_;
  Eigen::Matrix4d ex_imu2car_;
  Eigen::Matrix4d adjusted_camera2car_ = Eigen::Matrix4d::Identity();

  Eigen::Matrix4d projection_matrix_;
  std::map<std::string, Eigen::Matrix3d> K_;

  // Visualization related variables
  bool use_class_color_ = true;
  bool capture_screen_ = false;
  bool capture_video_ = false;
  bool show_camera_box2d_ = true;
  bool show_camera_box3d_ = true;
  bool show_camera_bdv_ = true;
  bool show_virtual_egolane_ = true;
  bool show_radar_pc_ = true;
  bool show_fusion_ = false;
  bool show_associate_color_ = false;
  bool show_type_id_label_ = true;
  bool show_verbose_ = false;
  bool show_trajectory_ = true;
  bool show_vp_grid_ = true;  // show vanishing point and ground plane grid
  bool draw_lane_objects_ = true;
  bool show_box_ = true;
  bool show_velocity_ = false;
  bool show_polygon_ = true;
  bool show_text_ = false;
  bool show_help_text_ = false;
  bool manual_calibration_mode_ = false;
  bool show_homography_object_ = false;
  unsigned int show_lane_count_ = 1;
  std::string help_str_;
  // color
  cv::Scalar color_cipv_ = cv::Scalar(255, 255, 255);
  cv::Scalar virtual_lane_color_ = cv::Scalar(0, 0, 255);
  int line_thickness_ = 2;
  int cipv_line_thickness_ = 6;
  int trajectory_line_thickness_ = 1;
  double speed_limit_ = 1.0;  // in m/s
  unsigned int lane_step_num_ = 20;
  Cipv cipv_;
  unsigned int all_camera_recieved_ = 0;
};

}  // namespace camera
}  // namespace perception
}  // namespace apollo
