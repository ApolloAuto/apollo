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
#include "modules/perception/camera/tools/offline/visualizer.h"

#include "cyber/common/log.h"

namespace apollo {
namespace perception {
namespace camera {

std::vector<cv::Scalar> colorlist = {
    cv::Scalar(0, 0, 255),
    cv::Scalar(0, 100, 255),
    cv::Scalar(0, 200, 255),
    cv::Scalar(100, 255, 255),
    cv::Scalar(200, 255, 255),
    cv::Scalar(255, 100, 255),
    cv::Scalar(255, 0, 255),
    cv::Scalar(255, 255, 100),
    cv::Scalar(255, 255, 0),
    cv::Scalar(255, 0, 100),
    cv::Scalar(255, 0, 0),
    cv::Scalar(0, 255, 0),
    cv::Scalar(100, 255, 100)};

bool Visualizer::Init(const std::vector<std::string> &camera_names,
                      TransformServer *tf_server) {
  tf_server_ = tf_server;
  CHECK(tf_server_ != nullptr);
  last_timestamp_ = 0;
  int width = 1920;
  int height = 1080;
  small_h_ = static_cast<int>(height * 0.7);
  small_w_ = static_cast<int>(width * 0.7);
  world_h_ = 2 * small_h_;
  wide_pixel_ = 800;
  m2pixel_ = 6;

  for (size_t i = 0; i < camera_names.size(); ++i) {
    camera_image_[camera_names[i]] =
        cv::Mat(small_h_, small_w_, CV_8UC3, cv::Scalar(0, 0, 0));
  }
  world_image_ = cv::Mat(world_h_, wide_pixel_, CV_8UC3, cv::Scalar(0, 0, 0));
  return true;
}

bool Visualizer::Init_all_info_single_camera(const std::string &camera_name,
            std::map<std::string, Eigen::Matrix3f> intrinsic_map,
            std::map<std::string, Eigen::Matrix4d> extrinsic_map,
            Eigen::Matrix4d ex_lidar2imu, double pitch_adj) {
  intrinsic_map_ = intrinsic_map;
  extrinsic_map_ = extrinsic_map;
  last_timestamp_ = 0;
  int width = 1920;
  int height = 1080;
  small_h_ = static_cast<int>(height * 0.6);
  small_w_ = static_cast<int>(width * 0.6);
  world_h_ = 2 * small_h_;
  wide_pixel_ = 800;
  m2pixel_ = 6;

  camera_image_[camera_name + "_2D"] =
      cv::Mat(small_h_, small_w_, CV_8UC3, cv::Scalar(0, 0, 0));
  camera_image_[camera_name + "_3D"] =
      cv::Mat(small_h_, small_w_, CV_8UC3, cv::Scalar(0, 0, 0));
  world_image_ = cv::Mat(world_h_, wide_pixel_, CV_8UC3, cv::Scalar(0, 0, 0));

  extrinsic_map_.at(camera_name).block(0, 3, 3, 1) =
                   - extrinsic_map_.at(camera_name).block(0, 3, 3, 1);
  Eigen::Matrix4d Rz;
  Rz << 0, 1, 0, 0,
       -1, 0, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
  extrinsic_map_.at(camera_name) = extrinsic_map_.at(camera_name) *
                                   ex_lidar2imu * Rz;
  extrinsic_map_.at(camera_name)(2, 3) -= 1.6;
  // adj of pitch in camera coords
  Eigen::Matrix4d Rx;
  Rx << 1, 0, 0, 0,
        0, cos(pitch_adj), -sin(pitch_adj), 0,
        0, sin(pitch_adj), cos(pitch_adj), 0,
        0, 0, 0, 1;
  extrinsic_map_.at(camera_name) = extrinsic_map_.at(camera_name) * Rx;
  AINFO << "extrinsic parameter from camera to car: "
        << extrinsic_map_.at(camera_name);

  // compute the homography matrix, such that H [u, v, 1]' ~ [X_l, Y_l, 1]
  Eigen::Matrix3d K = intrinsic_map_.at(camera_name).cast<double> ();
  Eigen::Matrix3d R = extrinsic_map_.at(camera_name).block(0, 0, 3, 3);
  Eigen::Vector3d T = extrinsic_map_.at(camera_name).block(0, 3, 3, 1);
  Eigen::Matrix3d H;

  H.block(0, 0, 3, 2) = (K * R.transpose()).block(0, 0, 3, 2);
  H.block(0, 2, 3, 1) = -K * R.transpose() * T;
  homography_im2ground_ = H.inverse();

  AINFO << "homography_im2ground_: " << homography_im2ground_;
  return true;
}

void Visualizer::SetDirectory(const std::string &path) {
  std::string command;
  command = "mkdir -p " + path;
  system(command.c_str());
  command = "rm " + path + "/*.jpg";
  system(command.c_str());
  path_ = path;
}

void Visualizer::Draw2Dand3D(const cv::Mat &img, const CameraFrame &frame) {
  cv::Mat image = img.clone();
  Eigen::Affine3d pose;
  if (!tf_server_->QueryPos(frame.timestamp, &pose)) {
    pose.setIdentity();
  }
  Eigen::Affine3d lidar2novatel;
  tf_server_->QueryTransform("velodyne128", "novatel", &lidar2novatel);
  Eigen::Affine3d lidar2world = pose * lidar2novatel;
  Eigen::Affine3d world2lidar = lidar2world.inverse();
  for (const auto &object : frame.tracked_objects) {
    base::RectF rect(object->camera_supplement.box);
    cv::Rect r(static_cast<int>(rect.x),
               static_cast<int>(rect.y),
               static_cast<int>(rect.width),
               static_cast<int>(rect.height));
    cv::rectangle(image, r, colorlist[object->track_id % colorlist.size()], 2);
    cv::putText(image,
                std::to_string(object->track_id),
                cv::Point(static_cast<int>(rect.x), static_cast<int>(rect.y)),
                cv::FONT_HERSHEY_DUPLEX,
                1,
                cv::Scalar(0, 0, 255),
                2);
    Eigen::Vector3d theta;
    theta << cos(object->theta), sin(object->theta), 0;
    theta = world2lidar.linear() * theta;
    float yaw = static_cast<float>(atan2(theta[1], theta[0]));
    Eigen::Matrix2d rotate;
    rotate << cos(yaw), -sin(yaw),
      sin(yaw), cos(yaw);

    Eigen::Vector3d pos;
    pos << object->center[0], object->center[1], object->center[2];
    pos = world2lidar * pos;
    Eigen::Vector2d pos_2d;
    pos_2d << pos[0], pos[1];
    Eigen::Vector3d v;
    v << object->velocity[0], object->velocity[1], object->velocity[2];
    v = world2lidar.linear() * v;
    Eigen::Vector2d v_2d;
    v_2d << v[0] + pos_2d[0], v[1] + pos_2d[1];
    Eigen::Vector2d p1;
    p1 << object->size[0] / 2, object->size[1] / 2;
    p1 = rotate * p1 + pos_2d;
    Eigen::Vector2d p2;
    p2 << -object->size[0] / 2, object->size[1] / 2;
    p2 = rotate * p2 + pos_2d;
    Eigen::Vector2d p3;
    p3 << -object->size[0] / 2, -object->size[1] / 2;
    p3 = rotate * p3 + pos_2d;
    Eigen::Vector2d p4;
    p4 << object->size[0] / 2, -object->size[1] / 2;
    p4 = rotate * p4 + pos_2d;

    cv::line(world_image_, world_point_to_bigimg(p1),
             world_point_to_bigimg(p2),
             colorlist[object->track_id % colorlist.size()], 2);
    cv::line(world_image_, world_point_to_bigimg(p2),
             world_point_to_bigimg(p3),
             colorlist[object->track_id % colorlist.size()], 2);
    cv::line(world_image_, world_point_to_bigimg(p3),
             world_point_to_bigimg(p4),
             colorlist[object->track_id % colorlist.size()], 2);
    cv::line(world_image_, world_point_to_bigimg(p4),
             world_point_to_bigimg(p1),
             colorlist[object->track_id % colorlist.size()], 2);
    cv::line(world_image_, world_point_to_bigimg(pos_2d),
             world_point_to_bigimg(v_2d),
             colorlist[object->track_id % colorlist.size()], 2);
  }
  last_timestamp_ = frame.timestamp;
  camera_image_[frame.data_provider->sensor_name()] = image;
  cv::resize(image, camera_image_[frame.data_provider->sensor_name()],
             cv::Size(small_w_, small_h_));
}

void Visualizer::ShowResult(const cv::Mat &img, const CameraFrame &frame) {
  cv::Mat image = img.clone();
  std::string camera_name = frame.data_provider->sensor_name();

  if (frame.timestamp - last_timestamp_ > 0.02) {
    cv::Mat bigimg(world_h_, small_w_ + wide_pixel_, CV_8UC3);
    camera_image_["front_6mm"].copyTo(bigimg(cv::Rect(0,
                                                            0,
                                                            small_w_,
                                                            small_h_)));
    camera_image_["front_12mm"].copyTo(bigimg(cv::Rect(0,
                                                          small_h_,
                                                          small_w_,
                                                          small_h_)));
    world_image_.copyTo(bigimg(cv::Rect(small_w_, 0, wide_pixel_, world_h_)));
    char path[1000];
    snprintf(path, sizeof(path), "%s/%06d.jpg", path_.c_str(), frame.frame_id);
    AINFO << path;
    cv::imshow("", bigimg);
    cv::imwrite(path, bigimg);
    cvWaitKey(30);
    world_image_ = cv::Mat(world_h_, wide_pixel_, CV_8UC3, cv::Scalar(0, 0, 0));
    draw_range_circle();
  }

  cv::putText(image,
              "timestamp:" + std::to_string(frame.timestamp),
              cv::Point(10, 50),
              cv::FONT_HERSHEY_DUPLEX,
              1.3,
              cv::Scalar(0, 0, 255),
              3);
  cv::putText(image,
              "camera_name: " + camera_name,
              cv::Point(10, 100),
              cv::FONT_HERSHEY_DUPLEX,
              1.3,
              cv::Scalar(0, 0, 255),
              3);
  cv::putText(image,
              "frame id: " + std::to_string(frame.frame_id),
              cv::Point(10, 150),
              cv::FONT_HERSHEY_DUPLEX,
              1.3,
              cv::Scalar(0, 0, 255),
              3);
  Draw2Dand3D(image, frame);
}

void Visualizer::Draw2Dand3D_all_info_single_camera(const cv::Mat &img,
                                      const CameraFrame &frame,
                                      Eigen::Matrix3d intrinsic,
                                      Eigen::Matrix4d extrinsic) {
  cv::Mat img2 = img;
  // plot FOV
  cv::Point p_fov_1;
  p_fov_1.x = 0;
  p_fov_1.y = static_cast<int>(1080/1.5);

  cv::Point p_fov_2;
  p_fov_2.x = 1920-1;
  p_fov_2.y = static_cast<int>(1080/1.5);

  cv::Point p_fov_3;
  p_fov_3.x = 0;
  p_fov_3.y = 1080-1;

  cv::Point p_fov_4;
  p_fov_4.x = 1920-1;
  p_fov_4.y = 1080-1;

  cv::line(img2, p_fov_1, p_fov_2,
          cv::Scalar(255, 255, 255), 2);
  cv::line(img2, p_fov_1, p_fov_3,
          cv::Scalar(255, 255, 255), 2);
  cv::line(img2, p_fov_2, p_fov_4,
          cv::Scalar(255, 255, 255), 2);
  cv::line(world_image_,
      world_point_to_bigimg(image2ground(p_fov_1)),
      world_point_to_bigimg(image2ground(p_fov_2)),
      cv::Scalar(255, 255, 255), 2);
  cv::line(world_image_,
      world_point_to_bigimg(image2ground(p_fov_1)),
      world_point_to_bigimg(image2ground(p_fov_3)),
      cv::Scalar(255, 255, 255), 2);
  cv::line(world_image_,
      world_point_to_bigimg(image2ground(p_fov_2)),
      world_point_to_bigimg(image2ground(p_fov_4)),
      cv::Scalar(255, 255, 255), 2);

  AINFO << "FOV point 1: " << image2ground(p_fov_1);
  AINFO << "FOV point 2: " << image2ground(p_fov_2);
  AINFO << "FOV point 3: " << image2ground(p_fov_3);
  AINFO << "FOV point 4: " << image2ground(p_fov_4);

  // plot laneline on image and ground plane
  for (const auto &object : frame.lane_objects) {
    cv::Scalar lane_color;
    switch (object.pos_type) {
      case base::LaneLinePositionType::CURB_LEFT: {
        lane_color = colorlist[0];
        break;
      }
      case base::LaneLinePositionType::FOURTH_LEFT: {
        lane_color = colorlist[1];
        break;
      }
      case base::LaneLinePositionType::THIRD_LEFT: {
        lane_color = colorlist[2];
        break;
      }
      case base::LaneLinePositionType::ADJACENT_LEFT: {
        lane_color = colorlist[3];
        break;
      }
      case base::LaneLinePositionType::EGO_LEFT: {
        lane_color = colorlist[4];
        break;
      }
      case base::LaneLinePositionType::EGO_RIGHT: {
        lane_color = colorlist[5];
        break;
      }
      case base::LaneLinePositionType::ADJACENT_RIGHT: {
        lane_color = colorlist[6];
        break;
      }
      case base::LaneLinePositionType::THIRD_RIGHT: {
        lane_color = colorlist[7];
        break;
      }
      case base::LaneLinePositionType::FOURTH_RIGHT: {
        lane_color = colorlist[8];
        break;
      }
      case base::LaneLinePositionType::CURB_RIGHT: {
        lane_color = colorlist[9];
        break;
      }
    }

    cv::Point p_prev;
    p_prev.x = static_cast<int> (object.curve_image_point_set[0].x);
    p_prev.y = static_cast<int> (object.curve_image_point_set[0].y);
    Eigen::Vector2d p_prev_ground = image2ground(p_prev);

    for (unsigned i=1; i < object.curve_image_point_set.size(); i++) {
      cv::Point p_cur;
      p_cur.x = static_cast<int> (object.curve_image_point_set[i].x);
      p_cur.y = static_cast<int> (object.curve_image_point_set[i].y);
      Eigen::Vector2d p_cur_ground = image2ground(p_cur);

      cv::line(img2, p_prev, p_cur,
          lane_color, 2);
      cv::line(world_image_,
          world_point_to_bigimg(p_prev_ground),
          world_point_to_bigimg(p_cur_ground),
          lane_color, 2);
      p_prev = p_cur;
      p_prev_ground = p_cur_ground;
    }
  }

  cv::Mat image_2D = img2.clone();
  cv::Mat image_3D = img2.clone();
  for (const auto &object : frame.tracked_objects) {
    // plot 2D box on image_2D
    base::RectF rect(object->camera_supplement.box);
    cv::Rect r(static_cast<int>(rect.x),
               static_cast<int>(rect.y),
               static_cast<int>(rect.width),
               static_cast<int>(rect.height));
    cv::rectangle(image_2D, r,
                  colorlist[object->track_id % colorlist.size()], 2);
    cv::putText(image_2D,
                std::to_string(object->track_id),
                cv::Point(static_cast<int>(rect.x), static_cast<int>(rect.y)),
                cv::FONT_HERSHEY_DUPLEX,
                1,
                cv::Scalar(0, 0, 255),
                2);

    // compute 8 vetices in camera coodinates
    Eigen::Vector3d pos;
    pos << object->camera_supplement.local_center[0],
           object->camera_supplement.local_center[1],
           object->camera_supplement.local_center[2];
    double theta_ray = atan2(pos[0], pos[2]);
    double theta = object->camera_supplement.alpha + theta_ray;

    Eigen::Matrix3d rotate_ry;
    rotate_ry << cos(theta), 0, sin(theta),
                          0, 1, 0,
                -sin(theta), 0, cos(theta);
    Eigen::Vector3d p1;
    p1 << object->size[0] / 2, object->size[2] / 2, object->size[1] / 2;
    p1 = rotate_ry * p1 + pos;
    Eigen::Vector3d p2;
    p2 << -object->size[0] / 2, object->size[2] / 2, object->size[1] / 2;
    p2 = rotate_ry * p2 + pos;
    Eigen::Vector3d p3;
    p3 << -object->size[0] / 2, object->size[2] / 2, -object->size[1] / 2;
    p3 = rotate_ry * p3 + pos;
    Eigen::Vector3d p4;
    p4 << object->size[0] / 2, object->size[2] / 2, -object->size[1] / 2;
    p4 = rotate_ry * p4 + pos;
    Eigen::Vector3d p5;
    p5 << object->size[0] / 2, -object->size[2] / 2, object->size[1] / 2;
    p5 = rotate_ry * p5 + pos;
    Eigen::Vector3d p6;
    p6 << -object->size[0] / 2, -object->size[2] / 2, object->size[1] / 2;
    p6 = rotate_ry * p6 + pos;
    Eigen::Vector3d p7;
    p7 << -object->size[0] / 2, -object->size[2] / 2, -object->size[1] / 2;
    p7 = rotate_ry * p7 + pos;
    Eigen::Vector3d p8;
    p8 << object->size[0] / 2, -object->size[2] / 2, -object->size[1] / 2;
    p8 = rotate_ry * p8 + pos;

    // compute 4 bottom vetices in lidar coordinate
    // Eigen::Vector3d p1_l_3d = extrinsic.block(0,0,3,3) * p1 +
    //                           extrinsic.block(0,3,3,1);
    // Eigen::Vector3d p2_l_3d = extrinsic.block(0,0,3,3) * p2 +
    //                           extrinsic.block(0,3,3,1);
    // Eigen::Vector3d p3_l_3d = extrinsic.block(0,0,3,3) * p3 +
    //                           extrinsic.block(0,3,3,1);
    // Eigen::Vector3d p4_l_3d = extrinsic.block(0,0,3,3) * p4 +
    //                           extrinsic.block(0,3,3,1);

    // Eigen::Vector2d p1_l = p1_l_3d.block(0, 0, 2, 1);
    // Eigen::Vector2d p2_l = p2_l_3d.block(0, 0, 2, 1);
    // Eigen::Vector2d p3_l = p3_l_3d.block(0, 0, 2, 1);
    // Eigen::Vector2d p4_l = p4_l_3d.block(0, 0, 2, 1);

    // compute obstacle center in lidar ground
    cv::Point c_2D;
    c_2D.x = static_cast<int>(rect.x + rect.width/2);
    c_2D.y = static_cast<int>(rect.y + rect.height);
    Eigen::Vector2d c_2D_l = image2ground(c_2D);
    Eigen::Matrix2d rotate_rz;
    theta = theta - M_PI_2;
    rotate_rz << cos(theta), sin(theta),
                 -sin(theta), cos(theta);
    // plot obstacles on ground plane in lidar coordinates
    Eigen::Vector2d p1_l;
    p1_l << object->size[0] / 2, object->size[1] / 2;
    p1_l = rotate_rz * p1_l + c_2D_l;
    Eigen::Vector2d p2_l;
    p2_l << -object->size[0] / 2, object->size[1] / 2;
    p2_l = rotate_rz * p2_l + c_2D_l;
    Eigen::Vector2d p3_l;
    p3_l << -object->size[0] / 2, -object->size[1] / 2;
    p3_l = rotate_rz * p3_l + c_2D_l;
    Eigen::Vector2d p4_l;
    p4_l << object->size[0] / 2, -object->size[1] / 2;
    p4_l = rotate_rz * p4_l + c_2D_l;
    cv::line(world_image_, world_point_to_bigimg(p1_l),
             world_point_to_bigimg(p2_l),
             colorlist[object->track_id % colorlist.size()], 2);
    cv::line(world_image_, world_point_to_bigimg(p2_l),
             world_point_to_bigimg(p3_l),
             colorlist[object->track_id % colorlist.size()], 2);
    cv::line(world_image_, world_point_to_bigimg(p3_l),
             world_point_to_bigimg(p4_l),
             colorlist[object->track_id % colorlist.size()], 2);
    cv::line(world_image_, world_point_to_bigimg(p4_l),
             world_point_to_bigimg(p1_l),
             colorlist[object->track_id % colorlist.size()], 2);

    // plot projected 3D box on image_3D
    p1 = intrinsic * p1;
    p2 = intrinsic * p2;
    p3 = intrinsic * p3;
    p4 = intrinsic * p4;
    p5 = intrinsic * p5;
    p6 = intrinsic * p6;
    p7 = intrinsic * p7;
    p8 = intrinsic * p8;
    cv::Point p1_proj;
    p1_proj.x = static_cast<int>(p1[0] / p1[2]);
    p1_proj.y = static_cast<int>(p1[1] / p1[2]);
    cv::Point p2_proj;
    p2_proj.x = static_cast<int>(p2[0] / p2[2]);
    p2_proj.y = static_cast<int>(p2[1] / p2[2]);
    cv::Point p3_proj;
    p3_proj.x = static_cast<int>(p3[0] / p3[2]);
    p3_proj.y = static_cast<int>(p3[1] / p3[2]);
    cv::Point p4_proj;
    p4_proj.x = static_cast<int>(p4[0] / p4[2]);
    p4_proj.y = static_cast<int>(p4[1] / p4[2]);
    cv::Point p5_proj;
    p5_proj.x = static_cast<int>(p5[0] / p5[2]);
    p5_proj.y = static_cast<int>(p5[1] / p5[2]);
    cv::Point p6_proj;
    p6_proj.x = static_cast<int>(p6[0] / p6[2]);
    p6_proj.y = static_cast<int>(p6[1] / p6[2]);
    cv::Point p7_proj;
    p7_proj.x = static_cast<int>(p7[0] / p7[2]);
    p7_proj.y = static_cast<int>(p7[1] / p7[2]);
    cv::Point p8_proj;
    p8_proj.x = static_cast<int>(p8[0] / p8[2]);
    p8_proj.y = static_cast<int>(p8[1] / p8[2]);
    cv::line(image_3D, p1_proj, p2_proj,
             colorlist[object->track_id % colorlist.size()], 2);
    cv::line(image_3D, p2_proj, p3_proj,
             colorlist[object->track_id % colorlist.size()], 2);
    cv::line(image_3D, p3_proj, p4_proj,
             colorlist[object->track_id % colorlist.size()], 2);
    cv::line(image_3D, p4_proj, p1_proj,
             colorlist[object->track_id % colorlist.size()], 2);
    cv::line(image_3D, p5_proj, p6_proj,
             colorlist[object->track_id % colorlist.size()], 2);
    cv::line(image_3D, p6_proj, p7_proj,
             colorlist[object->track_id % colorlist.size()], 2);
    cv::line(image_3D, p7_proj, p8_proj,
             colorlist[object->track_id % colorlist.size()], 2);
    cv::line(image_3D, p8_proj, p5_proj,
             colorlist[object->track_id % colorlist.size()], 2);
    cv::line(image_3D, p1_proj, p5_proj,
             cv::Scalar(255, 255, 255), 2);
    cv::line(image_3D, p2_proj, p6_proj,
             colorlist[object->track_id % colorlist.size()], 2);
    cv::line(image_3D, p3_proj, p7_proj,
             colorlist[object->track_id % colorlist.size()], 2);
    cv::line(image_3D, p4_proj, p8_proj,
             colorlist[object->track_id % colorlist.size()], 2);
  }
  last_timestamp_ = frame.timestamp;
  camera_image_[frame.data_provider->sensor_name() + "_2D"] = image_2D;
  cv::resize(image_2D, camera_image_[frame.data_provider->sensor_name() +
             "_2D"], cv::Size(small_w_, small_h_));
  camera_image_[frame.data_provider->sensor_name() + "_3D"] = image_3D;
  cv::resize(image_3D, camera_image_[frame.data_provider->sensor_name() +
             "_3D"], cv::Size(small_w_, small_h_));
}

void Visualizer::ShowResult_all_info_single_camera(const cv::Mat &img,
                                      const CameraFrame &frame) {
  cv::Mat image = img.clone();
  std::string camera_name = frame.data_provider->sensor_name();

  if (frame.timestamp - last_timestamp_ > 0.02) {
    cv::Mat bigimg(world_h_, small_w_ + wide_pixel_, CV_8UC3);
    camera_image_[camera_name + "_2D"].copyTo(bigimg(cv::Rect(0,
                                                            0,
                                                            small_w_,
                                                            small_h_)));
    camera_image_[camera_name + "_3D"].copyTo(bigimg(cv::Rect(0,
                                                          small_h_,
                                                          small_w_,
                                                          small_h_)));
    world_image_.copyTo(bigimg(cv::Rect(small_w_, 0, wide_pixel_, world_h_)));
    char path[1000];
    snprintf(path, sizeof(path), "%s/%06d.jpg", path_.c_str(), frame.frame_id);
    AINFO << path;
    cv::imshow("", bigimg);
    cv::imwrite(path, bigimg);
    cvWaitKey(30);
    world_image_ = cv::Mat(world_h_, wide_pixel_, CV_8UC3, cv::Scalar(0, 0, 0));
    draw_range_circle();
  }

  cv::putText(image,
              "timestamp:" + std::to_string(frame.timestamp),
              cv::Point(10, 50),
              cv::FONT_HERSHEY_DUPLEX,
              1.3,
              cv::Scalar(0, 0, 255),
              3);
  cv::putText(image,
              "camera_name: " + camera_name,
              cv::Point(10, 100),
              cv::FONT_HERSHEY_DUPLEX,
              1.3,
              cv::Scalar(0, 0, 255),
              3);
  cv::putText(image,
              "frame id: " + std::to_string(frame.frame_id),
              cv::Point(10, 150),
              cv::FONT_HERSHEY_DUPLEX,
              1.3,
              cv::Scalar(0, 0, 255),
              3);
  if (intrinsic_map_.find(camera_name) != intrinsic_map_.end() &&
     extrinsic_map_.find(camera_name) != extrinsic_map_.end()) {
    Draw2Dand3D_all_info_single_camera(image, frame,
                intrinsic_map_.at(camera_name).cast<double> (),
                extrinsic_map_.at(camera_name));
  } else {
    AERROR << "fail to find necessuary intrinsic or extrinsic params.";
  }
}

void Visualizer::draw_range_circle() {
  cv::Scalar color(255, 100, 0);
  cv::circle(world_image_,
             cv::Point(wide_pixel_ / 2, world_h_),
             1 * m2pixel_,
             color,
             1);
  for (int i = 20; i < 300; i += 20) {
    cv::circle(world_image_,
               cv::Point(wide_pixel_ / 2, world_h_),
               i * m2pixel_,
               color,
               2);
  }
  for (int i = 50; i < 300; i += 50) {
    cv::putText(world_image_,
                std::to_string(i),
                cv::Point(wide_pixel_ / 2, world_h_ - i * m2pixel_),
                cv::FONT_HERSHEY_DUPLEX,
                1,
                cv::Scalar(0, 0, 255),
                2);
  }
}

cv::Point Visualizer::world_point_to_bigimg(const Eigen::Vector2d &p) {
  cv::Point point;
  point.x = static_cast<int>(-p[1] * m2pixel_ + wide_pixel_ / 2);
  point.y = static_cast<int>(world_h_ - p[0] * m2pixel_);
  return point;
}

Eigen::Vector2d Visualizer::image2ground(cv::Point p_img) {
  Eigen::Vector3d p_homo;
  p_homo << p_img.x, p_img.y, 1;
  p_homo = homography_im2ground_ * p_homo;
  p_homo[0] = p_homo[0] / p_homo[2];
  p_homo[1] = p_homo[1] / p_homo[2];
  return p_homo.block(0, 0, 2, 1);
}
}  // namespace camera
}  // namespace perception
}  // namespace apollo
