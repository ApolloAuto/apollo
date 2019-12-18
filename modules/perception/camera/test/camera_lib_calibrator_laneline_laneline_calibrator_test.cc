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

#include "modules/perception/camera/lib/calibrator/laneline/laneline_calibrator.h"

#include "absl/strings/str_cat.h"
#include "cyber/common/log.h"
#include "modules/perception/camera/test/camera_lib_calibrator_laneline_app_util.h"
#include "modules/perception/camera/test/camera_lib_calibrator_laneline_lane_calibrator_util.h"
#include "modules/perception/camera/test/camera_lib_calibrator_laneline_lane_io.h"

namespace apollo {
namespace perception {
namespace camera {

DEFINE_bool(vis, false, "plot results on image sequences");

TEST(LanelineCalibratorTest, laneline_calibrator_test) {
  std::string root_path =
      "/apollo/modules/perception/testdata/"
      "camera/lib/calibrator/laneline/data/";
  std::string img_path = root_path + "/img/";
  std::string lane_path = root_path + "/lane/";
  std::string intrinsics_file = root_path + "/intrinsics.txt";
  std::string tf_file = root_path + "/camera2world.txt";

  std::string vis_path = root_path + "/vis/";
  std::vector<std::string> image_list;

  // color constants (BGR)
  static const cv::Scalar kColorLaneLine_left(0, 0, 255);
  static const cv::Scalar kColorLaneLine_right(0, 255, 0);
  static const cv::Scalar kColorVaniLine_res(255, 0, 0);
  static const cv::Scalar kColorText(20, 20, 20);

  // default constant
  static const float kYawRate = kYawRateDefault;
  static const float kVelocity = kVelocityDefault;
  static const float kTimeDiff = kTimeDiffDefault;

  // load data
  float k_mat[9] = {0};
  int image_width = 0;
  int image_height = 0;

  EXPECT_FALSE(adu::perception::obstacle::load_ref_camera_k_mat(
      ".", k_mat, &image_width, &image_height));
  EXPECT_TRUE(adu::perception::obstacle::load_ref_camera_k_mat(
      intrinsics_file, k_mat, &image_width, &image_height));
  AINFO << "----intrinsics:\n"
        << k_mat[0] << ", " << k_mat[1] << ", " << k_mat[2] << "\n"
        << k_mat[3] << ", " << k_mat[4] << ", " << k_mat[5] << "\n"
        << k_mat[6] << ", " << k_mat[7] << ", " << k_mat[8];
  AINFO << "----width, height: \n" << image_width << ", " << image_height;

  std::vector<std::string> lane_list;

  EXPECT_TRUE(
      adu::perception::obstacle::load_filename(lane_path, "", &lane_list));
  EXPECT_TRUE(
      adu::perception::obstacle::load_filename(lane_path, ".txt", &lane_list));
  int nr_frames = lane_list.size();
  image_list.resize(0);
  for (int i = 0; i < nr_frames; ++i) {
    int slash_pos = lane_list[i].find_last_of("/");
    int dot_pos = lane_list[i].find_last_of(".");
    std::string token =
        lane_list[i].substr(slash_pos + 1, dot_pos - slash_pos - 1);
    image_list.push_back(img_path + "/" + token + ".jpg");
  }

  std::vector<std::string> frame_list;  // these three are aligned
  std::vector<double> time_stamps;
  std::vector<Eigen::Matrix4d> camera2world;
  EXPECT_FALSE(camera::LoadCamera2WorldTfs("abc", &frame_list, &time_stamps,
                                           &camera2world));
  EXPECT_TRUE(camera::LoadCamera2WorldTfs(tf_file, &frame_list, &time_stamps,
                                          &camera2world));
  EXPECT_EQ(nr_frames, (int)frame_list.size());
  EXPECT_EQ(nr_frames, (int)time_stamps.size());
  AINFO << "number of frames: " << nr_frames;

  camera::EgoLane ego_lane;
  EXPECT_FALSE(LoadLaneDet("", &ego_lane));

  // process
  CalibratorInitOptions init_options;
  init_options.image_width = image_width;
  init_options.image_height = image_height;
  init_options.focal_x = k_mat[0];
  init_options.focal_y = k_mat[4];
  init_options.cx = k_mat[2];
  init_options.cy = k_mat[5];

  LaneLineCalibrator calibrator;
  LocalCalibratorInitOptions local_init_options;
  local_init_options.image_width = image_width;
  local_init_options.image_height = image_height;
  local_init_options.focal_y = init_options.focal_y;
  local_init_options.focal_x = init_options.focal_x;
  local_init_options.cx = init_options.cx;
  local_init_options.cy = init_options.cy;
  CalibratorParams params1;
  CalibratorParams params2;
  params1 = params2;
  calibrator.calibrator_.Init(local_init_options, nullptr);
  calibrator.calibrator_.ClearUp();

  calibrator.calibrator_.Init(local_init_options, &params1);
  Eigen::Vector2f point1(1, -1);
  Eigen::Vector2f point2(-1, 1);
  Eigen::Vector2f point3(image_width + 1, 1);
  Eigen::Vector2f point4(1, image_height + 1);
  Eigen::Vector2f point5(1, 1);

  calibrator.calibrator_.image_width_ = image_width;
  calibrator.calibrator_.image_height_ = image_height;

  calibrator.calibrator_.is_in_image(point1);
  calibrator.calibrator_.is_in_image(point2);
  calibrator.calibrator_.is_in_image(point3);
  calibrator.calibrator_.is_in_image(point4);
  calibrator.calibrator_.is_in_image(point5);

  float line_seg_l[4] = {0};
  float line_seg_r[4] = {0};
  VanishingPoint v_point;
  calibrator.calibrator_.GetIntersectionFromTwoLineSegments(
      line_seg_l, line_seg_r, &v_point);

  LaneLine line;
  line.lane_point.resize(0);
  float line_seg[4] = {0};
  calibrator.calibrator_.SelectTwoPointsFromLineForVanishingPoint(line,
                                                                  line_seg);
  int num_points = 20;
  line.lane_point.resize(0);
  for (int i = 0; i < num_points; ++i) {
    line.lane_point.push_back(Eigen::Vector2f(i + 1.0, i + 1.0));
  }
  calibrator.calibrator_.SelectTwoPointsFromLineForVanishingPoint(line,
                                                                  line_seg);

  line.lane_point.resize(0);
  for (int i = 0; i < num_points; ++i) {
    line.lane_point.push_back(Eigen::Vector2f(num_points - i, num_points - i));
  }
  calibrator.calibrator_.SelectTwoPointsFromLineForVanishingPoint(line,
                                                                  line_seg);

  EXPECT_TRUE(calibrator.Init(init_options));

  CvFont font;
  cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 1.0f, 1.0f, 0, 2, 8);
  float pitch_calib = -9999.9f;
  cv::Size size_half(image_width / 2, image_height / 2);

  for (int i = 0; i < nr_frames; ++i) {
    AINFO << "------------------------\n";
    AINFO << "Processing frame: " << i << " , " << image_list[i];

    camera::EgoLane ego_lane;
    if (!LoadLaneDet(lane_list[i], &ego_lane)) {
      continue;
    }

    AINFO << "nr-of-left-lane-pts: " << ego_lane.left_line.lane_point.size();
    AINFO << "nr-of-right-lane-pts: " << ego_lane.right_line.lane_point.size();

    cv::Mat test_image;
    if (FLAGS_vis) {
      // load image
      test_image = cv::imread(image_list[i].c_str());
      EXPECT_EQ(test_image.cols, image_width);
      EXPECT_EQ(test_image.rows, image_height);

      // draw lane pts
      camera::draw_lane_pts(ego_lane.left_line.lane_point, kColorLaneLine_left,
                            &test_image);
      camera::draw_lane_pts(ego_lane.right_line.lane_point,
                            kColorLaneLine_right, &test_image);
    }

    CameraFrame frame;
    CalibratorOptions calibrator_options;
    calibrator_options.lane_objects =
        std::make_shared<std::vector<base::LaneLine>>(frame.lane_objects);
    calibrator_options.camera2world_pose =
        std::make_shared<Eigen::Affine3d>(frame.camera2world_pose);
    calibrator_options.timestamp = &(frame.timestamp);
    float pitch_angle;
    // blank ego lane
    EXPECT_FALSE(calibrator.Calibrate(calibrator_options, &pitch_angle));

    frame.lane_objects.resize(1);
    frame.lane_objects[0].pos_type = base::LaneLinePositionType::EGO_LEFT;
    int num_left_points = ego_lane.left_line.lane_point.size();
    frame.lane_objects[0].curve_image_point_set.resize(num_left_points);
    for (int j = 0; j < num_left_points; ++j) {
      frame.lane_objects[0].curve_image_point_set[j].x =
          ego_lane.left_line.lane_point[j](0);
      frame.lane_objects[0].curve_image_point_set[j].y =
          ego_lane.left_line.lane_point[j](1);
    }
    calibrator_options.lane_objects =
        std::make_shared<std::vector<base::LaneLine>>(frame.lane_objects);
    calibrator_options.camera2world_pose =
        std::make_shared<Eigen::Affine3d>(frame.camera2world_pose);
    calibrator_options.timestamp = &(frame.timestamp);
    // lack ego right
    EXPECT_FALSE(calibrator.Calibrate(calibrator_options, &pitch_angle));
    frame.lane_objects.resize(2);
    frame.lane_objects[1].pos_type = base::LaneLinePositionType::EGO_RIGHT;
    int num_right_points = ego_lane.right_line.lane_point.size();
    frame.lane_objects[1].curve_image_point_set.resize(num_right_points);
    for (int j = 0; j < num_right_points; ++j) {
      frame.lane_objects[1].curve_image_point_set[j].x =
          ego_lane.right_line.lane_point[j](0);
      frame.lane_objects[1].curve_image_point_set[j].y =
          ego_lane.right_line.lane_point[j](1);
    }

    frame.timestamp = time_stamps[i];
    frame.camera2world_pose = camera2world[i];
    bool update = false;

    // process
    calibrator_options.lane_objects =
        std::make_shared<std::vector<base::LaneLine>>(frame.lane_objects);
    calibrator_options.camera2world_pose =
        std::make_shared<Eigen::Affine3d>(frame.camera2world_pose);
    calibrator_options.timestamp = &(frame.timestamp);
    update = calibrator.Calibrate(calibrator_options, &pitch_angle);
    if (update) {
      pitch_calib = pitch_angle;
    }

    if (update) {
      AINFO << "updated";
    } else {
      AINFO << "not updated";
    }
    AINFO << "pitch_calib: " << pitch_calib;
    // end-of-process

    const std::string timediff_yawrate_velocity_text = absl::StrCat(
        "time_diff_: ", std::to_string(calibrator.GetTimeDiff()).substr(0, 4),
        " | yaw_rate_: ", std::to_string(calibrator.GetYawRate()).substr(0, 4),
        " | velocity_: ",
        std::to_string(calibrator.GetVelocity()).substr(0, 4));
    AINFO << timediff_yawrate_velocity_text;

    cv::Mat test_image_half_result;
    if (FLAGS_vis) {
      adu::perception::obstacle::write_text_on_image(
          &test_image, 100.0f, 100.0f, timediff_yawrate_velocity_text.c_str(),
          font, kColorText);
      cv::resize(test_image, test_image_half_result, size_half);
    }
    if (update) {
      int vanishing_row_res = calibrator.GetVanishingRow();
      AINFO << "result from online calib:\n"
            << "pitch_res: " << pitch_calib << "\n"
            << "v-row res: " << vanishing_row_res << "\n";
      if (FLAGS_vis) {
        std::string pitch_calib_text =
            "pitch: " + std::to_string(pitch_calib).substr(0, 8);
        adu::perception::obstacle::write_text_on_image(
            &test_image_half_result, 50.0f, 100.0f, pitch_calib_text.c_str(),
            font, kColorText);
        EXPECT_TRUE(camera::draw_vanishing_row_on_image(
            kColorVaniLine_res, vanishing_row_res / 2,
            &test_image_half_result));
        EXPECT_FALSE(camera::draw_vanishing_row_on_image(
            kColorVaniLine_res, -1, &test_image_half_result));
        EXPECT_FALSE(camera::draw_vanishing_row_on_image(
            kColorVaniLine_res, test_image_half_result.cols,
            &test_image_half_result));
      } else {
        cv::Mat image_fake(10, 10, CV_8UC3, cv::Scalar::all(100));
        EXPECT_TRUE(camera::draw_vanishing_row_on_image(
            kColorVaniLine_res, image_fake.cols / 2, &image_fake));
        EXPECT_FALSE(camera::draw_vanishing_row_on_image(kColorVaniLine_res, -1,
                                                         &image_fake));
        EXPECT_FALSE(camera::draw_vanishing_row_on_image(
            kColorVaniLine_res, image_fake.cols, &image_fake));
      }
    }

    if (FLAGS_vis) {
      int slash_pos = lane_list[i].find_last_of("/");
      std::string save_name = vis_path + image_list[i].substr(slash_pos + 1);
      cv::imwrite(save_name, test_image_half_result);
      AINFO << "save_name: " << save_name;
    }
  }
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
