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

#include <iomanip>

#include "modules/common/util/file.h"
#include "modules/common/util/string_util.h"
#include "modules/perception/base/distortion_model.h"
#include "modules/perception/camera/app/obstacle_camera_perception.h"
#include "modules/perception/camera/tools/offline/transform_server.h"
#include "modules/perception/camera/tools/offline/visualizer.h"
#include "modules/perception/common/io/io_util.h"

DEFINE_string(test_list, "full_test_list.txt", "exe image list");
DEFINE_string(image_root, "", "root dir of images");
DEFINE_string(image_ext, ".jpg", "extension of image name");
DEFINE_string(image_color, "bgr", "color space of image");
DEFINE_string(config_root, "conf/perception/camera/", "config_root");
DEFINE_string(tf_file, "", "tf file");
DEFINE_string(config_file, "obstacle.pt", "config_file");
DEFINE_string(narrow_name, "front_12mm", " camera for projecting");
DEFINE_string(base_camera_name, "front_6mm", "camera to be peojected");
DEFINE_string(sensor_name, "front_6mm,front_12mm", "camera to use");
DEFINE_string(params_dir, "/home/caros/cyber/params", "params dir");
DEFINE_string(visualize_dir, "/tmp/0000", "visualize dir");
DEFINE_double(camera_fps, 15, "camera_fps");
DEFINE_bool(do_undistortion, false, "do_undistortion");
DEFINE_string(undistortion_save_dir, "", "save imgs dir after undistortion");

namespace apollo {
namespace perception {
namespace camera {

static const float kDefaultPitchAngle = 0.0f;
static const float kDefaultCameraHeight = 1.5f;

void save_image(const std::string &path, base::Image8U &image) { // NOLINT
  AINFO << path;
  int cv_type = image.type() == base::Color::GRAY ? CV_8UC1 : CV_8UC3;
  cv::Mat cv_img(image.rows(), image.cols(), cv_type,
                 image.mutable_cpu_data(),
                 image.width_step());
  cv::imwrite(path, cv_img);
}

int work() {
  // Init pipeline:
  ObstacleCameraPerception perception;
  CameraPerceptionInitOptions init_option;
  CameraPerceptionOptions options;
  init_option.root_dir = FLAGS_config_root;
  init_option.conf_file = FLAGS_config_file;
  init_option.lane_calibration_working_sensor_name = FLAGS_base_camera_name;
  CHECK(perception.Init(init_option));

  // Init frame
  const int FRAME_CAPACITY = 20;
  std::vector<CameraFrame> frame_list(FRAME_CAPACITY);
  for (auto &frame : frame_list) {
    frame.track_feature_blob.reset(new base::Blob<float>());
  }

  // Init input list
  int frame_id = -1;
  std::ifstream fin;
  fin.open(FLAGS_test_list, std::ifstream::in);
  if (!fin.is_open()) {
    AERROR << "Cannot open exe list: " << FLAGS_test_list;
    return -1;
  }

  // Init camera list
  std::vector<std::string> camera_names;
  apollo::common::util::Split(FLAGS_sensor_name, ',', &camera_names);

  // Init data provider
  DataProvider::InitOptions data_options;
  data_options.image_height = 1080;
  data_options.image_width = 1920;
  data_options.do_undistortion = FLAGS_do_undistortion;
  data_options.device_id = 0;
  std::map<std::string, DataProvider *> name_provider_map;

  std::vector<DataProvider> data_providers(camera_names.size());

  for (size_t i = 0; i < camera_names.size(); ++i) {
    data_options.sensor_name = camera_names[i];
    CHECK(data_providers[i].Init(data_options));
    name_provider_map.insert(std::pair<std::string, DataProvider *>(
        camera_names[i], &data_providers[i]));
    AINFO << "Init data_provider for " << camera_names[i];
  }

  // Init intrinsic
  std::map<std::string, Eigen::Matrix3f> intrinsic_map;
  auto manager = common::SensorManager::Instance();
  for (const auto &camera_name : camera_names) {
    base::BaseCameraModelPtr model;
    model = manager->GetUndistortCameraModel(camera_name);
    auto pinhole = dynamic_cast<base::PinholeCameraModel *> (model.get());
    Eigen::Matrix3f intrinsic = pinhole->get_intrinsic_params();
    intrinsic_map[camera_name] = intrinsic;
    AINFO << "#intrinsics of " << camera_name << ": "
             << intrinsic_map[camera_name];
  }

  // Init extrinsic
  TransformServer transform_server;
  CHECK(transform_server.Init(camera_names, FLAGS_params_dir));
  transform_server.print();

  // Init transform
  if (FLAGS_tf_file != "") {
    CHECK(transform_server.LoadFromFile(FLAGS_tf_file));
  }

  // Set calibration service camera_ground_height
  //      pitch_angle and project_matrix
  std::map<std::string, float> name_camera_ground_height_map;
  std::map<std::string, float> name_camera_pitch_angle_diff_map;
  for (size_t i = 0; i < camera_names.size(); ++i) {
    Eigen::Affine3d c2g;
    if (!transform_server.QueryTransform(camera_names[i], "ground", &c2g)) {
      AINFO << "Failed to query transform from " << camera_names[i]
               << " to ground";
      return -1;
    }
    float camera_ground_height = static_cast<float>(c2g.translation()[2]);
    AINFO << camera_names[i] << " height: " << camera_ground_height;
    name_camera_ground_height_map[camera_names[i]] = camera_ground_height;
    Eigen::Matrix3d project_matrix;
    float pitch_diff = 0.0f;
    if (FLAGS_base_camera_name == camera_names[i]) {
      project_matrix = Eigen::Matrix3d::Identity();
      pitch_diff = 0;
    } else {
      Eigen::Affine3d trans;
      if (!transform_server.QueryTransform(camera_names[i],
                                           FLAGS_base_camera_name,
                                           &trans)) {
        AINFO << "Failed to query transform";
        return -1;
      }
      Eigen::Vector3d euler = trans.linear().eulerAngles(0, 1, 2);
      pitch_diff = static_cast<float>(euler(0));
      if (pitch_diff > 10.0 * M_PI / 180.0) {
        pitch_diff = 0;
      }
      project_matrix =
          intrinsic_map.at(FLAGS_base_camera_name).cast<double>() *
              trans.linear() *
              intrinsic_map.at(camera_names[i]).cast<double>().inverse();
    }
    name_camera_pitch_angle_diff_map[camera_names[i]] = pitch_diff;
  }
  perception.SetCameraHeightAndPitch(name_camera_ground_height_map,
                                     name_camera_pitch_angle_diff_map,
                                     kDefaultPitchAngle);
  Visualizer visualize;
  CHECK(visualize.Init(camera_names, &transform_server));
  visualize.SetDirectory(FLAGS_visualize_dir);
  std::string line;
  std::string image_name;
  std::string camera_name;

  while (fin >> line) {
    std::vector<std::string> temp_strs;
    apollo::common::util::Split(line, '/', &temp_strs);
    if (temp_strs.size() != 2) {
      AERROR << "invaid format in " << FLAGS_test_list;
    }
    camera_name = temp_strs[0];
    image_name = temp_strs[1];

    AINFO << "image: " << image_name << " camera_name:" << camera_name;
    std::string image_path = FLAGS_image_root + "/" + camera_name + "/"
        + image_name + FLAGS_image_ext;
    cv::Mat image;
    if (FLAGS_image_color == "gray") {
      image = cv::imread(image_path, CV_LOAD_IMAGE_GRAYSCALE);
      cv::cvtColor(image, image, CV_GRAY2RGB);
    } else if (FLAGS_image_color == "rgb") {
      image = cv::imread(image_path, CV_LOAD_IMAGE_COLOR);
      cv::cvtColor(image, image, CV_BGR2RGB);
    } else if (FLAGS_image_color == "bgr") {
      image = cv::imread(image_path, CV_LOAD_IMAGE_COLOR);
    } else {
      AERROR << "Invalid color: " << FLAGS_image_color;
    }
    if (image.data == 0) {
      AERROR << "Cannot read image: " << image_path;
      return -1;
    }

    frame_id++;
    CameraFrame &frame = frame_list[frame_id % FRAME_CAPACITY];
    frame.frame_id = frame_id;
    std::stringstream ss(image_name);
    frame.timestamp = 0.0;
    ss >> frame.timestamp;
    frame.timestamp *= 1e-9;
    if (frame.timestamp < 1e-3) {
      frame.timestamp = 1.0 / FLAGS_camera_fps * frame_id;
    }
    AINFO << "Timestamp: " << std::fixed << std::setprecision(10)
             << frame.timestamp;

    if (FLAGS_base_camera_name == camera_name) {
      frame.project_matrix = Eigen::Matrix3d::Identity();
    } else {
      Eigen::Affine3d trans;
      if (!transform_server.QueryTransform(camera_name,
                                           FLAGS_base_camera_name,
                                           &trans)) {
        AINFO << "Failed to query transform from "
                 << camera_name << " to " << FLAGS_base_camera_name;
        return -1;
      }
      frame.project_matrix =
          intrinsic_map.at(FLAGS_base_camera_name).cast<double>() *
              trans.linear() *
              intrinsic_map.at(camera_name).cast<double>().inverse();
    }
    frame.data_provider = name_provider_map.at(camera_name);
    AINFO << "Project Matrix: \n" << frame.project_matrix;
    Eigen::Affine3d pose;
    if (!transform_server.QueryPos(frame.timestamp, &pose)) {
      pose.setIdentity();
    }

    Eigen::Affine3d c2n;
    if (!transform_server.QueryTransform(camera_name, "novatel", &c2n)) {
      AINFO << "Failed to query transform from "
               << camera_name << " to novatel";
      return -1;
    }
    frame.camera2world_pose = pose * c2n;
    frame.data_provider->FillImageData(image.rows,
                                       image.cols,
                                       (const uint8_t *) (
                                           image.data),
                                       "bgr8");
    perception.GetCalibrationService(&frame.calibration_service);

    // save distortion images
    std::string save_dir = FLAGS_undistortion_save_dir + "/" + camera_name;
    if (FLAGS_do_undistortion && (FLAGS_undistortion_save_dir != "") &&
        apollo::common::util::PathExists(save_dir)) {
      base::Image8U image1;
      DataProvider::ImageOptions image_options;
      image_options.target_color = base::Color::BGR;
      frame.data_provider->GetImage(image_options, &image1);
      save_image(save_dir + "/" + image_name + FLAGS_image_ext, image1);
    }

    CHECK(perception.Perception(options, &frame));
    visualize.ShowResult(image, frame);
  }
  return 0;
}
}  // namespace camera
}  // namespace perception
}  // namespace apollo

int main(int argc, char *argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::SetUsageMessage("command line brew\n"
                          "Usage: camera_benchmark <args>\n");

  return apollo::perception::camera::work();
}
