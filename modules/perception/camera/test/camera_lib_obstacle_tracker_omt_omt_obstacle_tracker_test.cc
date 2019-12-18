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

#include "modules/perception/camera/lib/obstacle/tracker/omt/omt_obstacle_tracker.h"

#include "absl/strings/str_cat.h"
#include "absl/strings/str_split.h"
#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "gflags/gflags.h"

#include "modules/perception/base/box.h"
#include "modules/perception/base/distortion_model.h"
#include "modules/perception/base/object.h"
#include "modules/perception/base/object_types.h"
#include "modules/perception/camera/common/camera_frame.h"
#include "modules/perception/camera/common/object_template_manager.h"
#include "modules/perception/camera/lib/interface/base_obstacle_detector.h"
#include "modules/perception/camera/lib/interface/base_obstacle_tracker.h"
#include "modules/perception/common/geometry/common.h"
#include "modules/perception/common/io/io_util.h"

namespace apollo {
namespace perception {
namespace camera {

DEFINE_string(data_root,
              "/apollo/modules/perception/testdata/"
              "camera/lib/obstacle/tracker/omt/test_fusion_data",
              "root dir of images");
DEFINE_int32(max_img_num, 10, "max length of test images");
DEFINE_string(narrow_name, "narrow", "type of camera for projecting");
DEFINE_string(obstacle_name, "obstacle", "type of camera to be peojected");
DEFINE_string(image_root,
              "/apollo/modules/perception/testdata/"
              "camera/lib/obstacle/tracker/omt/images",
              "root dir of images");
DEFINE_int32(feature_length, 576, "size of feature");
DEFINE_string(base_camera_name, "onsemi_obstacle", "camera to be peojected");
DEFINE_string(sensor_name, "onsemi_obstacle,onsemi_narrow", "camera to use");
DEFINE_string(test_list, "test.txt", "exe image list");
DEFINE_double(camera_fps, 15, "camera_fps");
DEFINE_string(image_ext, ".jpg", "extension of image name");
DEFINE_string(image_color, "bgr", "color space of image");

int read_detections(const std::string &path, const int &feature_dim,
                    const std::string &camera_name, CameraFrame *frame) {
  const TemplateMap &kMinTemplateHWL =
      ObjectTemplateManager::Instance()->MinTemplateHWL();

  std::ifstream fin(path);
  if (!fin.is_open()) {
    AERROR << "Cannot open : " << path;
    return -1;
  }
  int frame_num = -1;
  int det_count = 0;

  int feature_size = feature_dim;
  fin >> frame_num >> det_count;
  frame->frame_id = frame_num;
  frame->track_feature_blob.reset(new base::Blob<float>);
  (frame->track_feature_blob)->Reshape({det_count, feature_size});
  float x = 0.0f;
  float y = 0.0f;
  float width = 0.0f;
  float height = 0.0f;
  float feature = 0.0f;
  float score = 0.0f;
  frame->detected_objects.clear();
  for (int i = 0; i < det_count; ++i) {
    fin >> x >> y >> width >> height >> score;
    base::BBox2DF bbox(x, y, x + width, y + height);
    base::ObjectPtr object(new base::Object);
    object->camera_supplement.box = bbox;
    object->camera_supplement.sensor_name = camera_name;
    object->sub_type = base::ObjectSubType::CAR;
    object->type = base::ObjectType::VEHICLE;
    object->size(0) = kMinTemplateHWL.at(base::ObjectSubType::CAR).at(2);
    object->size(1) = kMinTemplateHWL.at(base::ObjectSubType::CAR).at(1);
    object->size(2) = kMinTemplateHWL.at(base::ObjectSubType::CAR).at(0);
    float *data =
        frame->track_feature_blob->mutable_cpu_data() + i * feature_size;
    for (int j = 0; j < feature_size; j++) {
      fin >> feature;
      *data = feature;
      ++data;
    }
    frame->detected_objects.push_back(object);
  }
  return 0;
}

int write_track_imgs(const std::string &out_path, const cv::Mat &frame,
                     const std::vector<base::ObjectPtr> &tracked_objects) {
  cv::Mat image_mat_src = frame.clone();
  for (auto &visual_object : tracked_objects) {
    cv::Rect rect(visual_object->camera_supplement.box.xmin,
                  visual_object->camera_supplement.box.ymin,
                  visual_object->camera_supplement.box.xmax -
                      visual_object->camera_supplement.box.xmin,
                  visual_object->camera_supplement.box.ymax -
                      visual_object->camera_supplement.box.ymin);
    cv::rectangle(image_mat_src, rect, cv::Scalar(0, 255, 0), 2);
    cv::Point txt_pos;
    txt_pos.x = 0.5 * (visual_object->camera_supplement.box.xmin +
                       visual_object->camera_supplement.box.xmax);
    txt_pos.y = 0.5 * (visual_object->camera_supplement.box.ymin +
                       visual_object->camera_supplement.box.ymax);
    cv::putText(image_mat_src, std::to_string(visual_object->track_id), txt_pos,
                cv::FONT_HERSHEY_PLAIN, 2.0, cv::Scalar(0, 0, 255), 2);
  }
  cv::imwrite(out_path, image_mat_src);
  return 0;
}

// @description: load camera extrinsics from yaml file
bool LoadExtrinsics(const std::string &yaml_file,
                    Eigen::Matrix4d *camera_extrinsic) {
  if (!apollo::cyber::common::PathExists(yaml_file)) {
    AINFO << yaml_file << " not exist!";
    return false;
  }
  YAML::Node node = YAML::LoadFile(yaml_file);
  double qw = 0.0;
  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
  double tx = 0.0;
  double ty = 0.0;
  double tz = 0.0;
  try {
    if (node.IsNull()) {
      AINFO << "Load " << yaml_file << " failed! please check!";
      return false;
    }
    qw = node["transform"]["rotation"]["w"].as<double>();
    qx = node["transform"]["rotation"]["x"].as<double>();
    qy = node["transform"]["rotation"]["y"].as<double>();
    qz = node["transform"]["rotation"]["z"].as<double>();
    tx = node["transform"]["translation"]["x"].as<double>();
    ty = node["transform"]["translation"]["y"].as<double>();
    tz = node["transform"]["translation"]["z"].as<double>();
  } catch (YAML::InvalidNode &in) {
    AERROR << "load camera extrisic file " << yaml_file
           << " with error, YAML::InvalidNode exception";
    return false;
  } catch (YAML::TypedBadConversion<double> &bc) {
    AERROR << "load camera extrisic file " << yaml_file
           << " with error, YAML::TypedBadConversion exception";
    return false;
  } catch (YAML::Exception &e) {
    AERROR << "load camera extrisic file " << yaml_file
           << " with error, YAML exception:" << e.what();
    return false;
  }
  camera_extrinsic->setConstant(0);
  Eigen::Quaterniond q;
  q.x() = qx;
  q.y() = qy;
  q.z() = qz;
  q.w() = qw;
  (*camera_extrinsic).block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
  (*camera_extrinsic)(0, 3) = tx;
  (*camera_extrinsic)(1, 3) = ty;
  (*camera_extrinsic)(2, 3) = tz;
  (*camera_extrinsic)(3, 3) = 1;
  return true;
}

// @description: get project matrix
bool GetProjectMatrix(
    const std::string &camera_name,
    const std::map<std::string, Eigen::Matrix4d> &extrinsic_map,
    const std::map<std::string, Eigen::Matrix3f> &intrinsic_map,
    Eigen::Matrix3d *project_matrix, double *pitch_diff = nullptr) {
  std::string base_camera_name = FLAGS_base_camera_name;
  if (camera_name == base_camera_name) {
    *project_matrix = Eigen::Matrix3d::Identity();
    return true;
  }

  *project_matrix =
      intrinsic_map.at(base_camera_name).cast<double>() *
      extrinsic_map.at(base_camera_name).block<3, 3>(0, 0).inverse() *
      extrinsic_map.at(camera_name).block<3, 3>(0, 0) *
      intrinsic_map.at(camera_name).cast<double>().inverse();
  // extract the pitch_diff = pitch_narrow - pitch_obstacle
  if (pitch_diff != nullptr) {
    Eigen::Vector3d euler =
        (extrinsic_map.at(base_camera_name).block<3, 3>(0, 0).inverse() *
         extrinsic_map.at(camera_name).block<3, 3>(0, 0))
            .eulerAngles(0, 1, 2);
    *pitch_diff = euler(0);
    AINFO << "pitch diff: " << *pitch_diff;
  }
  return true;
}

// @description: modified from common::LoadBrownCameraIntrinsic
// the original funciton return by BrownCameraDistortionModel class, but
// intrinsic_parm_ is a protected filed, cannot get directly
bool LoadCameraIntrinsics(const std::string &yaml_file,
                          Eigen::Matrix3f *camera_intrinsic) {
  if (!apollo::cyber::common::PathExists(yaml_file)) {
    AINFO << yaml_file << " not exist!";
    return false;
  }
  YAML::Node node = YAML::LoadFile(yaml_file);
  if (node.IsNull()) {
    AINFO << "Load " << yaml_file << " failed! please check!";
    return false;
  }
  try {
    for (int i = 0; i < static_cast<int>(node["K"].size()); ++i) {
      (*camera_intrinsic)(i / 3, i % 3) = node["K"][i].as<float>();
    }
  } catch (YAML::Exception &e) {
    AERROR << "load camera extrisic file " << yaml_file
           << " with error, YAML exception: " << e.what();
    return false;
  }
  return true;
}

TEST(FusionObstacleTrackerTest, FusionObstacleTracker_test) {
  // Init object template
  ObjectTemplateManagerInitOptions object_template_init_options;
  object_template_init_options.root_dir =
      "/apollo/modules/perception/testdata/"
      "camera/app/data/perception/camera/common/object_template/";
  object_template_init_options.conf_file = "object_template.pt";
  CHECK(ObjectTemplateManager::Instance()->Init(object_template_init_options));

  // Init camera list
  const std::vector<std::string> camera_names =
      absl::StrSplit(FLAGS_sensor_name, ',');
  // Init data provider
  DataProvider::InitOptions data_options;
  data_options.image_height = 1080;
  data_options.image_width = 1920;
  data_options.do_undistortion = false;
  data_options.device_id = 0;
  std::map<std::string, DataProvider *> name_provider_map;

  std::vector<DataProvider> data_providers(camera_names.size());

  for (int i = 0; i < camera_names.size(); i++) {
    data_options.sensor_name = camera_names[i];
    CHECK(data_providers[i].Init(data_options));
    name_provider_map.insert(std::pair<std::string, DataProvider *>(
        camera_names[i], &data_providers[i]));
    AINFO << "Init data_provider for " << camera_names[i];
  }

  // Init extrinsic/intrinsic
  std::map<std::string, Eigen::Matrix4d> extrinsic_map;
  std::map<std::string, Eigen::Matrix3f> intrinsic_map;
  for (int i = 0; i < camera_names.size(); i++) {
    Eigen::Matrix3f intrinsic;
    ASSERT_TRUE(LoadCameraIntrinsics(
        FLAGS_data_root + "/params/" + camera_names[i] + "_intrinsics.yaml",
        &intrinsic));
    intrinsic_map[camera_names[i]] = intrinsic;
    Eigen::Matrix4d extrinsic;
    ASSERT_TRUE(LoadExtrinsics(
        FLAGS_data_root + "/params/" + camera_names[i] + "_extrinsics.yaml",
        &extrinsic));
    extrinsic_map[camera_names[i]] = extrinsic;
  }

  inference::CudaUtil::set_device_id(0);
  // init tracker
  ObstacleTrackerInitOptions init_options;
  init_options.root_dir =
      "/apollo/modules/perception/testdata/"
      "camera/lib/obstacle/tracker/omt/data/models/"
      "omt_obstacle_tracker";
  init_options.conf_file = "config.pt";
  init_options.image_height = 1080;
  init_options.image_width = 1920;
  std::unique_ptr<BaseObstacleTracker> camera_tracker(
      BaseObstacleTrackerRegisterer::GetInstanceByName("OMTObstacleTracker"));
  ASSERT_TRUE(camera_tracker->Init(init_options));
  EXPECT_EQ("OMTObstacleTracker", camera_tracker->Name());

  // read gt
  std::string filename = FLAGS_data_root + "/det_gt.txt";
  std::ifstream fin_gt(filename);
  ASSERT_TRUE(fin_gt.is_open());
  std::vector<std::vector<base::CameraObjectSupplement>> det_gts;
  std::string image_name;
  int det_count = 0;
  while (fin_gt >> image_name >> det_count) {
    std::vector<base::CameraObjectSupplement> bboxs;
    base::CameraObjectSupplement bbox;
    float x = 0.0f;
    float y = 0.0f;
    float width = 0.0f;
    float height = 0.0f;
    int id = 0;
    for (int i = 0; i < det_count; ++i) {
      fin_gt >> x >> y >> width >> height >> id;
      base::BBox2DF box(x, y, x + width, y + height);
      bbox.box = box;
      bbox.local_track_id = id;
      bboxs.push_back(bbox);
    }
    det_gts.push_back(bboxs);
  }

  std::vector<std::vector<int>> tracked_index(det_gts.size());
  std::vector<std::vector<base::CameraObjectSupplement>> tracked_results(
      det_gts.size());
  std::vector<CameraFrame> frames(det_gts.size());
  std::string line;
  std::string camera_name;
  int frame_num = 0;

  // Init input list
  std::ifstream fin;
  filename = FLAGS_data_root + "/" + FLAGS_test_list;
  fin.open(filename, std::ifstream::in);
  ASSERT_TRUE(fin.is_open());
  while (fin >> line) {
    const std::vector<std::string> temp_strs = absl::StrSplit(line, '/');
    if (temp_strs.size() != 2) {
      AERROR << "invaid format in " << FLAGS_test_list;
    }
    camera_name = temp_strs[0];
    image_name = temp_strs[1];

    AINFO << "image: " << image_name << " camera_name:" << camera_name;
    std::string image_path = FLAGS_image_root + "/" + camera_name + "/" +
                             image_name + FLAGS_image_ext;
    CameraFrame &frame = frames[frame_num];

    // read detections from txt
    const std::string filename =
        absl::StrCat(FLAGS_data_root, "/detection_feature/", frame_num, ".txt");
    read_detections(filename, FLAGS_feature_length, camera_name, &frame);
    AINFO << "Frame " << frame_num << " has " << frame.detected_objects.size()
          << " detection objects";
    frame.frame_id = frame_num;
    std::stringstream ss(image_name);
    frame.timestamp = 0.0;
    ss >> frame.timestamp;
    frame.timestamp *= 1e-9;
    AINFO << "Timestamp: " << frame.timestamp;
    Eigen::Matrix3d project_matrix;
    double pitch_diff = 0.0;

    ASSERT_TRUE(GetProjectMatrix(camera_name, extrinsic_map, intrinsic_map,
                                 &project_matrix, &pitch_diff));
    frame.project_matrix = project_matrix;
    frame.data_provider = name_provider_map.at(camera_name);
    AINFO << "Project Matrix: \n" << frame.project_matrix;
    ASSERT_TRUE(camera_tracker->Predict(ObstacleTrackerOptions(), &frame));
    ASSERT_TRUE(camera_tracker->Associate2D(ObstacleTrackerOptions(), &frame));
    ASSERT_TRUE(camera_tracker->Associate3D(ObstacleTrackerOptions(), &frame));
    ASSERT_TRUE(camera_tracker->Track(ObstacleTrackerOptions(), &frame));
    AINFO << "Frame " << frame_num
          << " tracked object size: " << frame.tracked_objects.size();
    for (auto &bbox_gt : det_gts[frame_num]) {
      int id = -1;
      float max_iou = 0.0f;
      Eigen::Matrix<double, 3, 1> center0, size0;
      center0[0] = bbox_gt.box.Center().x;
      size0[0] = bbox_gt.box.xmax - bbox_gt.box.xmin;
      center0[1] = bbox_gt.box.Center().y;
      size0[1] = bbox_gt.box.ymax - bbox_gt.box.ymin;
      base::CameraObjectSupplement bbox;
      for (int i = 0; i < frames[frame_num].tracked_objects.size(); i++) {
        Eigen::Matrix<double, 3, 1> center1, size1;
        base::BBox2DF temp_box;
        temp_box = frames[frame_num].tracked_objects[i]->camera_supplement.box;
        center1[0] = temp_box.Center().x;
        size1[0] = temp_box.xmax - temp_box.xmin;
        center1[1] = temp_box.Center().y;
        size1[1] = temp_box.ymax - temp_box.ymin;
        float iou = common::CalculateIou2DXY(center0, size0, center1, size1);
        AINFO << "IOU is :" << iou;
        if (iou > max_iou) {
          max_iou = iou;
          id = i;
          bbox.local_track_id = frames[frame_num].tracked_objects[i]->track_id;
        }
      }
      if (frame_num >= dynamic_cast<OMTObstacleTracker *>(camera_tracker.get())
                           ->omt_param_.target_param()
                           .tracked_life()) {
        ASSERT_GE(max_iou, 0.5);
      }
      tracked_index[frame_num].push_back(id);
      tracked_results[frame_num].push_back(bbox);
    }
    ++frame_num;
  }
  std::vector<int> ids(2, -100);
  for (frame_num = dynamic_cast<OMTObstacleTracker *>(camera_tracker.get())
                       ->omt_param_.target_param()
                       .tracked_life();
       frame_num < det_gts.size(); ++frame_num) {
    ASSERT_GE(tracked_results[frame_num].size(), 1);
    for (int i = 0; i < det_gts[frame_num].size(); ++i) {
      if (ids[i] < -50) {
        ids[i] = det_gts[frame_num][i].local_track_id -
                 tracked_results[frame_num][i].local_track_id;
      } else {
        ASSERT_EQ(ids[i], det_gts[frame_num][i].local_track_id -
                              tracked_results[frame_num][i].local_track_id);
      }
    }
  }
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
