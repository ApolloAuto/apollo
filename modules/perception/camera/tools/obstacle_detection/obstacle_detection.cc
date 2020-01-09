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
#include <opencv2/opencv.hpp>

#include <fstream>

#include "gtest/gtest.h"

#include "modules/perception/base/distortion_model.h"
#include "modules/perception/base/object_types.h"
#include "modules/perception/camera/lib/interface/base_obstacle_detector.h"
#include "modules/perception/camera/lib/interface/base_obstacle_transformer.h"
#include "modules/perception/common/io/io_util.h"

DEFINE_string(test_list, "full_test_list.txt", "test image list");
DEFINE_string(image_root, "", "root dir of images");
DEFINE_string(image_ext, ".jpg", "extension of image name");
DEFINE_string(dest_dir, "/tmp", "output dir");
DEFINE_string(vis_dir, "", "output dir");
DEFINE_string(pre_detected_dir, "", "pre-detected obstacles (skip Detect)");
DEFINE_int32(height, 1080, "image height");
DEFINE_int32(width, 1920, "image width");
DEFINE_string(detector, "YoloObstacleDetector", "detector");
DEFINE_string(transformer, "MultiCueObstacleTransformer", "transformer");
DEFINE_string(detector_root, "./data/yolo", "detector data root");
DEFINE_string(detector_conf, "config.pt", "detector config");
DEFINE_string(transformer_root, "./data/multicue", "transformer data root");
DEFINE_string(transformer_conf, "config.pt", "transformer config");
DEFINE_string(dist_type, "", "dist pred type: H-on-h, H-from-h");

namespace apollo {
namespace perception {
namespace camera {

static const cv::Scalar kBoxColorMap[] = {
    cv::Scalar(0, 0, 0),        // 0
    cv::Scalar(128, 128, 128),  // 1
    cv::Scalar(255, 0, 0),      // 2
    cv::Scalar(0, 255, 0),      // 3
    cv::Scalar(0, 0, 255),      // 4
    cv::Scalar(255, 255, 0),    // 5
    cv::Scalar(0, 255, 255),    // 6
    cv::Scalar(255, 0, 255),    // 7
    cv::Scalar(255, 255, 255),  // 8
};

static const cv::Scalar kFaceColorMap[] = {
    cv::Scalar(255, 255, 255),  // 0
    cv::Scalar(255, 0, 0),      // 1
    cv::Scalar(0, 255, 0),      // 2
    cv::Scalar(0, 0, 255),      // 3
};

base::ObjectSubType GetObjectSubType(const std::string &type_name) {
  if (type_name == "car") {
    return base::ObjectSubType::CAR;
  } else if (type_name == "van") {
    return base::ObjectSubType::VAN;
  } else if (type_name == "bus") {
    return base::ObjectSubType::BUS;
  } else if (type_name == "truck") {
    return base::ObjectSubType::TRUCK;
  } else if (type_name == "cyclist") {
    return base::ObjectSubType::CYCLIST;
  } else if (type_name == "motorcyclist") {
    return base::ObjectSubType::MOTORCYCLIST;
  } else if (type_name == "tricyclelist") {
    return base::ObjectSubType::TRICYCLIST;
  } else if (type_name == "pedestrian") {
    return base::ObjectSubType::PEDESTRIAN;
  } else if (type_name == "trafficcone") {
    return base::ObjectSubType::TRAFFICCONE;
  } else {
    // type_name is "" or unknown
    return base::ObjectSubType::UNKNOWN;
  }
}

bool LoadFromKitti(const std::string &kitti_path, CameraFrame *frame) {
  frame->detected_objects.clear();
  FILE *fp = fopen(kitti_path.c_str(), "r");
  if (fp == nullptr) {
    AERROR << "Failed to load object file: " << kitti_path;
    return false;
  }
  while (!feof(fp)) {
    base::ObjectPtr obj = nullptr;
    obj.reset(new base::Object);
    float trash = 0.0f;
    float score = 0.0f;
    char type[255];
    float x1 = 0.0f;
    float y1 = 0.0f;
    float x2 = 0.0f;
    float y2 = 0.0f;
    memset(type, 0, sizeof(type));

    int ret = 0;
    ret = fscanf(fp, "%254s %f %f %lf %f %f %f %f %f %f %f %lf %lf %lf %f %f",
                 type, &trash, &trash, &obj->camera_supplement.alpha, &x1, &y1,
                 &x2, &y2, &obj->size[2], &obj->size[1], &obj->size[0],
                 &obj->center[0], &obj->center[1], &obj->center[2], &obj->theta,
                 &score);
    AINFO << "fscanf return: " << ret;
    if (FLAGS_dist_type == "H-from-h") {
      obj->size[0] = static_cast<float>(obj->center[2]);
    } else if (FLAGS_dist_type == "H-on-h") {
      obj->size[0] = static_cast<float>(obj->center[2]) * (y2 - y1);
    }
    obj->camera_supplement.box.xmin = std::max<float>(x1, 0);
    obj->camera_supplement.box.ymin = std::max<float>(y1, 0);
    obj->camera_supplement.box.xmax =
        std::min<float>(x2, static_cast<float>(FLAGS_width));
    obj->camera_supplement.box.ymax =
        std::min<float>(y2, static_cast<float>(FLAGS_height));
    obj->camera_supplement.area_id = 5;

    obj->sub_type = GetObjectSubType(type);
    obj->type = base::kSubType2TypeMap.at(obj->sub_type);
    obj->type_probs.assign(static_cast<int>(base::ObjectType::MAX_OBJECT_TYPE),
                           0);
    obj->sub_type_probs.assign(
        static_cast<int>(base::ObjectSubType::MAX_OBJECT_TYPE), 0);
    obj->sub_type_probs[static_cast<int>(obj->sub_type)] = score;
    obj->type_probs[static_cast<int>(obj->type)] = score;

    frame->detected_objects.push_back(obj);
  }
  fclose(fp);
  return true;
}

int main() {
  CameraFrame frame;
  DataProvider data_provider;
  frame.data_provider = &data_provider;
  if (frame.track_feature_blob == nullptr) {
    frame.track_feature_blob.reset(new base::Blob<float>());
  }

  DataProvider::InitOptions dp_init_options;
  dp_init_options.sensor_name = "front_6mm";

  dp_init_options.image_height = FLAGS_height;
  dp_init_options.image_width = FLAGS_width;
  dp_init_options.device_id = 0;

  AINFO << "Init DataProvider ...";
  CHECK(frame.data_provider->Init(dp_init_options));
  AINFO << "Done!";

  ObstacleDetectorInitOptions init_options;
  init_options.root_dir = FLAGS_detector_root;
  init_options.conf_file = FLAGS_detector_conf;

  base::BrownCameraDistortionModel model;
  common::LoadBrownCameraIntrinsic("params/front_6mm_intrinsics.yaml", &model);
  init_options.base_camera_model = model.get_camera_model();
  auto pinhole =
      static_cast<base::PinholeCameraModel *>(model.get_camera_model().get());
  Eigen::Matrix3f intrinsic = pinhole->get_intrinsic_params();
  frame.camera_k_matrix = intrinsic;

  AINFO << "Init Detector ...";
  BaseObstacleDetector *detector =
      BaseObstacleDetectorRegisterer::GetInstanceByName(FLAGS_detector);
  if (FLAGS_pre_detected_dir == "") {
    CHECK_EQ(detector->Name(), FLAGS_detector);
    CHECK(detector->Init(init_options));
  }
  AINFO << "Done!";

  AINFO << "Init Transformer ...";
  ObstacleTransformerInitOptions transformer_init_options;
  transformer_init_options.root_dir = FLAGS_transformer_root;
  transformer_init_options.conf_file = FLAGS_transformer_conf;

  BaseObstacleTransformer *transformer =
      BaseObstacleTransformerRegisterer::GetInstanceByName(FLAGS_transformer);
  CHECK_EQ(transformer->Name(), FLAGS_transformer);
  CHECK(transformer->Init(transformer_init_options));
  AINFO << "Done!";

  ObstacleDetectorOptions options;
  ObstacleTransformerOptions transformer_options;

  // load image list
  std::ifstream fin;
  fin.open(FLAGS_test_list, std::ifstream::in);
  if (!fin.is_open()) {
    AERROR << "Cannot open test list: " << FLAGS_test_list;
    return -1;
  }

  std::string image_name;
  while (fin >> image_name) {
    AINFO << "image: " << image_name;
    std::string image_path =
        FLAGS_image_root + "/" + image_name + FLAGS_image_ext;
    std::string result_path = FLAGS_dest_dir + "/" + image_name + ".txt";

    auto cv_img = cv::imread(image_path, CV_LOAD_IMAGE_COLOR);

    if (FLAGS_pre_detected_dir != "") {
      std::string kitti_path =
          FLAGS_pre_detected_dir + "/" + image_name + ".txt";
      if (!LoadFromKitti(kitti_path, &frame)) {
        AINFO << "loading kitti result failed: " << kitti_path;
        continue;
      }
    } else {
      base::Image8U image(cv_img.rows, cv_img.cols, base::Color::BGR);

      for (int y = 0; y < cv_img.rows; ++y) {
        memcpy(image.mutable_cpu_ptr(y), cv_img.ptr<uint8_t>(y),
               image.width_step());
      }

      CHECK(frame.data_provider->FillImageData(cv_img.rows, cv_img.cols,
                                               image.gpu_data(), "bgr8"));

      EXPECT_TRUE(detector->Detect(options, &frame));
    }
    EXPECT_TRUE(transformer->Transform(transformer_options, &frame));

    FILE *fp = fopen(result_path.c_str(), "w");
    if (fp == nullptr) {
      AINFO << "Failed to open result path: " << result_path;
      return -1;
    }
    int obj_id = 0;
    for (auto obj : frame.detected_objects) {
      auto &supp = obj->camera_supplement;
      auto &box = supp.box;
      auto area_id = supp.area_id;
      fprintf(fp,
              "%s 0 0 %6.3f %8.2f %8.2f %8.2f %8.2f %6.3f %6.3f %6.3f "
              "%6.3f %6.3f %6.3f %6.3f %6.3f "
              "%4d %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f\n",
              base::kSubType2NameMap.at(obj->sub_type).c_str(), supp.alpha,
              supp.box.xmin, supp.box.ymin, supp.box.xmax, supp.box.ymax,
              obj->size[2], obj->size[1], obj->size[0], obj->center[0],
              obj->center[1] + obj->size[2] * .5, obj->center[2],
              supp.alpha + atan2(obj->center[0], obj->center[2]),
              obj->type_probs[static_cast<int>(obj->type)], area_id,
              supp.visible_ratios[0], supp.visible_ratios[1],
              supp.visible_ratios[2], supp.visible_ratios[3],
              supp.cut_off_ratios[0], supp.cut_off_ratios[1],
              supp.cut_off_ratios[2], supp.cut_off_ratios[3]);
      if (FLAGS_vis_dir != "") {
        cv::rectangle(
            cv_img,
            cv::Point(static_cast<int>(box.xmin), static_cast<int>(box.ymin)),
            cv::Point(static_cast<int>(box.xmax), static_cast<int>(box.ymax)),
            cv::Scalar(0, 0, 0), 8);
        float xmid = (box.xmin + box.xmax) / 2;
        CHECK(area_id > 0 && area_id < 9);
        if (area_id & 1) {
          cv::rectangle(
              cv_img,
              cv::Point(static_cast<int>(box.xmin), static_cast<int>(box.ymin)),
              cv::Point(static_cast<int>(box.xmax), static_cast<int>(box.ymax)),
              kFaceColorMap[area_id / 2], 2);
        } else {
          auto &tl = supp.cut_off_ratios[2];
          auto &tr = supp.cut_off_ratios[3];
          auto &&left_ratio = supp.visible_ratios[(area_id / 2) % 4];
          auto w = box.xmax - box.xmin;
          auto x = box.xmin;
          auto tm = std::max(tl, tr);
          if (tm > 1e-2) {
            if (tl > tr) {
              xmid = (x - w * tl) + (w + w * tl) * left_ratio;
            } else if (tl < tr) {
              xmid = x + (w + w * tr) * left_ratio;
            }
          } else {
            xmid = x + w * left_ratio;
          }
          cv::rectangle(
              cv_img,
              cv::Point(static_cast<int>(box.xmin), static_cast<int>(box.ymin)),
              cv::Point(static_cast<int>(xmid), static_cast<int>(box.ymax)),
              kFaceColorMap[(area_id / 2) % 4], 3);
          cv::rectangle(
              cv_img,
              cv::Point(static_cast<int>(xmid), static_cast<int>(box.ymin)),
              cv::Point(static_cast<int>(box.xmax), static_cast<int>(box.ymax)),
              kFaceColorMap[area_id / 2 - 1], 2);
        }
        fprintf(stderr,
                "obj-%02d: %.3f %.3f %.3f %.3f -- %.3f %.3f %.3f %.3f "
                "-- %.0f %.0f %.0f %d\n",
                obj_id, supp.visible_ratios[0], supp.visible_ratios[1],
                supp.visible_ratios[2], supp.visible_ratios[3],
                supp.cut_off_ratios[0], supp.cut_off_ratios[1],
                supp.cut_off_ratios[2], supp.cut_off_ratios[3], box.xmin, xmid,
                box.xmax, area_id);
        std::stringstream text;
        auto &name = base::kSubType2NameMap.at(obj->sub_type);
        text << name[0] << name[1] << name[2] << " - " << obj_id++;
        cv::putText(
            cv_img, text.str(),
            cv::Point(static_cast<int>(box.xmin), static_cast<int>(box.ymin)),
            cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 0, 0), 2);
      }
    }
    if (FLAGS_vis_dir != "") {
      auto vis_path = FLAGS_vis_dir + "/" + image_name + ".jpg";
      cv::imwrite(vis_path.c_str(), cv_img);
    }
    fclose(fp);
  }
  delete transformer;
  delete detector;
  return 0;
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo

int main(int argc, char *argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::SetUsageMessage(
      "command line brew\n"
      "Usage: camera_benchmark <args>\n");
  return apollo::perception::camera::main();
}
