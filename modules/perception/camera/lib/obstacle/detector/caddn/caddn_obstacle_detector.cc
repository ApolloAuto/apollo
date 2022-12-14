/******************************************************************************
 * Copyright 2022 The Apollo Authors. All Rights Reserved.
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
#include "modules/perception/camera/lib/obstacle/detector/caddn/caddn_obstacle_detector.h"

#include <algorithm>
#include <functional>

#include "modules/perception/camera/common/timer.h"
#include "modules/perception/common/perception_gflags.h"

namespace apollo {
namespace perception {
namespace camera {

// CaddnObstacleDetector::~CaddnObstacleDetector() {}

bool CaddnObstacleDetector::Init(const StageConfig &stage_config) {
  if (!Initialize(stage_config)) {
    return false;
  }
  ACHECK(stage_config.has_camera_detector_config());

  std::string shape_range_info =
      "/apollo/modules/perception/production/data/perception/camera/models/"
      "caddn/shape_range_info.pbtxt";

  if (gpu_id_ >= 0) {
    config_.EnableUseGpu(100, 0);
  }

  config_.SetModel(FLAGS_caddn_model_file, FLAGS_caddn_params_file);
  config_.EnableMemoryOptim();
  // tensorRT

  if (FLAGS_use_trt) {
    paddle::AnalysisConfig::Precision precision;
    if (FLAGS_trt_precision == 0) {
      precision = paddle_infer::PrecisionType::kFloat32;
    } else if (FLAGS_trt_precision == 1) {
      precision = paddle_infer::PrecisionType::kHalf;
    } else {
      AERROR << "Tensorrt type can only support 0 or 1, but recieved is"
             << FLAGS_trt_precision << "\n";
      return false;
    }
    config_.EnableTensorRtEngine(1 << 30, 1, 12, precision,
                                 FLAGS_trt_use_static, false);
    // config.CollectShapeRangeInfo(FLAGS_dynamic_shape_file);
    //  config.EnableTunedTensorRtDynamicShape(FLAGS_dynamic_shape_file, true);
    if (FLAGS_trt_use_static) {
      config_.SetOptimCacheDir(FLAGS_trt_static_dir);
    }
  }
  config_.SwitchIrOptim(false);
  predictor_ = paddle_infer::CreatePredictor(config_);
  if (nullptr == predictor_) {
    return false;
  }

  return true;
}

bool CaddnObstacleDetector::Init(const ObstacleDetectorInitOptions &options) {
  return true;
}

bool CaddnObstacleDetector::Detect(const ObstacleDetectorOptions &options,
                                   CameraFrame *frame) {
  return true;
}

bool CaddnObstacleDetector::Process(DataFrame *data_frame) {
  if (nullptr == data_frame) {
    return false;
  }

  auto frame = data_frame->camera_frame;
  Timer timer;

  const auto &camera_k_matrix = frame->camera_k_matrix;

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) std::cout << camera_k_matrix(i, j) << " ";
  }
  std::cout << "\n";

  std::vector<float> input_cam_intrinsic(12);

  for (size_t i = 0; i < 3; ++i) {
    size_t i3 = i * 4;
    for (size_t j = 0; j < 4; ++j) {
      if (3 == j) {
        input_cam_intrinsic.at(i3 + j) = 0.0;

      } else if (frame->data_provider->sensor_name() == "front_12mm") {
        input_cam_intrinsic.at(i3 + j) = camera_k_matrix(i, j) * 2.f;

      } else {
        input_cam_intrinsic.at(i3 + j) = camera_k_matrix(i, j);
      }
    }
  }

  AINFO << "Start: " << static_cast<double>(timer.Toc()) * 0.001 << "ms";
  DataProvider::ImageOptions image_options;
  image_options.target_color = base::Color::BGR;

  std::shared_ptr<base::Image8U> image_temp = nullptr;
  image_temp.reset(
      new base::Image8U(image_height_, image_width_, base::Color::BGR));

  frame->data_provider->GetImage(image_options, image_temp.get());

  AINFO << "GetImageBlob: " << static_cast<double>(timer.Toc()) * 0.001 << "ms";

  cv::Mat output_image(1080, 1920, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::Mat img_resized;
  std::vector<float> input_image_data(1 * 3 * height_ * width_, 0.0f);
  memcpy(output_image.data, image_temp->cpu_data(),
         image_temp->total() * sizeof(uint8_t));

  cv::resize(output_image, img_resized, cv::Size(width_, height_), 0, 0,
             cv::INTER_LINEAR);

  Normalize(&img_resized, mean_val_, std_val_, scale_);

  Mat2Vec(&img_resized, input_image_data.data());

  AINFO << "Resize: " << static_cast<double>(timer.Toc()) * 0.001 << "ms";

  AINFO << "Camera type: " << frame->data_provider->sensor_name();
  /////////////////////////// detection part ///////////////////////////

  std::vector<int> input_image_shape = {1, 3, height_, width_};
  std::vector<float> out_detections;
  std::vector<float> out_labels;
  std::vector<float> out_scores;
  std::vector<float> out_detections_final;
  std::vector<float> out_labels_final;

  for (auto i : input_lidar_data_) {
    std::cout << i << "\n";
  }

  for (auto i : input_lidar_shape_) {
    std::cout << i << "\n";
  }

  Run(predictor_.get(), input_image_shape, input_image_data, input_cam_shape_,
      input_cam_intrinsic, input_lidar_shape_, input_lidar_data_,
      &out_detections, &out_labels, &out_scores);

  AINFO << "Network Forward: " << static_cast<double>(timer.Toc()) * 0.001
        << "ms";
  GetCaddnObjects(&(frame->detected_objects), frame->data_provider->src_width(),
                  frame->data_provider->src_height() - offset_y_,
                  &out_detections, &out_labels, &out_scores);

  AINFO << "GetObj: " << static_cast<double>(timer.Toc()) * 0.001 << "ms";

  FeatureExtractorOptions feature_options;
  feature_options.normalized = true;
  AINFO << "Post1: " << static_cast<double>(timer.Toc()) * 0.001 << "ms";

  AINFO << "Extract: " << static_cast<double>(timer.Toc()) * 0.001 << "ms";

  RecoverCaddnBbox(frame->data_provider->src_width(),
                   frame->data_provider->src_height() - offset_y_, offset_y_,
                   &frame->detected_objects);

  AINFO << "Post2: " << static_cast<double>(timer.Toc()) * 0.001 << "ms";

  return true;
}
void CaddnObstacleDetector::Normalize(cv::Mat *im,
                                      const std::vector<float> &mean,
                                      const std::vector<float> &std,
                                      float scale) {
  if (scale) {
    (*im).convertTo(*im, CV_32FC3, scale);
  }
  for (int h = 0; h < im->rows; h++) {
    for (int w = 0; w < im->cols; w++) {
      im->at<cv::Vec3f>(h, w)[0] =
          (im->at<cv::Vec3f>(h, w)[0] - mean[0]) / std[0];
      im->at<cv::Vec3f>(h, w)[1] =
          (im->at<cv::Vec3f>(h, w)[1] - mean[1]) / std[1];
      im->at<cv::Vec3f>(h, w)[2] =
          (im->at<cv::Vec3f>(h, w)[2] - mean[2]) / std[2];
    }
  }
}

void CaddnObstacleDetector::Run(
    paddle_infer::Predictor *predictor, const std::vector<int> &images_shape,
    const std::vector<float> &images_data, const std::vector<int> &cam_shape,
    const std::vector<float> &cam_data, const std::vector<int> &lidar_shape,
    const std::vector<float> &lidar_data, std::vector<float> *boxes,
    std::vector<float> *labels, std::vector<float> *scores) {
  auto input_names = predictor->GetInputNames();
  for (const auto &tensor_name : input_names) {
    auto in_tensor = predictor->GetInputHandle(tensor_name);
    if (tensor_name == "images") {
      in_tensor->Reshape(images_shape);
      in_tensor->CopyFromCpu(images_data.data());
    } else if (tensor_name == "trans_cam_to_img") {
      in_tensor->Reshape(cam_shape);
      in_tensor->CopyFromCpu(cam_data.data());
    } else if (tensor_name == "trans_lidar_to_cam") {
      in_tensor->Reshape(lidar_shape);
      in_tensor->CopyFromCpu(lidar_data.data());
    }
  }

  CHECK(predictor->Run());

  auto output_names = predictor->GetOutputNames();
  for (size_t i = 0; i != output_names.size(); i++) {
    auto output = predictor->GetOutputHandle(output_names[i]);
    std::vector<int> output_shape = output->shape();
    int out_num = std::accumulate(output_shape.begin(), output_shape.end(), 1,
                                  std::multiplies<int>());
    if (i == 0) {
      boxes->resize(out_num);
      output->CopyToCpu(boxes->data());
    } else if (i == 1) {
      labels->resize(out_num);
      output->CopyToCpu(labels->data());
    } else if (i == 2) {
      scores->resize(out_num);
      output->CopyToCpu(scores->data());
    }
  }
}

void CaddnObstacleDetector::GetCaddnObjects(
    std::vector<base::ObjectPtr> *objects, int width, int height,
    const std::vector<float> *boxes, const std::vector<float> *labels,
    const std::vector<float> *scores) {
  objects->clear();

  int len_pred = 7;
  for (size_t i = 0; i < labels->size(); i++) {
    const float *bbox = boxes->data() + i * len_pred;
    float score = *(scores->data() + i);
    if (score < FLAGS_score_threshold) {
      continue;
    }

    float label = *(labels->data() + i);
    base::ObjectPtr obj = nullptr;
    obj.reset(new base::Object);
    obj->sub_type = GetCaddnObjectSubtype(label);
    obj->type = base::kSubType2TypeMap.at(obj->sub_type);
    obj->type_probs.assign(static_cast<int>(base::ObjectType::MAX_OBJECT_TYPE),
                           0);
    obj->sub_type_probs.assign(
        static_cast<int>(base::ObjectSubType::MAX_OBJECT_TYPE), 0);
    obj->type_probs[static_cast<int>(obj->type)] = score;
    obj->sub_type_probs[static_cast<int>(obj->sub_type)] = score;
    obj->confidence = score;

    Eigen::Matrix<float, 3, 4> velo_to_cam;
    velo_to_cam << 0.00753374, -0.9999714, -0.0006166, -0.00406977, 0.01480249,
        0.00072807, -0.9998902, -0.07631618, 0.9998621, 0.00752379, 0.01480755,
        -0.2717806;

    Eigen::Matrix<float, 3, 3> R;
    R << 0.9999239, 0.00983776, -0.00744505, -0.0098698, 0.9999421, -0.00427846,
        0.00740253, 0.00435161, 0.9999631;

    std::vector<float> test_result(7);
    Bbox3dLidar2Camera(velo_to_cam, R, bbox, &test_result);

    for (auto i : test_result) {
      std::cout << i << " " << std::flush;
    }
    std::cout << std::endl;

    std::vector<float> bbox2d_result;
    Bbox3d2Bbox2d(camera_k_matrix_, test_result.data(), &bbox2d_result);
    for (auto i : bbox2d_result) {
      std::cout << i << " " << std::flush;
    }
    std::cout << std::endl;

    FillCaddnBase(obj, bbox2d_result.data(), width, height);
    FillCaddnBbox3d(obj, test_result.data());

    objects->push_back(obj);
  }
}

void CaddnObstacleDetector::Bbox3dLidar2Camera(
    const Eigen::Matrix<float, 3, 4> &V2C, const Eigen::Matrix<float, 3, 3> &R,
    const float *bbox_lidar, std::vector<float> *bbox_camera) {
  float x = *(bbox_lidar + 0);
  float y = *(bbox_lidar + 1);
  float z = *(bbox_lidar + 2);
  float l = *(bbox_lidar + 3);
  float w = *(bbox_lidar + 4);
  float h = *(bbox_lidar + 5);
  float r = *(bbox_lidar + 6);
  r = -r - M_PI / 2.0;
  // 4*1
  Eigen::Vector4f xyz_lidar;
  xyz_lidar << x, y, z - h / 2.0, 1.0;
  Eigen::Matrix<float, 1, 3> pts_rect =
      xyz_lidar.transpose() * (V2C.transpose() * R.transpose());
  std::vector<float> final_result{
      pts_rect(0), pts_rect(1), pts_rect(2), l, h, w, r};
  bbox_camera->assign(final_result.data(),
                      final_result.data() + final_result.size());
}

void CaddnObstacleDetector::Bbox3d2Bbox2d(const Eigen::Matrix<float, 3, 3> &K,
                                          const float *bbox3d_camera,
                                          std::vector<float> *bbox_2d) {
  float x = *(bbox3d_camera + 0);
  float y = *(bbox3d_camera + 1);
  float z = *(bbox3d_camera + 2);
  float dx = *(bbox3d_camera + 3);
  float dy = *(bbox3d_camera + 4);
  float dz = *(bbox3d_camera + 5);
  float alpha = *(bbox3d_camera + 6);

  Eigen::Vector3f center_3d(x, y, z);
  Eigen::Matrix3f R;
  R << std::cos(alpha), 0, std::sin(alpha), 0, 1, 0, -std::sin(alpha), 0,
      std::cos(alpha);

  Eigen::Vector3f h_w_l_1(dx, dy, dz);
  Eigen::Vector3f h_w_l_2(-dx, dy, dz);
  Eigen::Vector3f h_w_l_3(dx, -dy, dz);
  Eigen::Vector3f h_w_l_4(dx, dy, -dz);
  Eigen::Vector3f h_w_l_5(dx, -dy, -dz);
  Eigen::Vector3f h_w_l_6(-dx, dy, -dz);
  Eigen::Vector3f h_w_l_7(-dx, -dy, dz);
  Eigen::Vector3f h_w_l_8(-dx, -dy, -dz);

  Eigen::Vector3f a1 = R * h_w_l_1 / 2 + center_3d;
  Eigen::Vector3f a2 = R * h_w_l_2 / 2 + center_3d;
  Eigen::Vector3f a3 = R * h_w_l_3 / 2 + center_3d;
  Eigen::Vector3f a4 = R * h_w_l_4 / 2 + center_3d;
  Eigen::Vector3f a5 = R * h_w_l_5 / 2 + center_3d;
  Eigen::Vector3f a6 = R * h_w_l_6 / 2 + center_3d;
  Eigen::Vector3f a7 = R * h_w_l_7 / 2 + center_3d;
  Eigen::Vector3f a8 = R * h_w_l_8 / 2 + center_3d;

  a1 = K * a1;
  a2 = K * a2;
  a3 = K * a3;
  a4 = K * a4;
  a5 = K * a5;
  a6 = K * a6;
  a7 = K * a7;
  a8 = K * a8;

  a1 = a1 / a1(2);
  a2 = a2 / a2(2);
  a3 = a3 / a3(2);
  a4 = a4 / a4(2);
  a5 = a5 / a5(2);
  a6 = a6 / a6(2);
  a7 = a7 / a7(2);
  a8 = a7 / a7(2);
  std::vector<float> x_value = {a1(0), a2(0), a3(0), a4(0),
                                a5(0), a6(0), a7(0), a8(0)};
  std::vector<float> y_value = {a1(1), a2(1), a3(1), a4(1),
                                a5(1), a6(1), a7(1), a8(1)};

  float x_min = *std::min_element(x_value.begin(), x_value.end());
  float y_min = *std::min_element(y_value.begin(), y_value.end());
  float x_max = *std::max_element(x_value.begin(), x_value.end());
  float y_max = *std::max_element(y_value.begin(), y_value.end());
  std::vector<float> bbox2d_result = {x_min, y_min, x_max, y_max};
  bbox_2d->assign(bbox2d_result.begin(), bbox2d_result.end());
}

void CaddnObstacleDetector::FillCaddnBase(base::ObjectPtr obj,
                                          const float *bbox, int width,
                                          int height) {
  obj->camera_supplement.box.xmin = bbox[0] / width;
  obj->camera_supplement.box.ymin = bbox[1] / height;
  obj->camera_supplement.box.xmax = bbox[2] / width;
  obj->camera_supplement.box.ymax = bbox[3] / height;
}

void CaddnObstacleDetector::FillCaddnBbox3d(base::ObjectPtr obj,
                                            const float *bbox) {
  obj->camera_supplement.alpha = bbox[6];
  obj->size[2] = bbox[3];
  obj->size[1] = bbox[4];
  obj->size[0] = bbox[5];

  obj->camera_supplement.local_center[0] = bbox[0];
  obj->camera_supplement.local_center[1] = bbox[1];
  obj->camera_supplement.local_center[2] = bbox[2];
}

void CaddnObstacleDetector::RecoverCaddnBbox(
    int roi_w, int roi_h, int offset_y, std::vector<base::ObjectPtr> *objects) {
  for (auto &obj : *objects) {
    float xmin = obj->camera_supplement.box.xmin;
    float ymin = obj->camera_supplement.box.ymin;
    float xmax = obj->camera_supplement.box.xmax;
    float ymax = obj->camera_supplement.box.ymax;
    float x = xmin * static_cast<float>(roi_w);
    float w = (xmax - xmin) * static_cast<float>(roi_w);
    float y = ymin * static_cast<float>(roi_h) + static_cast<float>(offset_y);
    float h = (ymax - ymin) * static_cast<float>(roi_h);
    base::RectF rect_det(x, y, w, h);
    base::RectF rect_img(0, 0, static_cast<float>(roi_w),
                         static_cast<float>(roi_h + offset_y));
    base::RectF rect = rect_det & rect_img;
    obj->camera_supplement.box = rect;

    double eps = 1e-2;

    // Truncation assignment based on bbox positions
    if ((ymin < eps) || (ymax >= (1.0 - eps))) {
      obj->camera_supplement.truncated_vertical = 0.5;
    } else {
      obj->camera_supplement.truncated_vertical = 0.0;
    }
    if ((xmin < eps) || (xmax >= (1.0 - eps))) {
      obj->camera_supplement.truncated_horizontal = 0.5;
    } else {
      obj->camera_supplement.truncated_horizontal = 0.0;
    }
  }
}

base::ObjectSubType CaddnObstacleDetector::GetCaddnObjectSubtype(int cls) {
  if (cls == 1) {
    return base::ObjectSubType::CAR;
  } else if (cls == 2) {
    return base::ObjectSubType::PEDESTRIAN;
  } else if (cls == 3) {
    return base::ObjectSubType::CYCLIST;
  } else {
    return base::ObjectSubType::UNKNOWN;
  }
}

void CaddnObstacleDetector::Mat2Vec(const cv::Mat *im, float *data) {
  int rh = im->rows;
  int rw = im->cols;
  int rc = im->channels();
  for (int i = 0; i < rc; ++i) {
    cv::extractChannel(*im, cv::Mat(rh, rw, CV_32FC1, data + i * rh * rw), i);
  }
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
