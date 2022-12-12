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
#include "modules/perception/camera/lib/obstacle/detector/bev_detection/bev_obstacle_detector.h"

#include <opencv2/opencv.hpp>

#include "modules/perception/camera/common/timer.h"
#include "modules/perception/common/perception_gflags.h"

namespace apollo {
namespace perception {
namespace camera {

bool BEVObstacleDetector::Init(const ObstacleDetectorInitOptions &options) {
  return true;
}

bool BEVObstacleDetector::Detect(const ObstacleDetectorOptions &options,
                                 CameraFrame *frame) {
  return true;
}

bool BEVObstacleDetector::Init(const StageConfig &stage_config) {
  AERROR << "wxt debug: into BEVObstacleDetector::Init";
  if (!Initialize(stage_config)) {
    return false;
  }
  // ACHECK(stage_config.has_bev_obstacle_detection_config());
  bev_obstacle_detection_config_ = stage_config.bev_obstacle_detection_config();

  paddle::AnalysisConfig config;
  config.EnableUseGpu(1000, FLAGS_gpu_id);
  config.SetModel(
      "modules/perception/production/data/perception/camera/models/petr_v1/"
      "petr_inference.pdmodel",
      "modules/perception/production/data/perception/camera/models/petr_v1/"
      "petr_inference.pdiparams");
  config.EnableMemoryOptim();
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
    config.EnableTensorRtEngine(1 << 30, 1, 12, precision, FLAGS_trt_use_static,
                                false);
    config.CollectShapeRangeInfo(FLAGS_dynamic_shape_file);
    // config.EnableTunedTensorRtDynamicShape(FLAGS_dynamic_shape_file, true);
    if (FLAGS_trt_use_static) {
      config.SetOptimCacheDir(FLAGS_trt_static_dir);
    }
  }
  config.SwitchIrOptim(true);
  predictor_ = paddle_infer::CreatePredictor(config);
  AERROR << "wxt debug: out BEVObstacleDetector::Init";
  if (nullptr == predictor_) {
    return false;
  }
  return true;
}

bool BEVObstacleDetector::Process(DataFrame *data_frame) {
  DataProvider::ImageOptions image_options;
  image_options.target_color = base::Color::BGR;
  /*
  image_options.crop_roi = base::RectI(
      0, offset_y_, static_cast<int>(base_camera_model_->get_width()),
      static_cast<int>(base_camera_model_->get_height()) - offset_y_);
  image_options.do_crop = true;
  */

  Timer timer;
  float scale = 1.0f;
  for (int i = 0; i < 6; ++i) {
    const auto camera_frame_temp = (data_frame + i)->camera_frame;

    std::shared_ptr<base::Image8U> image_temp = nullptr;
    camera_frame_temp->data_provider->GetImage(image_options, image_temp.get());

    cv::Mat image_mat(image_height_, image_width_, CV_8UC3,
                      cv::Scalar(0, 0, 0));
    memcpy(image_mat.data, image_temp->cpu_data(),
           image_temp->total() * sizeof(uint8_t));

    // deal image data
    cv::Mat mat_resized_tmp;
    Resize(image_mat, image_height_resized_, image_width_resized_,
           &mat_resized_tmp);

    // cv::imwrite("/apollo/test.jpg",mat_resized_tmp);
    cv::Mat mat_crop_tmp =
        mat_resized_tmp(cv::Range(130, 450), cv::Range(0, 800));

    Normalize(mean_, std_, scale, &mat_crop_tmp);

    // cv::imwrite("/apollo/mat_crop_tmp.jpg",mat_crop_tmp);
    std::vector<float> image_data(1 * 3 * img_height_crop_ * img_width_crop_,
                                  0.0f);
    Mat2Vec(mat_crop_tmp, image_data.data());

    images_data_.insert(images_data_.end(), image_data.begin(),
                        image_data.end());

    Eigen::Matrix4f img2lidar_matrix_rt_tmp;
    const auto &lidar2cam_matrix_rt =
        (camera_frame_temp->camera_extrinsic).cast<float>();
    const auto &cam_intrinstic_matrix_3f = camera_frame_temp->camera_k_matrix;
    GetImg2LidarMatrix(lidar2cam_matrix_rt, cam_intrinstic_matrix_3f,
                       &img2lidar_matrix_rt_tmp);

    std::vector<float> img2lidar_vec(
        img2lidar_matrix_rt_tmp.data(),
        img2lidar_matrix_rt_tmp.data() +
            img2lidar_matrix_rt_tmp.rows() * img2lidar_matrix_rt_tmp.cols());
    AERROR << "WXT DEBUG:" << img2lidar_matrix_rt_tmp.size();
    std::cout << img2lidar_matrix_rt_tmp << "\n";
    AERROR << "WXT DEBUG:" << img2lidar_vec.size();

    for (const auto i : img2lidar_vec) {
      std::cout << i << " ";
    }
    std::cout << "\n";

    k_data_.insert(k_data_.end(), img2lidar_vec.begin(), img2lidar_vec.end());
  }

  AINFO << "Preprocess: " << static_cast<double>(timer.Toc()) * 0.001 << "ms";

  std::vector<float> boxes;
  std::vector<float> scores;
  std::vector<int64_t> labels;
  AERROR << "WXT DEBUG: before inference";
  AERROR << "WXT DEBUG: k_data_.size: " << k_data_.size();
  AERROR << "WXT DEBUG: images_data.size:" << images_data_.size();
  Run(predictor_.get(), images_shape_, images_data_, k_shape_, k_data_, &boxes,
      &scores, &labels);

  AINFO << "Inference: " << static_cast<double>(timer.Toc()) * 0.001 << "ms";
  AERROR << "WXT DEBUG: after inference";
  AERROR << "boxes: " << std::endl;
  for (const auto &i : boxes) {
    std::cout << i << " ";
  }
  std::cout << "\n";

  AERROR << "scores: " << std::endl;
  for (const auto i : scores) {
    std::cout << i << " ";
  }
  std::cout << "\n";

  AERROR << "labels: " << std::endl;
  for (const auto &i : labels) {
    std::cout << i << " ";
  }
  std::cout << "\n";

  AERROR << "wxt debug: out BEVObstacleDetector::Process";

  return true;
}

void BEVObstacleDetector::Resize(const cv::Mat &img, int resized_h,
                                 int resized_w, cv::Mat *resize_img) {
  cv::resize(img, *resize_img, cv::Size(resized_w, resized_h), 0, 0,
             cv::INTER_LINEAR);
}

void BEVObstacleDetector::Normalize(const std::vector<float> &mean,
                                    const std::vector<float> &std, float scale,
                                    cv::Mat *im) {
  ACHECK(std.size() == 3);
  for (const auto std_value : std) {
    ACHECK(std_value != 0.0);
  }
  ACHECK(scale != 0.0);
  (*im).convertTo(*im, CV_32FC3, scale);
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

void BEVObstacleDetector::Mat2Vec(const cv::Mat &im, float *data) {
  ACHECK(nullptr != data);
  int rh = im.rows;
  int rw = im.cols;
  int rc = im.channels();

  for (int i = 0; i < rc; ++i) {
    cv::extractChannel(im, cv::Mat(rh, rw, CV_32FC1, data + i * rh * rw), i);
  }
}

void BEVObstacleDetector::FilterScore(
    const std::vector<float> &box3d, const std::vector<int64_t> &label_preds,
    const std::vector<float> &scores, float score_threshold,
    std::vector<float> *box3d_filtered,
    std::vector<int64_t> *label_preds_filtered) {
  for (size_t i = 0; i < scores.size(); ++i) {
    if (scores.at(i) > score_threshold) {
      box3d_filtered->insert(box3d_filtered->end(),
                             box3d.begin() + num_output_box_feature_ * i,
                             box3d.begin() + num_output_box_feature_ * (i + 1));
      label_preds_filtered->insert(label_preds_filtered->end(),
                                   *(label_preds.begin() + i));
    }
  }
}

void BEVObstacleDetector::Lidar2cam(const Eigen::Matrix4f &imu2camera,
                                    const Eigen::Matrix4f &imu2lidar,
                                    Eigen::Matrix4f *lidar2camera) {
  *lidar2camera = imu2camera * (imu2lidar.inverse());
}

void BEVObstacleDetector::GetMatrixRT(
    const Eigen::Quaterniond &rotation_quaternion,
    const Eigen::Vector3f &translation, Eigen::Matrix4f *matrix_rt) {
  ACHECK(nullptr != matrix_rt);
  Eigen::Matrix3d rotation_matrix = rotation_quaternion.toRotationMatrix();
  matrix_rt->setIdentity();
  matrix_rt->block<3, 3>(0, 0) = rotation_matrix.cast<float>();
  matrix_rt->block<3, 1>(0, 3) = translation;
}

void BEVObstacleDetector::GetImg2LidarMatrix(
    const Eigen::Matrix4f &imu2cam_matrix_rt,
    const Eigen::Matrix3f &cam_intrinstic_matrix_3f,
    const Eigen::Matrix4f &imu2lidar_matrix_rt,
    Eigen::Matrix4f *img2lidar_matrix_rt) {
  ACHECK(nullptr != img2lidar_matrix_rt);

  auto &lidar2cam_matrix_rt =
      imu2cam_matrix_rt * (imu2lidar_matrix_rt.inverse());
  Eigen::Matrix4f camera_intrinstic_matrix_4f;
  camera_intrinstic_matrix_4f.setIdentity();
  camera_intrinstic_matrix_4f.block<3, 3>(0, 0) = cam_intrinstic_matrix_3f;
  *img2lidar_matrix_rt =
      (camera_intrinstic_matrix_4f * lidar2cam_matrix_rt).inverse();
}

void BEVObstacleDetector::GetImg2LidarMatrix(
    const Eigen::Matrix4f &lidar2cam_matrix_rt,
    const Eigen::Matrix3f &cam_intrinstic_matrix_3f,
    Eigen::Matrix4f *img2lidar_matrix_rt) {
  ACHECK(nullptr != img2lidar_matrix_rt);

  Eigen::Matrix4f camera_intrinstic_matrix_4f;
  camera_intrinstic_matrix_4f.setIdentity();
  camera_intrinstic_matrix_4f.block<3, 3>(0, 0) = cam_intrinstic_matrix_3f;
  *img2lidar_matrix_rt =
      (camera_intrinstic_matrix_4f * lidar2cam_matrix_rt).inverse();
}

void BEVObstacleDetector::Run(
    paddle_infer::Predictor *predictor, const std::vector<int> &images_shape,
    const std::vector<float> &images_data, const std::vector<int> &k_shape,
    const std::vector<float> &k_data, std::vector<float> *boxes,
    std::vector<float> *scores, std::vector<int64_t> *labels) {
  auto input_names = predictor->GetInputNames();
  auto in_tensor0 = predictor->GetInputHandle(input_names[0]);
  in_tensor0->Reshape(images_shape);
  in_tensor0->CopyFromCpu(images_data.data());

  auto in_tensor1 = predictor->GetInputHandle(input_names[1]);
  in_tensor1->Reshape(k_shape);
  in_tensor1->CopyFromCpu(k_data.data());

  // auto start_time = std::chrono::steady_clock::now();
  ACHECK(predictor->Run());
  AINFO << "finish run!!!!";
  auto output_names = predictor->GetOutputNames();
  for (size_t i = 0; i != output_names.size(); i++) {
    auto output = predictor->GetOutputHandle(output_names[i]);

    std::vector<int> output_shape = output->shape();
    int out_num = std::accumulate(output_shape.begin(), output_shape.end(), 1,
                                  std::multiplies<int>());
    if (i == 0) {
      std::cout << "get bbox out size: " << out_num << std::endl;
      boxes->resize(out_num);
      output->CopyToCpu(boxes->data());
    } else if (i == 1) {
      std::cout << "get scores out size: " << out_num << std::endl;
      scores->resize(out_num);
      output->CopyToCpu(scores->data());
    } else if (i == 2) {
      std::cout << "get labels out size: " << out_num << std::endl;
      labels->resize(out_num);
      output->CopyToCpu(labels->data());
      std::cout << "finish get labels out size: " << out_num << std::endl;
    }
  }
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo