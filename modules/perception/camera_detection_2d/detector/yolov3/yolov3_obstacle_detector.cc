/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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
#include "modules/perception/camera_detection_2d/detector/yolov3/yolov3_obstacle_detector.h"

#include <opencv2/opencv.hpp>

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/perception/camera_detection_2d/detector/yolov3/postprocess.h"
#include "modules/perception/common/inference/inference_factory.h"
#include "modules/perception/common/util.h"

namespace apollo {
namespace perception {
namespace camera {

using cyber::common::GetAbsolutePath;

void Yolov3ObstacleDetector::LoadInputShape(
    const yolov3::ModelParam &model_param) {
  // loading config params
  width_ = model_param.resize().width();
  height_ = model_param.resize().height();

  image_width_ = options_.image_width;
  image_height_ = options_.image_height;

  AINFO << " image_height =" << image_height_
        << ", image_width=" << image_width_ << ", height=" << height_
        << ", width=" << width_;
}

void Yolov3ObstacleDetector::LoadParam(const yolov3::ModelParam &model_param) {
  confidence_threshold_ = model_param.confidence_threshold();
  border_ratio_ = model_param.border_ratio();

  // Init NMS proto param by config file
  const auto &nms_param = model_param.nms_param();
  nms_.set_sigma(nms_param.sigma());
  nms_.set_type(nms_param.type());
  nms_.set_threshold(nms_param.threshold());
}

void AddShape3D(
    std::map<std::string, std::vector<int>> *shape_map,
    const google::protobuf::RepeatedPtrField<common::ModelBlob> &model_blobs) {
  for (const auto &blob : model_blobs) {
    std::vector<int> shape(blob.shape().begin(), blob.shape().end());
    shape_map->insert(std::make_pair(blob.name(), shape));
  }
}

std::vector<std::string> GetBlobNames3D(
    const google::protobuf::RepeatedPtrField<common::ModelBlob> &model_blobs) {
  std::vector<std::string> blob_names;
  for (const auto &blob : model_blobs) {
    blob_names.push_back(blob.name());
  }
  return blob_names;
}

bool Yolov3ObstacleDetector::Init3DNetwork(const common::ModelInfo &model_info,
                                           const std::string &model_path) {
  // Network files
  std::string proto_file =
      GetModelFile(model_path, model_info.proto_file().file());
  std::string weight_file =
      GetModelFile(model_path, model_info.weight_file().file());

  // Network input and output names
  std::vector<std::string> input_names = GetBlobNames3D(model_info.inputs());
  std::vector<std::string> output_names = GetBlobNames3D(model_info.outputs());

  // Network type
  const auto &framework = model_info.framework();

  net_3D_.reset(inference::CreateInferenceByName(framework, proto_file,
                                                 weight_file, output_names,
                                                 input_names, model_path));

  ACHECK(net_3D_ != nullptr);
  net_3D_->set_gpu_id(gpu_id_);

  std::map<std::string, std::vector<int>> shape_map;
  AddShape3D(&shape_map, model_info.inputs());
  AddShape3D(&shape_map, model_info.outputs());

  if (!net_3D_->Init(shape_map)) {
    AERROR << model_info.name() << "init failed!";
    return false;
  }
  return true;
}

void Yolov3ObstacleDetector::Yolo3DInference(const base::Image8U *image,
                                             base::ObjectPtr obj) {
  ACHECK(image != nullptr);

  cv::Mat img = cv::Mat(image->rows(), image->cols(), CV_8UC3);

  cv::cvtColor(img, img, cv::COLOR_BGR2RGB);

  memcpy(img.data, image->cpu_data(),
         image->rows() * image->cols() * image->channels() * sizeof(uint8_t));

  // get left top and right bottom point
  float left_x = clamp(obj->camera_supplement.box.xmin, 0.f,
                       static_cast<float>(image->cols()));
  float left_y = clamp(obj->camera_supplement.box.ymin, 0.f,
                       static_cast<float>(image->rows()));
  float right_x = clamp(obj->camera_supplement.box.xmax, 0.f,
                        static_cast<float>(image->cols()));
  float right_y = clamp(obj->camera_supplement.box.ymax, 0.f,
                        static_cast<float>(image->rows()));

  float width = right_x - left_x;
  float height = right_y - left_y;

  cv::Rect object_roi(left_x, left_y, width, height);
  cv::Mat cropped_obj = img(object_roi);
  // cv::Mat show = cropped_obj;

  // generate new pure black bg as same size as ratio
  float ratio = std::max(width, height);
  int dim_diff = std::abs(height - width);
  int pad1 = std::floor(static_cast<double>(dim_diff) / 2.0);
  cv::Mat out(ratio, ratio, CV_8UC3, {0, 0, 0});  // helps to clear noisy

  // if h < w : add img to middle of bg
  // else : add img to left of bg
  if (width > height) {
    cropped_obj.copyTo(
        out(cv::Rect(0, pad1, cropped_obj.cols, cropped_obj.rows)));
  } else {
    cropped_obj.copyTo(
        out(cv::Rect(pad1, 0, cropped_obj.cols, cropped_obj.rows)));
  }

  cv::resize(out, out, cv::Size(224, 224), cv::INTER_CUBIC);

  out.convertTo(cropped_obj, CV_32F, 1.0 / 255, 0);

  std::vector<float> mean_values{0.485, 0.456, 0.406};
  std::vector<float> std_values{0.229, 0.224, 0.225};

  // normallize channel value from 0～255 to 0~1 and change it to float type
  std::vector<cv::Mat> rgbChannels(3);
  cv::split(cropped_obj, rgbChannels);
  for (int i = 0; i < 3; ++i) {
    rgbChannels[i].convertTo(rgbChannels[i], CV_32FC1, 1 / std_values[i],
                             (0.0 - mean_values[i]) / std_values[i]);
  }
  cv::Mat dst;
  cv::merge(rgbChannels, dst);
  cropped_obj = dst;

  auto model_inputs = model_param_.info_3d().info().inputs();

  auto input_blob_3d = net_3D_->get_blob(model_inputs[0].name());
  ACHECK(input_blob_3d != nullptr);

  int model_input_rows = cropped_obj.rows;
  int model_input_cols = cropped_obj.cols;
  int model_input_chs = cropped_obj.channels();
  float *input_data = input_blob_3d->mutable_cpu_data();

  input_blob_3d->Reshape(
      {1, model_input_chs, model_input_rows, model_input_cols});
  for (int i = 0; i < model_input_chs; ++i) {
    // put img model_input_chs i data to input_blob
    cv::extractChannel(
        cropped_obj,
        cv::Mat(model_input_rows, model_input_cols, CV_32FC1,
                input_data + i * model_input_rows * model_input_cols),
        i);
  }

  net_3D_->Infer();

  auto model_outputs = model_param_.info_3d().info().outputs();
  auto blob_orient = net_3D_->get_blob(model_outputs[0].name());
  auto blob_conf = net_3D_->get_blob(model_outputs[1].name());
  auto blob_dim = net_3D_->get_blob(model_outputs[2].name());

  ACHECK(blob_orient != nullptr);
  ACHECK(blob_conf != nullptr);
  ACHECK(blob_dim != nullptr);

  const float *orient_data = blob_orient->cpu_data();
  const float *conf_data = blob_conf->cpu_data();
  const float *dim_data = blob_dim->cpu_data();

  const int bin_number = model_param_.info_3d().bin_num();
  int max_index = 0;
  int max_conf = conf_data[max_index];
  for (int i = 1; i < bin_number; ++i) {
    if (conf_data[i] > max_conf) {
      max_conf = conf_data[i];
      max_index = i;
    }
  }

  int orient_index = 2 * max_index;
  float alpha = 0.f;
  float cos_result = orient_data[orient_index];
  float sin_result = orient_data[orient_index + 1];
  float angle_offset = std::atan2(sin_result, cos_result);

  float wedge = 2 * PI / bin_number;
  alpha = angle_offset + max_index * wedge;
  alpha = alpha + wedge / 2 - PI;
  if (alpha > PI) {
    alpha -= 2 * PI;
  }
  obj->camera_supplement.alpha = alpha;

  // add dim to object here
  obj->size[0] += dim_data[2];
  obj->size[1] += dim_data[1];
  obj->size[2] += dim_data[0];

  return;
}

bool Yolov3ObstacleDetector::Init(const ObstacleDetectorInitOptions &options) {
  options_ = options;

  gpu_id_ = options.gpu_id;
  BASE_CUDA_CHECK(cudaSetDevice(gpu_id_));
  BASE_CUDA_CHECK(cudaStreamCreate(&stream_));

  std::string config_file =
      GetConfigFile(options.config_path, options.config_file);
  if (!cyber::common::GetProtoFromFile(config_file, &model_param_)) {
    AERROR << "Read proto_config failed! " << config_file;
    return false;
  }

  LoadInputShape(model_param_);
  LoadParam(model_param_);

  const auto &model_info = model_param_.info();
  std::string model_path = GetModelPath(model_info.name());
  if (!InitNetwork(model_info, model_path)) {
    AERROR << "Init network failed!";
    return false;
  }
  AERROR << "[INFO] yolov3 2D model init success";

  const auto &model_info_3d = model_param_.info_3d().info();
  std::string model_path_3d = GetModelPath(model_info_3d.name());
  if (!Init3DNetwork(model_info_3d, model_path_3d)) {
    AERROR << "Init network failed!";
    return false;
  }
  AERROR << "[INFO] yolov3 3D model init success";

  return true;
}

bool Yolov3ObstacleDetector::Preprocess(const base::Image8U *image,
                                        base::BlobPtr<float> input_blob) {
  ACHECK(image != nullptr);
  ACHECK(input_blob != nullptr);

  // init cv img containter, same to image row and col
  cv::Mat img = cv::Mat(image->rows(), image->cols(), CV_8UC3);
  memcpy(img.data, image->cpu_data(),
         image->rows() * image->cols() * image->channels() * sizeof(uint8_t));
  // tools : image show to help check
  // cv::imwrite("yolov3_image.png", img);

  // generate new pure black bg as same size as ratio
  float ratio = std::max(image_width_, image_height_);
  int dim_diff = std::abs(image_height_ - image_width_);
  int pad1 = std::floor(static_cast<double>(dim_diff) / 2.0);
  cv::Mat out(ratio, ratio, CV_8UC3, {0, 0, 0});  // helps to clear noisy

  // image preprocess sames to training
  // if h < w : add img to middle of bg
  // else : add img to left of bg
  // add img to bg, avoid image distortion
  img.copyTo(out(cv::Rect(0, pad1, img.cols, img.rows)));

  // cv::resize(out, out, cv::Size(width_, height_), cv::INTER_NEAREST);
  cv::resize(out, out, cv::Size(width_, height_), cv::INTER_AREA);

  img = out;

  // normallize channel value from 0～255 to 0~1 and change it to float type
  img.convertTo(img, CV_32F, 1.0 / 255, 0);

  int model_input_rows = img.rows;
  int model_input_cols = img.cols;
  int model_input_chs = img.channels();

  // fill input_blob -> tensor_image will be used by model
  input_blob->Reshape({1, model_input_chs, model_input_rows, model_input_cols});
  float *input_data = input_blob->mutable_cpu_data();
  for (int i = 0; i < model_input_chs; ++i) {
    // put img model_input_chs i data to input_blob
    cv::extractChannel(
        img,
        cv::Mat(model_input_rows, model_input_cols, CV_32FC1,
                input_data + i * model_input_rows * model_input_cols),
        i);
  }
  return true;
}

bool Yolov3ObstacleDetector::Detect(onboard::CameraFrame *frame) {
  if (frame == nullptr) {
    return false;
  }

  if (cudaSetDevice(gpu_id_) != cudaSuccess) {
    AERROR << "Failed to set device to " << gpu_id_;
    return false;
  }

  auto model_inputs = model_param_.info().inputs();
  auto input_blob = net_->get_blob(model_inputs[0].name());

  DataProvider::ImageOptions image_options;
  image_options.target_color = base::Color::RGB;

  base::Image8U image;
  frame->data_provider->GetImage(image_options, &image);

  // TODO(lordon): use cuda
  Preprocess(&image, input_blob);

  // model infer, save to output blob
  net_->Infer();

  // get objects from network inference result
  auto model_outputs = model_param_.info().outputs();
  auto blob_predict = net_->get_blob(model_outputs[0].name());

  GetYolov3ObjectsCpu(blob_predict, model_param_, nms_, width_, height_,
                      image_width_, image_height_, &frame->detected_objects);

  frame->feature_blob = net_->get_blob(model_outputs[1].name());

  // post processing
  float border_ratio = 0.01;
  int left_boundary =
      static_cast<int>(border_ratio * static_cast<float>(image.cols()));
  int right_boundary = static_cast<int>((1.0f - border_ratio) *
                                        static_cast<float>(image.cols()));
  for (auto &obj : frame->detected_objects) {
    obj->camera_supplement.area_id = 1;

    Yolo3DInference(&image, obj);

    // clear cut off ratios
    auto &box = obj->camera_supplement.box;
    if (box.xmin >= left_boundary) {
      obj->camera_supplement.cut_off_ratios[2] = 0;
    }
    if (box.xmax <= right_boundary) {
      obj->camera_supplement.cut_off_ratios[3] = 0;
    }
  }

  return true;
}
REGISTER_OBSTACLE_DETECTOR(Yolov3ObstacleDetector);

}  // namespace camera
}  // namespace perception
}  // namespace apollo
