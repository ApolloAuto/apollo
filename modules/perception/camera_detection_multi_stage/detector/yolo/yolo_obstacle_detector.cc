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
#include "modules/perception/camera_detection_multi_stage/detector/yolo/yolo_obstacle_detector.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "cyber/profiler/profiler.h"
#include "modules/perception/common/camera/common/util.h"
#include "modules/perception/common/inference/utils/resize.h"
#include "modules/perception/common/util.h"

namespace apollo {
namespace perception {
namespace camera {

void YoloObstacleDetector::LoadInputShape(const yolo::ModelParam &model_param) {
  float offset_ratio = model_param.offset_ratio();
  float cropped_ratio = model_param.cropped_ratio();
  int resized_width = model_param.resized_width();
  int aligned_pixel = model_param.aligned_pixel();

  int image_width = options_.image_width;
  int image_height = options_.image_height;

  offset_y_ = static_cast<int>(std::round(offset_ratio * image_height));
  float roi_ratio = cropped_ratio * image_height / image_width;
  width_ = (resized_width + aligned_pixel / 2) / aligned_pixel * aligned_pixel;
  height_ = static_cast<int>(width_ * roi_ratio + aligned_pixel / 2) /
            aligned_pixel * aligned_pixel;

  AINFO << "image_height=" << image_height << ", image_width=" << image_width
        << ", roi_ratio=" << roi_ratio;
  AINFO << "offset_y=" << offset_y_ << ", height=" << height_
        << ", width=" << width_;
}

void YoloObstacleDetector::LoadParam(const yolo::ModelParam &model_param) {
  confidence_threshold_ = model_param.confidence_threshold();
  light_vis_conf_threshold_ = model_param.light_vis_conf_threshold();
  light_swt_conf_threshold_ = model_param.light_swt_conf_threshold();

  auto min_dims = model_param.min_dims();
  min_dims_.min_2d_height = min_dims.min_2d_height();
  min_dims_.min_3d_height = min_dims.min_3d_height();
  min_dims_.min_3d_width = min_dims.min_3d_width();
  min_dims_.min_3d_length = min_dims.min_3d_length();

  ori_cycle_ = model_param.ori_cycle();

  border_ratio_ = model_param.border_ratio();

  // init NMS
  const auto &nms_param = model_param.nms_param();
  nms_.sigma = nms_param.sigma();
  nms_.type = nms_param.type();
  nms_.threshold = nms_param.threshold();
  nms_.inter_cls_nms_thresh = nms_param.inter_cls_nms_thresh();
  nms_.inter_cls_conf_thresh = nms_param.inter_cls_conf_thresh();
}

void YoloObstacleDetector::InitYoloBlob() {
  auto obj_blob_scale1 = net_->get_blob("obj_pred");
  auto obj_blob_scale2 = net_->get_blob("det2_obj_blob");
  auto obj_blob_scale3 = net_->get_blob("det3_obj_blob");
  int output_height_scale1 = obj_blob_scale1->shape(1);
  int output_width_scale1 = obj_blob_scale1->shape(2);
  int obj_size = output_height_scale1 * output_width_scale1 *
                 static_cast<int>(anchors_.size()) / anchorSizeFactor;
  // todo(daohu527): why just obj_blob_scale2 != nullptr
  if (obj_blob_scale2) {
    int output_height_scale2 = obj_blob_scale2->shape(1);
    int output_width_scale2 = obj_blob_scale2->shape(2);
    int output_height_scale3 = obj_blob_scale3->shape(1);
    int output_width_scale3 = obj_blob_scale3->shape(2);
    obj_size = (output_height_scale1 * output_width_scale1 +
                output_height_scale2 * output_width_scale2 +
                output_height_scale3 * output_width_scale3) *
               static_cast<int>(anchors_.size()) / anchorSizeFactor / numScales;
  }

  yolo_blobs_.res_box_blob.reset(
      new base::Blob<float>(1, 1, obj_size, kBoxBlockSize));
  yolo_blobs_.res_cls_blob.reset(new base::Blob<float>(
      1, 1, static_cast<int>(types_.size() + 1), obj_size));
  yolo_blobs_.res_cls_blob->cpu_data();
  overlapped_.reset(
      new base::Blob<bool>(std::vector<int>{obj_k_, obj_k_}, true));
  overlapped_->cpu_data();
  overlapped_->gpu_data();
  idx_sm_.reset(new base::Blob<int>(std::vector<int>{obj_k_}, true));
  yolo_blobs_.anchor_blob.reset(
      new base::Blob<float>(1, 1, static_cast<int>(anchors_.size() / 2), 2));
  yolo_blobs_.expand_blob.reset(
      new base::Blob<float>(1, 1, 1, static_cast<int>(expands_.size())));
  auto expand_cpu_data = yolo_blobs_.expand_blob->mutable_cpu_data();
  memcpy(expand_cpu_data, expands_.data(), expands_.size() * sizeof(float));
  auto anchor_cpu_data = yolo_blobs_.anchor_blob->mutable_cpu_data();
  memcpy(anchor_cpu_data, anchors_.data(), anchors_.size() * sizeof(float));
  yolo_blobs_.anchor_blob->gpu_data();

  yolo_blobs_.det1_loc_blob = net_->get_blob("loc_pred");
  yolo_blobs_.det1_obj_blob = net_->get_blob("obj_pred");
  yolo_blobs_.det1_cls_blob = net_->get_blob("cls_pred");
  yolo_blobs_.det1_ori_conf_blob = net_->get_blob("detect1_ori_conf_pred");
  yolo_blobs_.det1_ori_blob = net_->get_blob("ori_pred");
  yolo_blobs_.det1_dim_blob = net_->get_blob("dim_pred");
  yolo_blobs_.det2_loc_blob = net_->get_blob("detect2_loc_pred");
  yolo_blobs_.det2_obj_blob = net_->get_blob("detect2_obj_pred");
  yolo_blobs_.det2_cls_blob = net_->get_blob("detect2_cls_pred");
  yolo_blobs_.det2_ori_conf_blob = net_->get_blob("detect2_ori_conf_pred");
  yolo_blobs_.det2_ori_blob = net_->get_blob("detect2_ori_pred");
  yolo_blobs_.det2_dim_blob = net_->get_blob("detect2_dim_pred");
  yolo_blobs_.det3_loc_blob = net_->get_blob("detect3_loc_pred");
  yolo_blobs_.det3_obj_blob = net_->get_blob("detect3_obj_pred");
  yolo_blobs_.det3_cls_blob = net_->get_blob("detect3_cls_pred");
  yolo_blobs_.det3_ori_conf_blob = net_->get_blob("detect3_ori_conf_pred");
  yolo_blobs_.det3_ori_blob = net_->get_blob("detect3_ori_pred");
  yolo_blobs_.det3_dim_blob = net_->get_blob("detect3_dim_pred");

  yolo_blobs_.lof_blob = net_->get_blob("lof_pred");
  yolo_blobs_.lor_blob = net_->get_blob("lor_pred");

  yolo_blobs_.brvis_blob = net_->get_blob("brvis_pred");
  yolo_blobs_.brswt_blob = net_->get_blob("brswt_pred");
  yolo_blobs_.ltvis_blob = net_->get_blob("ltvis_pred");
  yolo_blobs_.ltswt_blob = net_->get_blob("ltswt_pred");
  yolo_blobs_.rtvis_blob = net_->get_blob("rtvis_pred");
  yolo_blobs_.rtswt_blob = net_->get_blob("rtswt_pred");

  yolo_blobs_.area_id_blob = net_->get_blob("area_id_pred");
  yolo_blobs_.visible_ratio_blob = net_->get_blob("vis_pred");
  yolo_blobs_.cut_off_ratio_blob = net_->get_blob("cut_pred");

  yolo_blobs_.feat_blob = net_->get_blob("conv3_3");
}

bool YoloObstacleDetector::Init(const ObstacleDetectorInitOptions &options) {
  options_ = options;

  gpu_id_ = options.gpu_id;
  BASE_GPU_CHECK(cudaSetDevice(gpu_id_));
  BASE_GPU_CHECK(cudaStreamCreate(&stream_));

  std::string config_file =
      GetConfigFile(options.config_path, options.config_file);
  if (!cyber::common::GetProtoFromFile(config_file, &model_param_)) {
    AERROR << "Read proto_config failed! " << config_file;
    return false;
  }

  const auto &model_info = model_param_.info();
  std::string model_path = GetModelPath(model_info.name());
  std::string anchors_file =
      GetModelFile(model_path, model_info.anchors_file().file());
  std::string types_file =
      GetModelFile(model_path, model_info.types_file().file());
  std::string expand_file =
      GetModelFile(model_path, model_info.expand_file().file());
  LoadInputShape(model_param_);
  LoadParam(model_param_);
  min_dims_.min_2d_height /= static_cast<float>(height_);

  if (!LoadAnchors(anchors_file, &anchors_)) {
    return false;
  }
  if (!LoadTypes(types_file, &types_)) {
    return false;
  }
  if (!LoadExpand(expand_file, &expands_)) {
    return false;
  }
  ACHECK(expands_.size() == types_.size());

  if (!InitNetwork(model_param_.info(), model_path)) {
    AERROR << "Init network failed!";
    return false;
  }

  InitYoloBlob();
  return true;
}

bool YoloObstacleDetector::Detect(onboard::CameraFrame *frame) {
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
  image_options.target_color = base::Color::BGR;
  image_options.crop_roi = base::RectI(0, offset_y_, options_.image_width,
                                       options_.image_height - offset_y_);
  image_options.do_crop = true;
  base::Image8U image;
  frame->data_provider->GetImage(image_options, &image);

  inference::ResizeGPU(image, input_blob, frame->data_provider->src_width(), 0);
  PERF_BLOCK("camera_2d_detector_infer")
  net_->Infer();
  PERF_BLOCK_END

  PERF_BLOCK("camera_2d_detector_get_obj")
  get_objects_cpu(yolo_blobs_, stream_, types_, nms_, model_param_,
                  light_vis_conf_threshold_, light_swt_conf_threshold_,
                  overlapped_.get(), idx_sm_.get(), &frame->detected_objects);
  PERF_BLOCK_END

  filter_bbox(min_dims_, &frame->detected_objects);

  recover_bbox(frame->data_provider->src_width(),
               frame->data_provider->src_height() - offset_y_, offset_y_,
               &frame->detected_objects);

  // appearance features for tracking
  frame->feature_blob = yolo_blobs_.feat_blob;

  // post processing
  int left_boundary =
      static_cast<int>(border_ratio_ * static_cast<float>(image.cols()));
  int right_boundary = static_cast<int>((1.0f - border_ratio_) *
                                        static_cast<float>(image.cols()));
  for (auto &obj : frame->detected_objects) {
    // recover alpha
    obj->camera_supplement.alpha /= ori_cycle_;
    // get area_id from visible_ratios
    if (model_param_.num_areas() == 0) {
      obj->camera_supplement.area_id =
          get_area_id(obj->camera_supplement.visible_ratios);
    }
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

REGISTER_OBSTACLE_DETECTOR(YoloObstacleDetector);

}  // namespace camera
}  // namespace perception
}  // namespace apollo
