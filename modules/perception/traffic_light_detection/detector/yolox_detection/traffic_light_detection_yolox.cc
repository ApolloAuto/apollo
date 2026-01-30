/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/traffic_light_detection/detector/yolox_detection/traffic_light_detection_yolox.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <numeric>
#include <sstream>
#include <string>
#include <utility>

#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <opencv2/opencv.hpp>

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "cyber/profiler/profiler.h"
#include "modules/perception/common/camera/common/util.h"
#include "modules/perception/common/inference/inference_factory.h"
#include "modules/perception/common/inference/model_util.h"
#include "modules/perception/common/inference/utils/resize.h"
#include "modules/perception/common/util.h"
#include "opencv2/opencv.hpp"

namespace apollo {
namespace perception {
namespace trafficlight {

void TrafficLightTLDetectorYolox::AddShapeYolox(
        std::map<std::string, std::vector<int>>* shape_map,
        const google::protobuf::RepeatedPtrField<common::ModelBlob>& model_blobs) {
    for (const auto& blob : model_blobs) {
        std::vector<int> shape(blob.shape().begin(), blob.shape().end());
        shape_map->insert(std::make_pair(blob.name(), shape));
    }
}

bool TrafficLightTLDetectorYolox::Init(const TrafficLightDetectorInitOptions& options) {
    std::string config_file = GetConfigFile(options.config_path, options.config_file);
    if (!cyber::common::GetProtoFromFile(config_file, &detection_param_)) {
        AERROR << "Read proto_config failed! " << config_file;
        return false;
    }

    AINFO << "Traffic light detection param: " << detection_param_.DebugString();

    const auto& model_info = detection_param_.info();
    std::string model_path = GetModelPath(model_info.name());
    AINFO << "Traffic light detection model_path: " << model_path;

    std::string proto_file = GetModelFile(model_path, model_info.proto_file().file());
    AINFO << "Traffic light detection proto_file: " << proto_file;

    std::string weight_file = GetModelFile(model_path, model_info.weight_file().file());
    AINFO << "Traffic light detection weight_file: " << weight_file;

    gpu_id_ = options.gpu_id;
    BASE_GPU_CHECK(cudaSetDevice(gpu_id_));

    // Network input and output names
    input_names_ = inference::GetBlobNames(model_info.inputs());
    output_names_ = inference::GetBlobNames(model_info.outputs());

    // Network type
    const auto& framework = model_info.framework();
    AINFO << "framework: " << framework;

    std::string plugin_name = model_info.infer_plugin();
    static const std::string class_namespace = "apollo::perception::inference::";
    if (model_info.has_infer_plugin() && !plugin_name.empty()) {
        plugin_name = class_namespace + plugin_name;
        detect_net_ = apollo::cyber::plugin_manager::PluginManager::Instance()->CreateInstance<inference::Inference>(
                plugin_name);
        detect_net_->set_model_info(proto_file, input_names_, output_names_);
        AINFO << "trafficlight detect_net_ load plugin success: " << plugin_name;
    } else {
        detect_net_.reset(inference::CreateInferenceByName(
                framework, proto_file, weight_file, output_names_, input_names_, model_path));
    }

    ACHECK(detect_net_.get() != nullptr) << "Failed to init trafficlight yolox inference instance";

    max_batch_size_ = detection_param_.max_batch_size();
    resize_height_ = detection_param_.resize_image_hight();
    resize_width_ = detection_param_.resize_image_width();
    cls_th_ = detection_param_.cls_th();
    int resize_height = detection_param_.min_crop_size();
    int resize_width = detection_param_.min_crop_size();

    CHECK_GT(resize_height, 0);
    CHECK_GT(resize_width, 0);

    padding_image_container_ = base::Image8U(max_batch_roi_, max_batch_roi_, base::Color::BGR);

    AINFO << "TrafficLightTLDetectorYolox net input base blobs: "
          << std::accumulate(
                     input_names_.begin(),
                     input_names_.end(),
                     std::string(""),
                     [](std::string& sum, const std::string& s) { return sum + " " + s; });

    AINFO << "TrafficLightTLDetectorYolox net output base blobs: "
          << std::accumulate(
                     output_names_.begin(),
                     output_names_.end(),
                     std::string(""),
                     [](std::string& sum, const std::string& s) { return sum + " " + s; });

    detect_net_->set_gpu_id(options.gpu_id);
    detect_net_->set_max_batch_size(max_batch_size_ * 2);

    std::map<std::string, std::vector<int>> shape_map;
    AddShapeYolox(&shape_map, model_info.inputs());
    AddShapeYolox(&shape_map, model_info.outputs());

    if (!detect_net_->Init(shape_map)) {
        AERROR << model_info.name() << "init failed!";
        return false;
    }

    switch (detection_param_.crop_method()) {
    default:
    case 0:
        crop_.reset(new CropBox(detection_param_.crop_scale(), detection_param_.min_crop_size()));
        break;
    case 1:
        crop_.reset(new CropBoxWholeImage());
        break;
    }

    select_.Init(resize_width, resize_height);

    return true;
}

bool TrafficLightTLDetectorYolox::Detect(camera::TrafficLightFrame* frame) {
    DetectHeadByModel(frame);
    return true;
}

bool TrafficLightTLDetectorYolox::DetectHeadByModel(camera::TrafficLightFrame* frame) {
    DetectTL(frame);
    return true;
}

bool TrafficLightTLDetectorYolox::DetectTL(camera::TrafficLightFrame* frame) {
    // step 0: init params & clear members
    detected_bboxes_.clear();  // Detected BBoxes
    detected_heads_.clear();   // Total Detected BBoxes Pair With Each Camera
    bbox_project_camera_.clear();

    pad_resize_params_.clear();
    detected_light_ids_.clear();  // Hdmap Light Ids

    // step 1: choose input area
    if (frame->traffic_lights.size() <= 0) {
        AINFO << "no hdmap lights to detect";
        return true;
    }
    if (cudaSetDevice(gpu_id_) != cudaSuccess) {
        AERROR << "Failed To Set Device To: " << gpu_id_;
        return false;
    }
    // PERCEPTION_PERF_BLOCK_START();
    // TrafficLightFrame is const, cann't use &
    std::vector<base::TrafficLightPtr> lights_ref = frame->traffic_lights;
    AINFO << "detect lights num: " << lights_ref.size() << " lights";

    // step2 :input preprocess
    int batch_num = (static_cast<int>(lights_ref.size()) + max_batch_size_ - 1) / max_batch_size_;
    auto input_image_blob = detect_net_->get_blob(input_names_[0]);
    auto output_loc_blob = detect_net_->get_blob(output_names_[0]);
    auto output_conf_blob = detect_net_->get_blob(output_names_[1]);
    auto output_cls_blob = detect_net_->get_blob(output_names_[2]);

    const auto& data_provider = frame->data_provider;
    const int& provider_img_width = data_provider->src_width();
    const int& provider_img_height = data_provider->src_height();

    for (auto& light : lights_ref) {
        base::RectI debug_rect(0, 0, 0, 0);
        light->region.detection_roi = light->region.projection_roi;
        light->region.debug_roi.clear();
        light->region.debug_roi_detect_scores.clear();
        light->region.debug_roi.push_back(debug_rect);
        light->region.debug_roi_detect_scores.push_back(0.0f);
    }

    for (auto& light : lights_ref) {
        if (light->region.outside_image
            || camera::OutOfValidRegion(light->region.projection_roi, provider_img_width, provider_img_height)
            || light->region.projection_roi.Area() <= 0) {
            light->region.projection_roi.x = 0;
            light->region.projection_roi.y = 0;
            light->region.projection_roi.width = 0;
            light->region.projection_roi.height = 0;
        }
    }
    AINFO << "Crop Box Detection";
    // process in each batch
    for (int batch_id = 0; batch_id < batch_num; batch_id++) {
        // AINFO << "batch id: " << batch_id;
        int batch_begin = batch_id * max_batch_size_;
        int batch_end = (batch_id + 1) * max_batch_size_ < static_cast<int>(lights_ref.size())
                ? (batch_id + 1) * max_batch_size_
                : static_cast<int>(lights_ref.size());
        // calculate batch size
        int batch_size = batch_end - batch_begin;
        int resize_index = 0;
        // reshape input blob
        // note: [N, C, H, W] format
        input_image_blob->Reshape(batch_size, 3, resize_height_, resize_width_);
        int box_num = output_loc_blob->shape(1);
        output_loc_blob->Reshape({batch_size, box_num, 4});
        output_conf_blob->Reshape({batch_size, box_num});
        output_cls_blob->Reshape({batch_size, box_num});

        // allocate size
        bbox_project_camera_.reserve(batch_size);
        pad_resize_params_.reserve(batch_size);
        detected_light_ids_.reserve(batch_size);

        // iterate each batch
        for (int light_id = batch_begin; light_id < batch_end; ++light_id) {
            auto& light_hdmap = lights_ref[light_id];

            detected_light_ids_.push_back(light_hdmap->id);  // fill project light id

            const auto& data_provider = frame->data_provider;
            const int& img_width = data_provider->src_width();
            const int& img_height = data_provider->src_height();
            base::RectI cbox;
            crop_->getCropBox(img_width, img_height, light_hdmap, &cbox);
            AINFO << "get crop box success " << cbox.x << " " << cbox.y << " " << cbox.width << " " << cbox.height
                  << " light id: " << light_hdmap->id;
            if (cbox.width == 0 && cbox.height == 0) {
                continue;
            }

            light_hdmap->region.debug_roi[0] = cbox;
            light_hdmap->region.crop_roi = cbox;

            data_provider_image_option_.crop_roi = cbox;
            data_provider_image_option_.do_crop = true;
            data_provider_image_option_.target_color = base::Color::BGR;
            base::Image8U image;
            data_provider->GetImage(data_provider_image_option_, &image);
            //   //       base::Image8U image_test;
            //   // data_provider->GetImage(data_provider_image_option_, &image_test);
            //       // ***************** img show ****************
            //   cv::Mat img(image.rows(), image.cols(), CV_8UC3, cv::Scalar(0, 0, 0));
            //   memcpy(img.data, image.cpu_data(), image.total() * sizeof(uint8_t));

            //   std::string camera_name_save =
            //       "data/TRA/"
            //       + std::to_string(frame->timestamp)
            //       + ".png";
            //   cv::imwrite(camera_name_save, img);
            // // *******************************************

            int crop_roi_width = light_hdmap->region.crop_roi.width;
            int crop_roi_height = light_hdmap->region.crop_roi.height;

            int left_pad = 0;
            int right_pad = 0;
            int top_pad = 0;
            int bottom_pad = 0;

            // do top pad on image
            if (crop_roi_width > crop_roi_height) {
                top_pad = 0;
                bottom_pad = crop_roi_width - crop_roi_height;
            } else if (crop_roi_height > crop_roi_width) {
                left_pad = 0;
                right_pad = crop_roi_height - crop_roi_width;
            }

            int dst_height = crop_roi_height + top_pad + bottom_pad;
            int dst_width = crop_roi_width + left_pad + right_pad;

            // generate new img
            base::Image8U tmp_image = padding_image_container_(base::RectI(0, 0, dst_width, dst_height));

            float scale = resize_width_ * 1.0 / dst_width;
            PadResizeParam prp(
                    left_pad,
                    right_pad,
                    top_pad,
                    bottom_pad,
                    data_provider->src_width(),
                    data_provider->src_height(),
                    scale);
            pad_resize_params_.push_back(prp);

            inference::ImageZeroPadding(
                    image, &tmp_image, img_width, left_pad, right_pad, top_pad, bottom_pad, 114, stream_, true);

            // means = {0, 0, 0}, 1.0 is scale factor
            // resize implementation: input_image_blob width/height
            inference::ResizeGPU(tmp_image, input_image_blob, dst_width, resize_index, 0, 0, 0, false, 1.0);
            resize_index++;
        }

        // step3: do inference
        AINFO << "infer start";
        PERF_BLOCK("traffic_light_detection_inference")
        cudaDeviceSynchronize();
        detect_net_->Infer();
        cudaDeviceSynchronize();
        PERF_BLOCK_END
        AINFO << "infer end";

        // step4: post-process
        GetCandidateHeads(batch_id, &detected_bboxes_, lights_ref);
        AINFO << "detected_bboxes_GetCandidateHeads";
        AINFO << detected_bboxes_.size();
    }

    auto& camera_name = frame->data_provider->sensor_name();
    NMS(&detected_bboxes_, camera_name);
    AINFO << "Dump output done! Get Box Num: " << detected_bboxes_.size();

    for (size_t j = 0; j < detected_bboxes_.size(); ++j) {
        base::RectI& region = detected_bboxes_[j]->region.detection_roi;
        float score = detected_bboxes_[j]->region.detect_score;
        lights_ref[0]->region.debug_roi.push_back(region);
        lights_ref[0]->region.debug_roi_detect_scores.push_back(score);
    }

    AINFO << "start select";
    select_.SelectTrafficLights(detected_bboxes_, &lights_ref);
    AINFO << "select success";

    return true;
}
bool TrafficLightTLDetectorYolox::GetCandidateHeads(
        const int batch_id,
        std::vector<base::TrafficLightPtr>* lights,
        std::vector<base::TrafficLightPtr>& lights_ref) {
    auto output_loc_blob = detect_net_->get_blob(output_names_[0]);
    auto output_conf_blob = detect_net_->get_blob(output_names_[1]);
    auto output_cls_blob = detect_net_->get_blob(output_names_[2]);

    int batch_size = output_loc_blob->shape()[0];
    int anchor_num = output_cls_blob->shape()[1];

    auto conf_data_cpu = output_conf_blob->cpu_data();
    auto bbx_data_cpu = output_loc_blob->cpu_data();
    auto cls_data_cpu = reinterpret_cast<const int*>(output_cls_blob->cpu_data());

    const int bbox_dims = 4;

    for (int i = 0; i < batch_size; i++) {
        auto conf = conf_data_cpu + i * anchor_num;
        for (int j = 0; j < anchor_num; j++) {
            bool keep = false;
            if (conf[j] >= cls_th_) {
                keep = true;
            }
            if (!keep) {
                // AINFO<<"not_keep";
                continue;
            }
            int light_idx = batch_id * max_batch_size_ + i;

            // new traffic light result
            base::TrafficLightPtr tmp(new base::TrafficLight);

            // fullfill sublight info
            PadResizeParam prp = pad_resize_params_[light_idx];
            // result box format: [x_ctr, y_ctr, w, h]
            float x = bbx_data_cpu[i * anchor_num * bbox_dims + j * bbox_dims];
            float y = bbx_data_cpu[i * anchor_num * bbox_dims + j * bbox_dims + 1];
            float w = bbx_data_cpu[i * anchor_num * bbox_dims + j * bbox_dims + 2];
            float h = bbx_data_cpu[i * anchor_num * bbox_dims + j * bbox_dims + 3];
            // AINFO << "box: " << x << " " << y << " " << w << " " << h;
            double head_area = (w / prp.scale) * (h / prp.scale);

            if (head_area < min_head_area_) {
                AINFO << "filter small roi: " << head_area;
                continue;
            }

            // class-index
            int shape_idx = i * anchor_num + j;
            tmp->region.detect_class_id = base::TLDetectionClass(cls_data_cpu[shape_idx]);
            // tmp->region2d.project_camera_name = bbox_project_camera;

            // roi format: [x1, y1, w, h]
            tmp->region.detection_roi.width = static_cast<int>(w / prp.scale);
            tmp->region.detection_roi.height = static_cast<int>(h / prp.scale);
            tmp->region.detection_roi.x = static_cast<int>(
                    x / prp.scale - prp.left_pad + lights_ref[light_idx]->region.crop_roi.x
                    - tmp->region.detection_roi.width / 2.);
            tmp->region.detection_roi.y = static_cast<int>(
                    y / prp.scale - prp.top_pad + lights_ref[light_idx]->region.crop_roi.y
                    - tmp->region.detection_roi.height / 2.);
            tmp->region.detect_score = conf[j];
            tmp->region.is_detected = true;
            tmp->id = detected_light_ids_[light_idx];

            if (camera::OutOfValidRegion(tmp->region.detection_roi, prp.roi_width, prp.roi_height)
                || tmp->region.detection_roi.Area() <= 0) {
                AINFO << "Invalid width or height or x or y: " << tmp->region.detection_roi.width << " | "
                      << tmp->region.detection_roi.height << " | " << tmp->region.detection_roi.x << " | "
                      << tmp->region.detection_roi.y;
                continue;
            }

            camera::RefineBox(tmp->region.detection_roi, prp.roi_width, prp.roi_height, &(tmp->region.detection_roi));

            AINFO << "push lights: " << tmp->region.detection_roi.x << " " << tmp->region.detection_roi.y << " "
                  << tmp->region.detection_roi.width << " " << tmp->region.detection_roi.height << " "
                  << tmp->region.detect_score;
            lights->push_back(tmp);
        }
    }

    return true;
}

void TrafficLightTLDetectorYolox::NMS(std::vector<base::TrafficLightPtr>* lights, const std::string camera_name) {
    // CHECK_NOTNULL(lights);
    for (size_t i = 0; i < lights->size(); i++) {
        base::TrafficLightPtr light = lights->at(i);
        const std::string& light_project_camera = camera_name;
        detected_heads_[light_project_camera].push_back(light);
    }
    (*lights).clear();
    // apply nms for each camera
    for (auto& lights_each_camera : detected_heads_) {
        std::vector<base::TrafficLightPtr> lights_for_each_camera = lights_each_camera.second;
        ApplyOverlapNMS(&lights_for_each_camera);
        ApplyNMS(&lights_for_each_camera);
        for (auto light : lights_for_each_camera) {
            (*lights).push_back(light);
            // debug - TrafficLightPtr
            AINFO << "nms light: " << light->region.detection_roi.x << " " << light->region.detection_roi.y << " "
                  << light->region.detection_roi.width << " " << light->region.detection_roi.height << " "
                  << light->region.detect_score;
        }
    }
}

void TrafficLightTLDetectorYolox::ApplyOverlapNMS(std::vector<base::TrafficLightPtr>* lights) {
    // CHECK_NOTNULL(lights);
    // (score, index) pairs sorted by area
    std::vector<std::pair<int, int>> score_index_vec(lights->size());
    for (size_t i = 0; i < lights->size(); i++) {
        score_index_vec[i].first
                = lights->at(i)->region.detection_roi.width * lights->at(i)->region.detection_roi.height;
        score_index_vec[i].second = i;
    }
    std::stable_sort(
            score_index_vec.begin(),
            score_index_vec.end(),
            [](const std::pair<int, int>& pr1, const std::pair<int, int>& pr2) { return pr1.first < pr2.first; });

    std::vector<int> kept_indices;
    while (score_index_vec.size()) {
        const int& idx = score_index_vec.back().second;
        bool keep = true;
        for (size_t k = 0; k < kept_indices.size(); k++) {
            const int& kept_idx = kept_indices[k];
            const auto& rect1 = lights->at(idx)->region.detection_roi;
            const auto& rect2 = lights->at(kept_idx)->region.detection_roi;
            float overlap = static_cast<float>((rect1 & rect2).Area()) / rect1.Area();
            // if current bbox has large overlap with any kept bbox, drop it
            keep = std::fabs(overlap) < 0.7;
            if (!keep) {
                break;
            }
        }
        if (keep) {
            kept_indices.push_back(idx);
        }
        score_index_vec.pop_back();
    }
    int idx = 0;
    auto parted_itr = std::stable_partition(lights->begin(), lights->end(), [&](const base::TrafficLightPtr& light) {
        return std::find(kept_indices.begin(), kept_indices.end(), idx++) != kept_indices.end();
    });
    lights->erase(parted_itr, lights->end());
}

void TrafficLightTLDetectorYolox::ApplyNMS(std::vector<base::TrafficLightPtr>* lights) {
    // (score, index) pairs sorted by detect score
    std::vector<std::pair<float, int>> score_index_vec(lights->size());
    for (size_t i = 0; i < lights->size(); ++i) {
        score_index_vec[i].first = lights->at(i)->region.detect_score;
        score_index_vec[i].second = i;
    }
    std::stable_sort(
            score_index_vec.begin(),
            score_index_vec.end(),
            [](const std::pair<float, int>& pr1, const std::pair<float, int>& pr2) { return pr1.first < pr2.first; });

    std::vector<int> kept_indices;
    while (score_index_vec.size() != 0) {
        const int idx = score_index_vec.back().second;
        bool keep = true;
        for (size_t k = 0; k < kept_indices.size(); ++k) {
            const int kept_idx = kept_indices[k];
            const auto& rect1 = lights->at(idx)->region.detection_roi;
            const auto& rect2 = lights->at(kept_idx)->region.detection_roi;
            float overlap = static_cast<float>((rect1 & rect2).Area()) / (rect1 | rect2).Area();
            // if current bbox has large overlap(>=iou_thresh) with any
            // kept bbox, drop it
            // keep = std::fabs(overlap) < detection_param_.nms_param().threshold();
            keep = std::fabs(overlap) < 0.7;
            if (!keep) {
                break;
            }
        }
        if (keep) {
            kept_indices.push_back(idx);
        }
        score_index_vec.pop_back();
    }

    int idx = 0;
    auto parted_itr = std::stable_partition(lights->begin(), lights->end(), [&](const base::TrafficLightPtr& light) {
        return std::find(kept_indices.begin(), kept_indices.end(), idx++) != kept_indices.end();
    });
    lights->erase(parted_itr, lights->end());
}

REGISTER_TRAFFIC_LIGHT_DETECTOR(TrafficLightTLDetectorYolox);

}  // namespace trafficlight
}  // namespace perception
}  // namespace apollo
