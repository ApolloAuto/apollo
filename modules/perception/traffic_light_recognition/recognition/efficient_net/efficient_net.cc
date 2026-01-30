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

#include "modules/perception/traffic_light_recognition/recognition/efficient_net/efficient_net.h"

#include "cyber/common/file.h"
#include "cyber/profiler/profiler.h"

#include "modules/perception/common/util.h"
#include "modules/perception/common/inference/model_util.h"
#include "modules/perception/common/inference/utils/resize.h"

namespace apollo {
namespace perception {
namespace trafficlight {

using apollo::cyber::common::GetProtoFromFile;

bool EfficientNetRecognition::Init(const TrafficLightRecognitorInitOptions& options) {
    gpu_id_ = options.gpu_id;
    BASE_GPU_CHECK(cudaSetDevice(gpu_id_));
    BASE_GPU_CHECK(cudaStreamCreate(&stream_));

    std::string config_file = GetConfigFile(options.config_path, options.config_file);
    ACHECK(apollo::cyber::common::GetProtoFromFile(config_file, &model_param_));
    resize_height_ = model_param_.classify_resize_height();
    resize_width_ = model_param_.classify_resize_width();
    unknown_threshold_ = model_param_.classify_threshold();
    scale_ = model_param_.scale();
    mean_.reset(new base::Blob<float>(1, 3, 1, 1));
    float* p = mean_->mutable_cpu_data();
    if (model_param_.is_bgr()) {
        data_provider_image_option_.target_color = base::Color::BGR;
        p[0] = model_param_.mean_b();
        p[1] = model_param_.mean_g();
        p[2] = model_param_.mean_r();
    } else {
        data_provider_image_option_.target_color = base::Color::RGB;
        p[0] = model_param_.mean_r();
        p[1] = model_param_.mean_g();
        p[2] = model_param_.mean_b();
    }
    std::vector<int> shape = {1, 3, resize_height_, resize_width_};
    mean_buffer_.reset(new base::Blob<float>(shape));

    // Init model
    if (!InitModel()) {
        AERROR << "Init model error.";
        return false;
    }
    // image data
    image_.reset(new base::Image8U(resize_height_, resize_width_, base::Color::BGR));

    return true;
}

bool EfficientNetRecognition::InitModel() {
    // name:blob maps
    std::map<std::string, std::vector<int>> blob_maps;
    auto model_info = model_param_.info();
    // efficientnet model file
    std::string model_path = GetModelPath(model_info.name());
    std::string onnx_file = GetModelFile(model_path, model_info.proto_file().file());
    std::vector<std::string> input_names = inference::GetBlobNames(model_info.inputs());
    std::vector<std::string> output_names = inference::GetBlobNames(model_info.outputs());
    inference_ = apollo::cyber::plugin_manager::PluginManager::Instance()->CreateInstance<inference::Inference>(
            "apollo::perception::inference::" + model_info.infer_plugin());
    inference_->set_model_info(onnx_file, input_names, output_names);
    inference_->set_gpu_id(gpu_id_);
    inference_->set_max_batch_size(model_param_.max_batch_size() * 2);

    inference::AddShape(&blob_maps, model_info.inputs());
    inference::AddShape(&blob_maps, model_info.outputs());

    inference_->Init(blob_maps);

    // init input blob
    if (input_names.size() == 1) {
        input_blob_ = inference_->get_blob(input_names.at(0));
        CHECK_NOTNULL(input_blob_.get());
    } else {
        AERROR << "Init inferece input blob error.";
        return false;
    }
    // init output blobs
    if (output_names.size() == 2) {
        outputs_cls_ = inference_->get_blob(output_names.at(0));
        outputs_status_ = inference_->get_blob(output_names.at(1));
        CHECK_NOTNULL(outputs_cls_.get());
        CHECK_NOTNULL(outputs_status_.get());
    } else {
        AERROR << "Init inferece output blob error.";
        return false;
    }

    return true;
}

bool EfficientNetRecognition::Detect(camera::TrafficLightFrame* frame) {
    if (frame->traffic_lights.size() <= 0) {
        AINFO << "No recognizable traffic lights";
        return true;
    }
    std::vector<base::TrafficLightPtr> candidate;
    candidate.reserve(frame->traffic_lights.size());
    // get traffic light region
    for (base::TrafficLightPtr light : frame->traffic_lights) {
        if (light->region.is_detected) {
            candidate.push_back(light);
        } else {
            light->status.color = base::TLColor::TL_UNKNOWN_COLOR;
            light->status.confidence = 0;
        }
    }
    // classify color
    Perform(frame, &candidate);

    return true;
}

void EfficientNetRecognition::Perform(
        const camera::TrafficLightFrame* frame,
        std::vector<base::TrafficLightPtr>* lights) {
    // input reshape
    input_blob_->Reshape({lights->size(), 3, resize_height_, resize_width_});
    // output shape
    outputs_cls_->Reshape({lights->size(), 4});
    outputs_status_->Reshape({lights->size(), 1});

    // classify
    int valid_index = 0;
    for (base::TrafficLightPtr light : *lights) {
        if (!light->region.is_detected) {
            continue;
        }
        // get image
        data_provider_image_option_.crop_roi = light->region.detection_roi;
        data_provider_image_option_.do_crop = true;
        data_provider_image_option_.target_color = base::Color::BGR;
        frame->data_provider->GetImage(data_provider_image_option_, image_.get());
        AINFO << "get img done";
        // image zero padding
        base::Image8U roi_tmp_image;
        int dst_height = 0;
        int dst_width = 0;
        int provider_w = frame->data_provider->src_width();
        int provider_h = frame->data_provider->src_height();
        ZeroPadding(light->region.detection_roi, &roi_tmp_image, &dst_width, &dst_height, provider_w, provider_h);
        // image reshape
        const float* mean = mean_.get()->cpu_data();
        inference::ResizeGPU(
                roi_tmp_image,
                input_blob_,
                roi_tmp_image.cols(),
                valid_index,
                mean[0],
                mean[1],
                mean[2],
                false,
                scale_);
        valid_index += 1;
        AINFO << "resize gpu finish.";
    }
    PERF_BLOCK("traffic_light_recognition_inference")
    inference_->Infer();
    PERF_BLOCK_END
    AINFO << "infer finish.";

    valid_index = 0;  // reset to 0
    for (base::TrafficLightPtr light : *lights) {
        if (!light->region.is_detected) {
            continue;
        }
        // get output result
        float* output_data = outputs_cls_->mutable_cpu_data() + outputs_cls_->offset(valid_index);
        // float *output_status_data = outputs_status_->mutable_cpu_data();
        Prob2Color(output_data, unknown_threshold_, light);
        valid_index += 1;
    }
}

void EfficientNetRecognition::ZeroPadding(
        const base::RectI& light_roi,
        base::Image8U* roi_tmp_image,
        int* dst_width,
        int* dst_height,
        const int& provider_w,
        const int& provider_h) {
    int roi_width = light_roi.width;
    int roi_height = light_roi.height;
    int max_len = std::max(roi_height, roi_width);
    int min_len = std::min(roi_height, roi_width);
    float edge_ratio = static_cast<float>(max_len) / static_cast<float>(min_len);

    int pad_left = 0;
    int pad_right = 0;
    int pad_up = 0;
    int pad_down = 0;
    if (edge_ratio > 1.4 && roi_height > roi_width) {
        pad_left = (roi_height - roi_width) / 2;
        pad_right = roi_height - roi_width - pad_left;
    } else if (edge_ratio > 1.4 && roi_width > roi_height) {
        pad_up = (roi_width - roi_height) / 2;
        pad_down = roi_width - roi_height - pad_up;
    } else {
        int height_dis = (max_len - roi_height) / 2;
        int width_dis = (max_len - roi_width) / 2;
        pad_up = max_len + height_dis;
        pad_down = 3 * max_len - roi_height - pad_up;
        pad_left = max_len + width_dis;
        pad_right = 3 * max_len - roi_width - pad_left;
    }
    *dst_height = roi_height + pad_up + pad_down;
    *dst_width = roi_width + pad_left + pad_right;
    if (*dst_height > provider_h) {
        int dist = *dst_height - provider_h;
        pad_up = pad_up - dist / 2 - 1;
        pad_down = pad_down - dist / 2 - 1;
    }
    if (*dst_width > provider_w) {
        int dist = *dst_width - provider_w;
        pad_left = pad_left - dist / 2 - 1;
        pad_right = pad_right - dist / 2 - 1;
    }

    *dst_height = roi_height + pad_up + pad_down;
    *dst_width = roi_width + pad_left + pad_right;
    *roi_tmp_image = base::Image8U(*dst_height, *dst_width, base::Color::BGR);
    float scale = resize_width_ * 1.0 / *dst_width;
    inference::ImageZeroPadding(
            *image_, roi_tmp_image, provider_w, pad_left, pad_right, pad_up, pad_down, 0, stream_, true);
}

void EfficientNetRecognition::Prob2Color(const float* output_data, float threshold, base::TrafficLightPtr light) {
    int max_color_id = 0;
    std::vector<base::TLColor> status_map
            = {base::TLColor::TL_BLACK, base::TLColor::TL_RED, base::TLColor::TL_YELLOW, base::TLColor::TL_GREEN};
    std::vector<std::string> name_map = {"Black", "Red", "Yellow", "Green"};
    std::vector<float> prob(output_data, output_data + status_map.size());
    auto max_prob = std::max_element(prob.begin(), prob.end());
    max_color_id = (*max_prob > threshold) ? static_cast<int>(std::distance(prob.begin(), max_prob)) : 0;
    light->status.color = status_map[max_color_id];
    light->status.confidence = output_data[max_color_id];
    AINFO << "Light status recognized as " << name_map[max_color_id];
    AINFO << "Color Prob:";
    for (size_t j = 0; j < status_map.size(); j++) {
        AINFO << output_data[j];
    }
}

REGISTER_TRAFFIC_LIGHT_DETECTOR(EfficientNetRecognition);

}  // namespace trafficlight
}  // namespace perception
}  // namespace apollo
