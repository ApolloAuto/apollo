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
#include "modules/perception/camera_detection_multi_stage/detector/yolox3d/yolox3d_obstacle_detector.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "cyber/profiler/profiler.h"
#include "modules/perception/camera_detection_multi_stage/detector/yolox3d/postprocess.h"
#include "modules/perception/common/base/image.h"
#include "modules/perception/common/inference/inference_factory.h"
#include "modules/perception/common/inference/utils/resize.h"
#include "modules/perception/common/util.h"

namespace apollo {
namespace perception {
namespace camera {

using cyber::common::GetAbsolutePath;

void Yolox3DObstacleDetector::LoadInputShape(const yolox3d::ModelParam &model_param) {
    // loading config params
    width_ = model_param.resize().width();
    height_ = model_param.resize().height();

    image_width_ = options_.image_width;
    image_height_ = options_.image_height;

    AINFO << " image_height =" << image_height_ << ", image_width=" << image_width_ << ", height=" << height_
          << ", width=" << width_;
}

void Yolox3DObstacleDetector::LoadParam(const yolox3d::ModelParam &model_param) {
    confidence_threshold_ = model_param.confidence_threshold();
    border_ratio_ = model_param.border_ratio();

    // Init NMS proto param by config file
    const auto &nms_param = model_param.nms_param();
    nms_.set_sigma(nms_param.sigma());
    nms_.set_type(nms_param.type());
    nms_.set_threshold(nms_param.threshold());
}

void AddShape3DYolox(
        std::map<std::string, std::vector<int>> *shape_map,
        const google::protobuf::RepeatedPtrField<common::ModelBlob> &model_blobs) {
    for (const auto &blob : model_blobs) {
        std::vector<int> shape(blob.shape().begin(), blob.shape().end());
        shape_map->insert(std::make_pair(blob.name(), shape));
    }
}

std::vector<std::string> GetBlobNames3DYolox(const google::protobuf::RepeatedPtrField<common::ModelBlob> &model_blobs) {
    std::vector<std::string> blob_names;
    for (const auto &blob : model_blobs) {
        blob_names.push_back(blob.name());
    }
    return blob_names;
}

bool Yolox3DObstacleDetector::Init3DNetwork(const common::ModelInfo &model_info, const std::string &model_path) {
    // Network files
    std::string proto_file = GetModelFile(model_path, model_info.proto_file().file());
    std::string weight_file = GetModelFile(model_path, model_info.weight_file().file());

    // Network input and output names
    std::vector<std::string> input_names = GetBlobNames3DYolox(model_info.inputs());
    std::vector<std::string> output_names = GetBlobNames3DYolox(model_info.outputs());

    // Network type
    const auto &framework = model_info.framework();
    std::string plugin_name = model_info.infer_plugin();
    static const std::string class_namespace = "apollo::perception::inference::";
    if (model_info.has_infer_plugin() && !plugin_name.empty()) {
        plugin_name = class_namespace + plugin_name;
        net_3D_ = apollo::cyber::plugin_manager::PluginManager::Instance()->CreateInstance<inference::Inference>(
                plugin_name);
        net_3D_->set_model_info(proto_file, input_names, output_names);
        AINFO << "net_3D load plugin success: " << plugin_name;
    } else {
        net_3D_.reset(inference::CreateInferenceByName(
                framework, proto_file, weight_file, output_names, input_names, model_path));
    }

    ACHECK(net_3D_ != nullptr);
    net_3D_->set_gpu_id(gpu_id_);
    net_3D_->set_max_batch_size(max_batch_size_);

    std::map<std::string, std::vector<int>> shape_map;
    AddShape3DYolox(&shape_map, model_info.inputs());
    AddShape3DYolox(&shape_map, model_info.outputs());

    if (!net_3D_->Init(shape_map)) {
        AERROR << model_info.name() << "init failed!";
        return false;
    }
    return true;
}

cv::Mat resizeKeepAspectRatioYolox(const cv::Mat &input, const cv::Size &dstSize, const cv::Scalar &bgcolor) {
    cv::Mat output;

    double h1 = dstSize.width * (input.rows / static_cast<double>(input.cols));
    double w2 = dstSize.height * (input.cols / static_cast<double>(input.rows));
    if (h1 <= dstSize.height) {
        cv::resize(input, output, cv::Size(dstSize.width, h1));
    } else {
        cv::resize(input, output, cv::Size(w2, dstSize.height));
    }

    int top = (dstSize.height - output.rows) / 2;
    int down = (dstSize.height - output.rows + 1) / 2;
    int left = (dstSize.width - output.cols) / 2;
    int right = (dstSize.width - output.cols + 1) / 2;

    cv::copyMakeBorder(output, output, top, down, left, right, cv::BORDER_CONSTANT, bgcolor);

    return output;
}

void Yolox3DObstacleDetector::Yolo3DInference(const base::Image8U *image, std::vector<base::ObjectPtr> &detect_objs) {
    ACHECK(image != nullptr);
    // init input blobs data space
    auto model_inputs = model_param_.info_3d().info().inputs();

    auto input_blob_3d = net_3D_->get_blob(model_inputs[0].name());
    ACHECK(input_blob_3d != nullptr);

    int model_input_rows = 224;
    int model_input_cols = 224;
    int model_input_chs = 3;
    // reshape max batch size
    input_blob_3d->Reshape({max_batch_size_, model_input_chs, model_input_rows, model_input_cols});

    // image preprocess
    PERF_BLOCK("net_3D_ preprocess, batch_size: unknown")
    for (int obj_id = 0; obj_id < (int)detect_objs.size(); obj_id++) {
        float *input_data = input_blob_3d->mutable_cpu_data() + input_blob_3d->offset(obj_id);

        if (obj_id >= max_batch_size_) {
            AERROR << "The detected target in this frame is greater than max batch "
                      "size.";
            AERROR << "which is max batch size " << max_batch_size_;
            AERROR << "Abandon greater than part.";
            continue;
        }

        auto obj = detect_objs[obj_id];
        obj->camera_supplement.area_id = 1;

        // get left top and right bottom point
        float left_x = Yoloxclamp(obj->camera_supplement.box.xmin, 0.f, static_cast<float>(image->cols()));
        float left_y = Yoloxclamp(obj->camera_supplement.box.ymin, 0.f, static_cast<float>(image->rows()));
        float right_x = Yoloxclamp(obj->camera_supplement.box.xmax, 0.f, static_cast<float>(image->cols()));
        float right_y = Yoloxclamp(obj->camera_supplement.box.ymax, 0.f, static_cast<float>(image->rows()));

        float width = right_x - left_x;
        float height = right_y - left_y;

        cv::Rect object_roi(left_x, left_y, width, height);
        cv::Mat cropped_obj = img_(object_roi);
        // resize croped image to 224*224, no padding
        cv::resize(cropped_obj, cropped_obj, cv::Size(224, 224));
        cropped_obj.convertTo(cropped_obj, CV_32F, 1.0f / 255.0);

        // BGR
        std::vector<float> mean_values{0.406, 0.456, 0.485};
        std::vector<float> std_values{0.225, 0.224, 0.229};

        // normallize channel value from 0～255 to 0~1 and change it to float type
        std::vector<cv::Mat> bgrChannels(3);
        cv::split(cropped_obj, bgrChannels);
        for (int i = 0; i < 3; ++i) {
            bgrChannels[i].convertTo(
                    bgrChannels[i], CV_32FC1, 1 / std_values[i], (0.0 - mean_values[i]) / std_values[i]);
        }
        cv::Mat dst;
        cv::merge(bgrChannels, dst);
        cropped_obj = dst;

        for (int i = 0; i < model_input_chs; ++i) {
            // put img model_input_chs i data to input_blob
            cv::extractChannel(
                    cropped_obj,
                    cv::Mat(model_input_rows,
                            model_input_cols,
                            CV_32FC1,
                            input_data + i * model_input_rows * model_input_cols),
                    i);
        }
    }
    int total_objs = max_batch_size_ < (int)detect_objs.size() ? max_batch_size_ : (int)detect_objs.size();
    // reshape real batch
    input_blob_3d->Reshape({total_objs, model_input_chs, model_input_rows, model_input_cols});

    // get model output
    auto model_outputs = model_param_.info_3d().info().outputs();
    auto blob_orient = net_3D_->get_blob(model_outputs[0].name());  // sin and cos 0～1
    auto blob_conf = net_3D_->get_blob(model_outputs[1].name());    // front or back 0 or 1
    auto blob_dim = net_3D_->get_blob(model_outputs[2].name());     // deviation value

    // reshape dynamic output blob, for multibatch plugin
    blob_orient->Reshape({total_objs, 1, 2, 2});
    blob_conf->Reshape({total_objs, 1, 2, 1});
    blob_dim->Reshape({total_objs, 1, 3, 1});
    PERF_BLOCK_END;

    // run infer
    PERF_BLOCK("net_3D_ infer, batch_size: " + std::to_string(total_objs))
    net_3D_->Infer();
    PERF_BLOCK_END;

    ACHECK(blob_orient != nullptr);
    ACHECK(blob_conf != nullptr);
    ACHECK(blob_dim != nullptr);

    PERF_BLOCK("net_3D_ postprocess, batch_size: " + std::to_string(total_objs))
    const int bin_number = model_param_.info_3d().bin_num();
    // post processing
    float border_ratio = 0.01;
    int left_boundary = static_cast<int>(border_ratio * static_cast<float>(image->cols()));
    int right_boundary = static_cast<int>((1.0f - border_ratio) * static_cast<float>(image->cols()));

    for (int obj_id = 0; obj_id < total_objs; obj_id++) {
        auto orient_data = blob_orient->mutable_cpu_data() + blob_orient->offset(obj_id);
        auto conf_data = blob_conf->mutable_cpu_data() + blob_conf->offset(obj_id);
        auto dim_data = blob_dim->mutable_cpu_data() + blob_dim->offset(obj_id);
        auto &obj = detect_objs[obj_id];

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

        float wedge = 2 * M_PI / bin_number;
        alpha = angle_offset + max_index * wedge;
        alpha = alpha + wedge / 2 - M_PI;
        if (alpha > M_PI) {
            alpha -= 2 * M_PI;
        }
        obj->camera_supplement.alpha = alpha;

        // add dim to object here
        obj->size[0] += dim_data[2];
        obj->size[1] += dim_data[1];
        obj->size[2] += dim_data[0];

        // clear cut off ratios
        auto &box = obj->camera_supplement.box;
        if (box.xmin >= left_boundary) {
            obj->camera_supplement.cut_off_ratios[2] = 0;
        }
        if (box.xmax <= right_boundary) {
            obj->camera_supplement.cut_off_ratios[3] = 0;
        }
    }
    PERF_BLOCK_END;
    return;
}

bool Yolox3DObstacleDetector::Init(const ObstacleDetectorInitOptions &options) {
    options_ = options;

    gpu_id_ = options.gpu_id;
    BASE_GPU_CHECK(cudaSetDevice(gpu_id_));
    BASE_GPU_CHECK(cudaStreamCreate(&stream_));

    std::string config_file = GetConfigFile(options.config_path, options.config_file);
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
    net_->Infer();
    AERROR << "[INFO] yolox3d 2D model init success";

    const auto &model_info_3d = model_param_.info_3d().info();
    std::string model_path_3d = GetModelPath(model_info_3d.name());
    if (!Init3DNetwork(model_info_3d, model_path_3d)) {
        AERROR << "Init network failed!";
        return false;
    }
    net_3D_->Infer();
    AERROR << "[INFO] yolox3d 3D model init success";

    return true;
}

bool Yolox3DObstacleDetector::Preprocess(const base::Image8U *image, base::BlobPtr<float> input_blob) {
    ACHECK(image != nullptr);
    ACHECK(input_blob != nullptr);

    // init cv img containter, same to image row and col
    img_ = cv::Mat(image->rows(), image->cols(), CV_8UC3);
    memcpy(img_.data, image->cpu_data(), image->rows() * image->cols() * image->channels() * sizeof(uint8_t));

    // generate new pure black bg as same size as ratio
    float ratio = std::max(image_width_, image_height_);
    cv::Mat out(ratio, ratio, CV_8UC3, {114, 114, 114});

    // pad to bottom
    img_.copyTo(out(cv::Rect(0, 0, img_.cols, img_.rows)));

    cv::resize(out, out, cv::Size(width_, height_), cv::INTER_LINEAR);

    out.convertTo(out, CV_32F, 1.0);
    // cv::imwrite("pics/yolox3d_pad_image.png", out);

    int model_input_chs = out.channels();
    int model_input_rows = out.rows;
    int model_input_cols = out.cols;

    // fill input_blob -> tensor_image will be used by model
    input_blob->Reshape({1, model_input_chs, model_input_rows, model_input_cols});
    float *input_data = input_blob->mutable_cpu_data();

    for (int i = 0; i < model_input_chs; ++i) {
        cv::extractChannel(
                out,
                cv::Mat(model_input_rows,
                        model_input_cols,
                        CV_32FC1,
                        input_data + i * model_input_rows * model_input_cols),
                i);
    }
    return true;
}

bool Yolox3DObstacleDetector::Detect(onboard::CameraFrame *frame) {
    if (frame == nullptr) {
        return false;
    }

    if (cudaSetDevice(gpu_id_) != cudaSuccess) {
        AERROR << "Failed to set device to " << gpu_id_;
        return false;
    }

    auto model_inputs = model_param_.info().inputs();
    auto input_blob_2d = net_->get_blob(model_inputs[0].name());

    DataProvider::ImageOptions image_options;
    image_options.target_color = base::Color::BGR;

    base::Image8U image;
    frame->data_provider->GetImage(image_options, &image);

    //////////////////////////
    // gpu preprocess
    int left_pad = 0;
    int right_pad = 0;
    int top_pad = 0;
    int bottom_pad = 0;
    int roi_width = image_width_;
    int roi_height = image_height_;

    // do down pad on image
    if (roi_width > roi_height) {
        top_pad = 0;
        bottom_pad = roi_width - roi_height;
    } else if (roi_height > roi_width) {
        left_pad = 0;
        right_pad = roi_height - roi_width;
    }

    int dst_height = roi_height + top_pad + bottom_pad;
    int dst_width = roi_width + left_pad + right_pad;

    base::Image8U padding_image_container_ = base::Image8U(dst_width, dst_height, base::Color::BGR);

    base::Image8U tmp_image = padding_image_container_(base::RectI(0, 0, dst_width, dst_height));

    PERF_BLOCK("2d padding stage")
    inference::ImageZeroPadding(
            image, &tmp_image, roi_width, left_pad, right_pad, top_pad, bottom_pad, 114, stream_, true);
    PERF_BLOCK_END;

    input_blob_2d->Reshape(
            {input_blob_2d->shape(0), input_blob_2d->shape(1), input_blob_2d->shape(2), input_blob_2d->shape(3)});

    PERF_BLOCK("2d resize stage")
    int mean_bgr[3] = {114, 114, 114};
    inference::ResizeGPU(
            tmp_image,
            input_blob_2d,
            frame->data_provider->src_width(),
            0,
            mean_bgr[0],
            mean_bgr[1],
            mean_bgr[2],
            false,
            1.0);
    PERF_BLOCK_END;
    // //////////////////////////

    // model infer, save to output blob
    net_->Infer();

    img_ = cv::Mat(image.rows(), image.cols(), CV_8UC3);
    memcpy(img_.data, image.cpu_data(), image.rows() * image.cols() * image.channels() * sizeof(uint8_t));

    // get objects from network inference result
    auto model_outputs = model_param_.info().outputs();

    // Attention(lordon): onnx predict blob is 1, pth is 0
    auto predict_blob = net_->get_blob(model_outputs[1].name());
    frame->feature_blob = net_->get_blob(model_outputs[0].name());

    PERF_BLOCK("2d postprocess stage")
    YoloxGetObjectsCpu(
            predict_blob, model_param_, nms_, width_, height_, image_width_, image_height_, &frame->detected_objects);
    PERF_BLOCK_END;

    // post process for 3d
    Yolo3DInference(&image, frame->detected_objects);
    return true;
}
REGISTER_OBSTACLE_DETECTOR(Yolox3DObstacleDetector);

}  // namespace camera
}  // namespace perception
}  // namespace apollo
