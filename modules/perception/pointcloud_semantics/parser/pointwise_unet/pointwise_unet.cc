/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/perception/pointcloud_semantics/parser/pointwise_unet/pointwise_unet.h"

#include "cyber/common/file.h"
#include "modules/perception/common/util.h"
#include "modules/perception/common/inference/model_util.h"
#include "modules/perception/common/lidar/common/lidar_point_label.h"
#include "modules/perception/common/lidar/common/lidar_timer.h"

namespace apollo {
namespace perception {
namespace lidar {

using apollo::cyber::common::GetProtoFromFile;

bool PointWiseUnet::Init(const PointCloudParserInitOptions& options) {
    std::string config_file = GetConfigFile(options.config_path, options.config_file);
    ACHECK(apollo::cyber::common::GetProtoFromFile(config_file, &model_param_));
    AINFO << "PointWise unet Configs: " << model_param_.DebugString();
    // gpu
    use_gpu_ = model_param_.use_gpu();

    // Unet params
    auto unet_params = model_param_.unet_params();
    resolution_ = unet_params.resolution();
    unet_width_ = unet_params.unet_width();
    unet_length_ = unet_params.unet_length();
    min_height_ = unet_params.min_height();
    max_height_ = unet_params.max_height();
    min_x_ = unet_params.min_x();
    max_x_ = unet_params.max_x();
    min_y_ = unet_params.min_y();
    max_y_ = unet_params.max_y();
    max_point_number_ = unet_params.max_point_number();

    // unet nodes init
    unet_nodes_.clear();
    unet_nodes_.resize(unet_width_ * unet_length_);

    // Init model
    if (!InitModel()) {
        AERROR << "Init model error.";
        return false;
    }
    // create cuda stream
    cudaStreamCreate(&stream_);

    // semantic class mapping
    sematic_class_mapping_.push_back(static_cast<uint8_t>(PointSemanticLabel::OBJECT));
    sematic_class_mapping_.push_back(static_cast<uint8_t>(PointSemanticLabel::GROUND));
    sematic_class_mapping_.push_back(static_cast<uint8_t>(PointSemanticLabel::CURB));
    sematic_class_mapping_.push_back(static_cast<uint8_t>(PointSemanticLabel::VEGETATION));
    sematic_class_mapping_.push_back(static_cast<uint8_t>(PointSemanticLabel::FENCE));
    sematic_class_mapping_.push_back(static_cast<uint8_t>(PointSemanticLabel::IGNORE));
    sematic_class_mapping_.push_back(static_cast<uint8_t>(PointSemanticLabel::NOISE));
    CHECK_EQ(sematic_class_mapping_.size(), 7) << "class num and semantic map size not match";

    return true;
}

bool PointWiseUnet::Parse(const PointCloudParserOptions& options, LidarFrame* frame) {
    // check input
    if (frame == nullptr) {
        AERROR << "Input null frame ptr.";
        return false;
    }
    if (frame->cloud == nullptr) {
        AERROR << "Input null frame cloud.";
        return false;
    }
    if (frame->cloud->size() == 0) {
        AERROR << "Input none points.";
        return false;
    }

    // record input cloud and lidar frame
    original_cloud_ = frame->cloud;
    original_world_cloud_ = frame->world_cloud;
    lidar_frame_ref_ = frame;
    if (cudaSetDevice(model_param_.preprocess().gpu_id()) != cudaSuccess) {
        AERROR << "Failed to set device to " << model_param_.preprocess().gpu_id();
        return false;
    }

    Timer timer;
    GeneratePfnFeature();
    double pfn_feature_generate_time = timer.toc(true);

    pfn_inference_->Infer();
    double pfn_infer_time = timer.toc(true);

    GenerateBackboneFeature();
    double backbone_feature_generate_time = timer.toc(true);

    inference_->Infer();
    double backbone_infer_time = timer.toc(true);

    LabelToPointCloud(frame);
    double label_pointcloud_time = timer.toc(true);

    AINFO << "PointUnet time: "
          << "pfn feature generate time: " << pfn_feature_generate_time << "ms, "
          << "pfn infer time: " << pfn_infer_time << "ms, "
          << "backbone feature generate time: " << backbone_feature_generate_time << "ms, "
          << "backbone infer time: " << backbone_infer_time << "ms, "
          << "pointcloud label time: " << label_pointcloud_time << "ms.";

    return true;
}

bool PointWiseUnet::InitModel() {
    // fpn name:blob maps
    std::map<std::string, std::vector<int>> pfn_blob_maps;
    // pfn model
    auto pfn_info = model_param_.pfn_info();
    auto preprocess = model_param_.preprocess();

    // pfn model file
    std::string model_path = GetModelPath(pfn_info.name());
    std::string pfn_proto_file = GetModelFile(model_path, pfn_info.proto_file().file());
    std::vector<std::string> pfn_input_names = inference::GetBlobNames(pfn_info.inputs());
    std::vector<std::string> pfn_output_names = inference::GetBlobNames(pfn_info.outputs());
    pfn_inference_ = apollo::cyber::plugin_manager::PluginManager::Instance()->CreateInstance<inference::Inference>(
            "apollo::perception::inference::" + pfn_info.infer_plugin());
    pfn_inference_->set_model_info(pfn_proto_file, pfn_input_names, pfn_output_names);
    pfn_inference_->set_gpu_id(preprocess.gpu_id());
    pfn_inference_->set_max_batch_size(max_point_number_ * 2);

    inference::AddShape(&pfn_blob_maps, pfn_info.inputs());
    inference::AddShape(&pfn_blob_maps, pfn_info.outputs());

    pfn_inference_->Init(pfn_blob_maps);
    // init pfn input blob
    if (pfn_input_names.size() == 1) {
        voxels_ = pfn_inference_->get_blob(pfn_input_names.at(0));
        CHECK_NOTNULL(voxels_.get());
    } else {
        AERROR << "Init pfn_inferece input blob error.";
        return false;
    }
    // init pfn output blobs
    if (pfn_output_names.size() == 1) {
        pfn_point_feats_ = pfn_inference_->get_blob(pfn_output_names.at(0));
        CHECK_NOTNULL(pfn_point_feats_.get());
    } else {
        AERROR << "Init pfn_inferece output blob error.";
        return false;
    }
    AINFO << "Init pointwise unet pfn model success.";

    // backbone name:blob maps
    std::map<std::string, std::vector<int>> blob_maps;
    // backbone info
    auto info = model_param_.info();
    // backbone model file
    std::string proto_file = GetModelFile(model_path, info.proto_file().file());
    std::vector<std::string> input_names = inference::GetBlobNames(info.inputs());
    std::vector<std::string> output_names = inference::GetBlobNames(info.outputs());
    inference_ = apollo::cyber::plugin_manager::PluginManager::Instance()->CreateInstance<inference::Inference>(
            "apollo::perception::inference::" + info.infer_plugin());
    inference_->set_model_info(proto_file, input_names, output_names);
    inference_->set_gpu_id(preprocess.gpu_id());
    inference_->set_max_batch_size(max_point_number_ * 2);

    inference::AddShape(&blob_maps, info.inputs());
    inference::AddShape(&blob_maps, info.outputs());

    inference_->Init(blob_maps);
    // init backbone input blobs
    // Caution: the order of input-names between pointwise_unet.h
    //   and pointwise_unet_param.pb.txt should be the same.
    if (input_names.size() == 3) {
        coors_ = inference_->get_blob(input_names.at(0));
        point_feats_ = inference_->get_blob(input_names.at(1));
        bev_feature_ = inference_->get_blob(input_names.at(2));

        CHECK_NOTNULL(coors_.get());
        CHECK_NOTNULL(point_feats_.get());
        CHECK_NOTNULL(bev_feature_.get());
    } else {
        AERROR << "Init inferece input blobs error.";
        return false;
    }
    // init output blob
    if (output_names.size() == 1) {
        seg_pred_blob_ = inference_->get_blob(output_names.at(0));
        CHECK_NOTNULL(seg_pred_blob_.get());
    } else {
        AERROR << "Init inferece output blobs error.";
        return false;
    }
    AINFO << "Init pointwise unet backbone model success.";

    return true;
}

void PointWiseUnet::GeneratePfnFeatureCPU() {
    // clear unet nodes
    unet_nodes_.clear();
    std::vector<UnetNode>().swap(unet_nodes_);
    unet_nodes_.resize(unet_width_ * unet_length_);

    // valid point index
    valid_point_index_.clear();
    valid_point_index_.reserve(original_cloud_->size());

    // valid node of unet
    valid_node_index_.clear();
    valid_node_index_.reserve(unet_width_ * unet_length_);

    // generate pfn model feature
    for (int i = 0; i < original_cloud_->size(); i++) {
        auto pt = original_cloud_->at(i);
        // check valid range
        if (pt.x < min_x_ || pt.x > max_x_ || pt.y < min_y_ || pt.y > max_y_ || pt.z < min_height_
            || pt.z > max_height_) {
            continue;
        }
        // get coordinate of pointcloud bev
        int grid_x = (pt.x - min_x_) / resolution_;
        int grid_y = (pt.y - min_y_) / resolution_;
        if (grid_x >= 0 && grid_x < unet_width_ && grid_y >= 0 && grid_y < unet_length_) {
            // unet bev grid_x and grid_y
            int pillar_idx = grid_y * unet_length_ + grid_x;
            // valid point index
            valid_point_index_.push_back(i);
            // valid node index
            valid_node_index_.push_back(pillar_idx);

            // unet bev node
            // gravity center
            unet_nodes_.at(pillar_idx).sum_x += pt.x;
            unet_nodes_.at(pillar_idx).sum_y += pt.y;
            unet_nodes_.at(pillar_idx).sum_z += pt.z;
            if (unet_nodes_.at(pillar_idx).point_size == 0) {
                // geometric center
                unet_nodes_.at(pillar_idx).geometric_x
                        = static_cast<float>(grid_x) * resolution_ + 0.5 * resolution_ + min_x_;
                unet_nodes_.at(pillar_idx).geometric_y
                        = static_cast<float>(grid_y) * resolution_ + 0.5 * resolution_ + min_y_;
                unet_nodes_.at(pillar_idx).geometric_z = 0;
                // grid coordinate
                unet_nodes_.at(pillar_idx).grid_x = grid_x;
                unet_nodes_.at(pillar_idx).grid_y = grid_y;
            }
            unet_nodes_.at(pillar_idx).point_size += 1;

            // max point number
            if (valid_point_index_.size() >= max_point_number_) {
                break;
            }
        }
    }

    // pfn input feature
    voxels_->Reshape({valid_point_index_.size(), 10});
    auto voxel_cpu_data = voxels_->mutable_cpu_data();
    // output
    pfn_point_feats_->Reshape({valid_point_index_.size(), 64});
    // backbone input feature
    coors_->Reshape({valid_point_index_.size(), 4});
    auto coors_cpu_data = coors_->mutable_cpu_data();
    for (size_t i = 0; i < valid_point_index_.size(); i++) {
        auto pt = original_cloud_->at(valid_point_index_.at(i));
        UnetNode node = unet_nodes_.at(valid_node_index_.at(i));
        // generate voxels feature
        voxel_cpu_data[i * 10 + 0] = pt.x;
        voxel_cpu_data[i * 10 + 1] = pt.y;
        voxel_cpu_data[i * 10 + 2] = pt.z;
        voxel_cpu_data[i * 10 + 3] = pt.intensity;
        voxel_cpu_data[i * 10 + 4] = pt.x - node.sum_x / (node.point_size + 1e-6);
        voxel_cpu_data[i * 10 + 5] = pt.y - node.sum_y / (node.point_size + 1e-6);
        voxel_cpu_data[i * 10 + 6] = pt.z - node.sum_z / (node.point_size + 1e-6);
        voxel_cpu_data[i * 10 + 7] = pt.x - node.geometric_x;
        voxel_cpu_data[i * 10 + 8] = pt.y - node.geometric_y;
        voxel_cpu_data[i * 10 + 9] = pt.z - node.geometric_z;

        // generate coors feauture
        coors_cpu_data[i * 4 + 0] = 0;
        coors_cpu_data[i * 4 + 1] = 0;
        coors_cpu_data[i * 4 + 2] = node.grid_y;
        coors_cpu_data[i * 4 + 3] = node.grid_x;
    }
}

void PointWiseUnet::GenerateBackboneFeatureCPU() {
    // generate point_feats feature
    point_feats_->Reshape(pfn_point_feats_->shape());
    point_feats_->set_gpu_data(pfn_point_feats_->mutable_gpu_data());

    // get pfn model output
    auto point_feats_cpu = pfn_point_feats_->mutable_cpu_data();

    // generate voxel_features_ and feature_coors_
    bev_feature_->Reshape({1, 64, unet_length_, unet_width_});
    auto bev_feature_data = bev_feature_->mutable_cpu_data();
    memset(bev_feature_data, 0, sizeof(float) * 64 * unet_length_ * unet_width_);

    for (auto i = 0; i < valid_node_index_.size(); i++) {
        int pillar_idx = valid_node_index_.at(i);
        UnetNode node = unet_nodes_.at(pillar_idx);
        // voxel_features_
        for (int j = 0; j < 64; j++) {
            int index = j * unet_length_ * unet_width_ + node.grid_y * unet_length_ + node.grid_x;
            float value = bev_feature_data[index];
            float max_val = point_feats_cpu[i * 64 + j];
            max_val = std::max(max_val, value);
            bev_feature_data[index] = max_val;
        }
    }

    // output
    seg_pred_blob_->Reshape({max_point_number_, 7});
}

void PointWiseUnet::LabelToPointCloud(LidarFrame* frame) {
    // check input
    if (!original_cloud_ || !original_world_cloud_) {
        AERROR << "Fail to labeling, original_cloud_ or original_world_cloud_ is empty !";
        return;
    }

    auto sematic_result = seg_pred_blob_->mutable_cpu_data();
    size_t ground_count = 0;
    float ground_height = 0.0;
    for (size_t i = 0; i < valid_point_index_.size(); i++) {
        int max_index = 0;
        float max_prob = std::numeric_limits<float>::min();
        for (int n = 0; n < 7; n++) {
            float prob = sematic_result[i * 7 + n];
            if (max_prob < prob) {
                max_prob = prob;
                max_index = n;
            }
        }
        int point_index = valid_point_index_.at(i);
        SetSemanticLabel(
                static_cast<PointSemanticLabel>(sematic_class_mapping_[max_index]),
                &original_cloud_->points_semantic_label(point_index));
        SetSemanticLabel(
                static_cast<PointSemanticLabel>(sematic_class_mapping_[max_index]),
                &original_world_cloud_->points_semantic_label(point_index));

        if (static_cast<PointSemanticLabel>(sematic_class_mapping_[max_index]) == PointSemanticLabel::GROUND) {
            ground_height += original_cloud_->at(point_index).z;
            ++ground_count;
        }
    }
    if (ground_count) {
        frame->parsing_ground_height = ground_height / ((float)(ground_count) * 1.0f);
    }
    AINFO << "This frame " << std::to_string(frame->timestamp) << " parsing ground height is "
          << frame->parsing_ground_height;
}

PERCEPTION_REGISTER_POINTCLOUDPARSER(PointWiseUnet);

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
