/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/lidar_cpdet_detection/detector/cpdet/cpdet.h"

#include <algorithm>
#include <functional>
#include <numeric>
#include <random>
#include <utility>

#include <unistd.h>

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/perception/common/base/object_pool_types.h"
#include "modules/perception/common/base/point_cloud_util.h"
#include "modules/perception/common/base/blob.h"
#include "modules/perception/common/inference/model_util.h"
#include "modules/perception/common/lidar/common/cloud_mask.h"
#include "modules/perception/common/lidar/common/lidar_timer.h"
#include "modules/perception/common/lidar/common/pcl_util.h"
#include "modules/perception/common/lidar/common/config_util.h"
#include "modules/perception/common/util.h"

#include "cyber/profiler/profiler.h"

namespace apollo {
namespace perception {
namespace lidar {

const float MIN_X_RANGE = -51.2;
const float MAX_X_RANGE = 51.2;
const float MIN_Y_RANGE = -51.2;
const float MAX_Y_RANGE = 51.2;
const float MIN_Z_RANGE = -3.5;
const float MAX_Z_RANGE = 3.5;

CPDetection::CPDetection()
    : x_min_range_(MIN_X_RANGE), x_max_range_(MAX_X_RANGE),
      y_min_range_(MIN_Y_RANGE), y_max_range_(MAX_Y_RANGE) {}

bool CPDetection::Init(const CPDetectorInitOptions &options) {
    std::string conf_file =
        GetConfigFile(options.config_path, options.config_file);
    ACHECK(apollo::cyber::common::GetProtoFromFile(conf_file, &cpdet_config_));

    LoadParams(cpdet_config_);

    if (!InitModel(cpdet_config_.model_param())) {
        AERROR << "Init CPDetection Model Failed";
        return false;
    }

    // create cuda stream
    cudaStreamCreate(&stream_);
    pfn_inference_->SetStream(stream_);
    backbone_inference_->SetStream(stream_);
    
    // set private feature blob
    point2grid_blob_.reset(new base::Blob<int>(std::vector<int>{max_voxel_num_}));
    grid2pointnum_blob_.reset(new base::Blob<int>(std::vector<int>{map_size_}));
    
    if (use_gpu_generate_feature_) {
        voxels_blob_->mutable_gpu_data();
        canvas_feature_blob_->mutable_gpu_data();
        point2grid_blob_->mutable_gpu_data();
        grid2pointnum_blob_->mutable_gpu_data();
    } else {
        voxels_blob_->mutable_cpu_data();
        canvas_feature_blob_->mutable_cpu_data();
        point2grid_blob_->mutable_cpu_data();
        grid2pointnum_blob_->mutable_cpu_data();
    }

    int channel_index = pillar_feature_dim_;
    feature_offset_["canvas_feature"] = canvas_feature_blob_->offset(0, 0);
    if (use_cnnseg_features_) {
        feature_offset_["max_height"] = canvas_feature_blob_->offset(0, channel_index++);
        feature_offset_["mean_height"] = canvas_feature_blob_->offset(0, channel_index++);
        feature_offset_["top_intensity"] = canvas_feature_blob_->offset(0, channel_index++);
        feature_offset_["mean_intensity"] = canvas_feature_blob_->offset(0, channel_index++);
        feature_offset_["count"] = canvas_feature_blob_->offset(0, channel_index++);
        feature_offset_["nonempty"] = canvas_feature_blob_->offset(0, channel_index++);
        feature_offset_["height_bin"] = canvas_feature_blob_->offset(0, channel_index);
    }

    max_candidate_num_ = num_tasks_ * nms_pre_max_size_;
    all_res_box_blob_.reset(new base::Blob<float>(std::vector<int>{max_candidate_num_, kBoxBlockSize}));
    all_res_conf_blob_.reset(new base::Blob<float>(std::vector<int>{max_candidate_num_}));
    all_res_cls_blob_.reset(new base::Blob<int>(std::vector<int>{max_candidate_num_}));
    
    res_box_num_blob_.reset(new base::Blob<int>(std::vector<int>{1}));
    res_box_blob_.reset(new base::Blob<float>(std::vector<int>{nms_pre_max_size_, kBoxBlockSize}));
    res_conf_blob_.reset(new base::Blob<float>(std::vector<int>{nms_pre_max_size_}));
    res_cls_blob_.reset(new base::Blob<int>(std::vector<int>{nms_pre_max_size_}));
    
    score_class_map_blob_.reset(new base::Blob<float>(std::vector<int>{num_classes_}));

    res_sorted_indices_blob_.reset(new base::Blob<int>(std::vector<int>{nms_pre_max_size_}));
    box_for_nms_blob_.reset(new base::Blob<float>(std::vector<int>{max_candidate_num_, 4}));
    box_corner_blob_.reset(new base::Blob<float>(std::vector<int>{max_candidate_num_, 8}));
    remain_conf_blob_.reset(new base::Blob<float>(std::vector<int>{nms_pre_max_size_}));
    rotate_overlapped_blob_.reset(new base::Blob<bool>(std::vector<int>{nms_pre_max_size_, nms_pre_max_size_}));
    kept_indices_blob_.reset(new base::Blob<int>(std::vector<int>{num_tasks_ * nms_post_max_size_}));

    box_corners_blob_.reset(new base::Blob<float>(std::vector<int>{max_candidate_num_, 8}));
    box_rects_blob_.reset(new base::Blob<float>(std::vector<int>{max_candidate_num_, 4}));
    valid_point_num_blob_.reset(new base::Blob<int>(std::vector<int>{1}));
    valid_point_indices_blob_.reset(new base::Blob<int>(std::vector<int>{max_valid_point_size_}));
    valid_point2boxid_blob_.reset(new base::Blob<int>(std::vector<int>{max_valid_point_size_}));

    res_box_num_blob_->mutable_gpu_data();
    res_box_blob_->mutable_gpu_data();
    res_conf_blob_->mutable_gpu_data();
    res_cls_blob_->mutable_gpu_data();
    all_res_box_blob_->mutable_gpu_data();
    all_res_conf_blob_->mutable_gpu_data();
    all_res_cls_blob_->mutable_gpu_data();
    score_class_map_blob_->mutable_gpu_data();
    remain_conf_blob_->mutable_gpu_data();
    rotate_overlapped_blob_->mutable_gpu_data();
    kept_indices_blob_->mutable_gpu_data();
    box_corners_blob_->mutable_gpu_data();
    box_rects_blob_->mutable_gpu_data();
    valid_point_num_blob_->mutable_gpu_data();
    valid_point_indices_blob_->mutable_gpu_data();
    valid_point2boxid_blob_->mutable_gpu_data();

    res_box_num_blob_->mutable_cpu_data();
    all_res_box_blob_->mutable_cpu_data();
    all_res_conf_blob_->mutable_cpu_data();
    all_res_cls_blob_->mutable_cpu_data();
    rotate_overlapped_blob_->mutable_cpu_data();
    box_corners_blob_->mutable_cpu_data();
    box_rects_blob_->mutable_cpu_data();
    valid_point_num_blob_->mutable_cpu_data();
    valid_point_indices_blob_->mutable_cpu_data();
    valid_point2boxid_blob_->mutable_cpu_data();

    if (cpdet_config_.has_plugins()) {
        const auto &plugin = cpdet_config_.plugins();
        const auto &name = plugin.name();
        down_sample_ = apollo::cyber::plugin_manager::PluginManager::Instance()
            ->CreateInstance<BaseDownSample>(ConfigUtil::GetFullClassName(name));
        if (!down_sample_) {
            AINFO << "Failed to find down_sample plugin: " << name << ", skipped";
            return false;
        }
        DownSampleInitOptions option;
        option.config_path = plugin.config_path();
        option.config_file = plugin.config_file();
        if (!down_sample_->Init(option)) {
            AINFO << "Failed to init down_sample plugin: " << name << ", skipped";
            return false;
        }
    }
    
    return true;
}

void CPDetection::LoadParams(const cpdet::CPDetConfig &cpdet_config) {
    x_min_range_ = cpdet_config.pre_process().min_x_range();
    x_max_range_ = cpdet_config.pre_process().max_x_range();
    y_min_range_ = cpdet_config.pre_process().min_y_range();
    y_max_range_ = cpdet_config.pre_process().max_y_range();
    z_min_range_ = cpdet_config.pre_process().min_z_range();
    z_max_range_ = cpdet_config.pre_process().max_z_range();
    voxel_x_size_ = cpdet_config.pre_process().voxel_x_size();
    voxel_y_size_ = cpdet_config.pre_process().voxel_y_size();
    voxel_z_size_ = cpdet_config.pre_process().voxel_z_size();
    x_offset_ = (voxel_x_size_) / 2 + x_min_range_;
    y_offset_ = (voxel_y_size_) / 2 + y_min_range_;
    z_offset_ = (voxel_z_size_) / 2 + z_min_range_;
    float x_range = x_max_range_ - x_min_range_;
    float y_range = y_max_range_ - y_min_range_;
    float z_range = z_max_range_ - z_min_range_;
    grid_x_size_ = static_cast<int>(x_range / voxel_x_size_);
    grid_y_size_ = static_cast<int>(y_range / voxel_y_size_);
    grid_z_size_ = static_cast<int>(z_range / voxel_z_size_);
    map_size_ = grid_x_size_ * grid_y_size_;
    point_dim_ = cpdet_config.pre_process().point_dim();
    enable_rotate_45degree_ = cpdet_config.pre_process().enable_rotate_45degree();
    use_input_norm_ = cpdet_config.pre_process().use_input_norm();
    
    max_point_number_ = 160000;

    use_cnnseg_features_ = cpdet_config.pre_process().use_cnnseg_features();
    cnnseg_feature_dim_ = 0;
    if (use_cnnseg_features_) {
        cnnseg_feature_dim_ = cpdet_config.pre_process().cnnseg_feature_dim();
        height_bin_min_height_ = cpdet_config.pre_process().height_bin_min_height();
        height_bin_max_height_ = cpdet_config.pre_process().height_bin_max_height();
        height_bin_voxel_size_ = cpdet_config.pre_process().height_bin_voxel_size();
        height_bin_dim_ = static_cast<int>((height_bin_max_height_
            - height_bin_min_height_) / height_bin_voxel_size_);
    }

    use_gpu_generate_feature_ = cpdet_config.pre_process().use_gpu_generate_feature();
    max_voxel_num_ = cpdet_config.pre_process().max_voxel_num();
    max_points_in_voxel_ = cpdet_config.pre_process().max_points_in_voxel();
    voxel_feature_dim_ = cpdet_config.pre_process().voxel_feature_dim();
    pillar_feature_dim_ = cpdet_config.pre_process().pillar_feature_dim();
    num_classes_ = cpdet_config.pre_process().num_classes();

    downsample_size_ = cpdet_config.post_process().downsample_size();
    num_tasks_ = cpdet_config.post_process().task_size();
    head_x_size_ = static_cast<int>(grid_x_size_ / downsample_size_);
    head_y_size_ = static_cast<int>(grid_y_size_ / downsample_size_);
    head_map_size_ = head_x_size_ * head_y_size_;
    
    nms_pre_max_size_ = cpdet_config.post_process().nms_pre_max_size();
    nms_post_max_size_ = cpdet_config.post_process().nms_post_max_size();
    score_thresh_ = cpdet_config.post_process().score_thresh();
    nms_overlap_thresh_ = cpdet_config.post_process().nms_overlap_thresh();

    for (size_t i = 0; i < cpdet_config.post_process().det_head_size(); ++i) {
        head_map_[cpdet_config.post_process().det_head(i).head_name()] =
            cpdet_config.post_process().det_head(i).head_shape();
    }
    for (size_t i = 0; i < num_tasks_; i++) {
        num_classes_in_task_.push_back(cpdet_config.post_process().task(i).sub_task_size());
        for (size_t j = 0; j < cpdet_config.post_process().task(i).sub_task_size(); j++) {
            auto type_thres_pair = std::make_pair(cpdet_config.post_process().task(i).sub_task(j).class_name(),
                cpdet_config.post_process().task(i).sub_task(j).score_thresh());
            score_thresh_per_class_[cpdet_config.post_process().task(i).sub_task(j).class_index()] = type_thres_pair;
        }
    }
    for (size_t i = 0; i < num_classes_; i++) {
        score_thresh_map_.push_back(score_thresh_per_class_.at(i).second);
    }
    max_valid_point_size_ = std::max(static_cast<int>(cpdet_config.pre_process().max_voxel_num() * 2),
        max_valid_point_size_);

    top_enlarge_value_ = cpdet_config.post_process().top_enlarge_height();
    bottom_enlarge_value_ = cpdet_config.post_process().bottom_enlarge_height();
    width_enlarge_value_ = cpdet_config.post_process().width_enlarge_value();
    length_enlarge_value_ = cpdet_config.post_process().length_enlarge_value();
    use_cpu_get_objects_ = cpdet_config.post_process().use_cpu_get_objects();
    use_cpu_assign_points_ = cpdet_config.post_process().use_cpu_assign_points();

    remove_semantic_ground_ = cpdet_config.filter_param().remove_semantic_ground();
    remove_raw_ground_ = cpdet_config.filter_param().remove_raw_ground();
    point_unique_ = cpdet_config.filter_param().point_unique();
    filter_by_semantic_type_ = cpdet_config.filter_param().filter_by_semantic_type();
    inter_class_nms_ = cpdet_config.filter_param().inter_class_nms();
    class_nms_iou_thres_ = cpdet_config.filter_param().class_nms_iou_thres();
    nms_strategy_ = cpdet_config.filter_param().nms_strategy();
    min_pts_num_fg_ = cpdet_config.filter_param().min_points_threshold();

    for (int i = 0; i < cpdet_config.filter_param().fore_semantic_filter_size(); i++) {
        int semantic_type = cpdet_config.filter_param().fore_semantic_filter(i).semantic_type();
        auto pair = std::make_pair(cpdet_config.filter_param().fore_semantic_filter(i).roi_out_ratio(), 
            cpdet_config.filter_param().fore_semantic_filter(i).filter_range());
        fore_semantic_filter_map_[semantic_type] = pair;
    }

    enable_downsample_ = cpdet_config.pre_process().enable_downsample_pointcloud();
}

bool CPDetection::InitModel(const cpdet::CPDetModelParam &model_param) {
    // fpn name:blob maps
    std::map<std::string, std::vector<int>> pfn_blob_maps;
    // pfn model
    auto pfn_model_info = model_param.pfn_model_info();
    auto preprocess = model_param.preprocess();
    std::string model_path = GetModelPath(pfn_model_info.name());

    std::string pfn_proto_file = GetModelFile(model_path,
        pfn_model_info.proto_file().file());
    std::vector<std::string> pfn_input_names =
        inference::GetBlobNames(pfn_model_info.inputs());
    std::vector<std::string> pfn_output_names =
        inference::GetBlobNames(pfn_model_info.outputs());
    pfn_inference_ = apollo::cyber::plugin_manager::PluginManager::Instance()->CreateInstance<inference::Inference>(
        "apollo::perception::inference::" + pfn_model_info.infer_plugin());
    pfn_inference_->set_model_info(pfn_proto_file,
        pfn_input_names, pfn_output_names);
    pfn_inference_->set_gpu_id(preprocess.gpu_id());
    pfn_inference_->set_max_batch_size(max_point_number_ * 2);

    inference::AddShape(&pfn_blob_maps, pfn_model_info.inputs());
    inference::AddShape(&pfn_blob_maps, pfn_model_info.outputs());

    pfn_inference_->Init(pfn_blob_maps);
    // init pfn input blob
    if (pfn_input_names.size() == 1) {
        voxels_blob_ = pfn_inference_->get_blob(pfn_input_names.at(0));
        CHECK_NOTNULL(voxels_blob_.get());
    } else {
        AERROR << "Init cpdet_pfn_inference input blob error.";
        return false;
    }
    // init pfn output blobs
    if (pfn_output_names.size() == 1) {
        pfn_pillar_feature_blob_ = pfn_inference_->get_blob(pfn_output_names.at(0));
        CHECK_NOTNULL(pfn_pillar_feature_blob_.get());
    } else {
        AERROR << "Init cpdet_pfn_inference output blob error.";
        return false;
    }
    AINFO << "Init cpdet pfn model success.";

    // backbone name : blob maps
    std::map<std::string, std::vector<int>> backbone_blob_maps;
    // backbone info
    auto backbone_model_info = model_param.backbone_model_info();
    // backbone model file
    std::string backbone_proto_file = GetModelFile(
        model_path, backbone_model_info.proto_file().file());
    std::vector<std::string> backbone_input_names =
        inference::GetBlobNames(backbone_model_info.inputs());
    std::vector<std::string> backbone_output_names =
        inference::GetBlobNames(backbone_model_info.outputs());
    backbone_inference_ = apollo::cyber::plugin_manager::PluginManager::Instance()->CreateInstance<inference::Inference>(
        "apollo::perception::inference::" + backbone_model_info.infer_plugin());
    backbone_inference_->set_model_info(backbone_proto_file,
        backbone_input_names, backbone_output_names);
    backbone_inference_->set_gpu_id(preprocess.gpu_id());
    backbone_inference_->set_max_batch_size(max_point_number_ * 2);

    inference::AddShape(&backbone_blob_maps, backbone_model_info.inputs());
    inference::AddShape(&backbone_blob_maps, backbone_model_info.outputs());

    backbone_inference_->Init(backbone_blob_maps);
    // init backbone input blobs
    if (backbone_input_names.size() == 1) {
        canvas_feature_blob_ = backbone_inference_->get_blob(backbone_input_names.at(0));
        CHECK_NOTNULL(canvas_feature_blob_.get());
    } else {
        AERROR << "Init cpdet_backbone_inference input blob error.";
        return false;
    }
    if (backbone_output_names.size() == 3) {
        output_box_blob_ = backbone_inference_->get_blob(backbone_output_names.at(0));
        output_cls_blob_ = backbone_inference_->get_blob(backbone_output_names.at(1));
        output_dir_blob_ = backbone_inference_->get_blob(backbone_output_names.at(2));
        CHECK_NOTNULL(output_box_blob_.get());
        CHECK_NOTNULL(output_cls_blob_.get());
        CHECK_NOTNULL(output_dir_blob_.get());
    } else {
        AERROR << "Init cpdet_backbone_inference output blobs error.";
        return false;
    }
    AINFO << "Init cpdet backbone model Success.";
    return true;
}

void CPDetection::PointCloudPreprocess() {
    if (!enable_downsample_) {
        return;
    }
    DownSampleOptions down_sample_options;
    ACHECK(down_sample_ != nullptr);
    down_sample_->Process(down_sample_options, cur_cloud_ptr_);
}

// for mmdetection-apollo model
base::ObjectSubType CPDetection::GetObjectSubType(const int label) {
    switch (label) {
        case 0:
            return base::ObjectSubType::SMALLMOT;
        case 1:
            return base::ObjectSubType::PEDESTRIAN;
        case 2:
            return base::ObjectSubType::NONMOT;
        case 3:
            return base::ObjectSubType::TRAFFICCONE;
        default:
            return base::ObjectSubType::UNKNOWN;
    }
}

void CPDetection::GenerateObjects(LidarFrame* frame) {
    // background object
    std::vector<base::ObjectPtr> fore_objects;
    size_t size = res_outputs_.size();
    for (size_t i = 0; i < size; ++i) {
        auto cluster = res_outputs_[i];
        if (cluster.point_ids.size() <= min_pts_num_fg_) {
            continue;
        }
        base::Object object;
        
        object.lidar_supplement.on_use = true;
        object.lidar_supplement.is_in_roi = true;
        object.lidar_supplement.is_background = false;
        object.lidar_supplement.is_clustered = false;
        
        for (auto idx : cluster.point_ids) {
            object.lidar_supplement.point_ids.push_back(static_cast<int>(idx));
        }
        object.lidar_supplement.num_points_in_roi = cluster.point_ids.size();
        object.lidar_supplement.cloud.CopyPointCloud(*original_cloud_, cluster.point_ids);
        object.lidar_supplement.cloud_world.CopyPointCloud(*original_world_cloud_, cluster.point_ids);
        
        // model original output
        object.lidar_supplement.detections.resize(7);
        object.lidar_supplement.detections[0] = cluster.x;
        object.lidar_supplement.detections[1] = cluster.y;
        object.lidar_supplement.detections[2] = cluster.z;
        object.lidar_supplement.detections[3] = cluster.l;
        object.lidar_supplement.detections[4] = cluster.w;
        object.lidar_supplement.detections[5] = cluster.h;
        object.lidar_supplement.detections[6] = cluster.yaw;

        // center and size
        object.center(0) = cluster.x;
        object.center(1) = cluster.y;
        object.center(2) = cluster.z;
        object.size(0) = cluster.l;
        object.size(1) = cluster.w;
        object.size(2) = cluster.h;

        // direction
        object.theta = cluster.yaw;
        object.direction[0] = cosf(cluster.yaw);
        object.direction[1] = sinf(cluster.yaw);
        object.direction[2] = 0;
        object.lidar_supplement.is_orientation_ready = true;

        // confidence
        object.confidence = cluster.confidence;
        object.id = i;

        // classification
        object.lidar_supplement.raw_probs.push_back(std::vector<float>(
            static_cast<int>(base::ObjectType::MAX_OBJECT_TYPE), 0.f));
        object.lidar_supplement.raw_classification_methods.push_back(Name());
        object.sub_type = GetObjectSubType(cluster.model_type_index);
        object.type = base::kSubType2TypeMap.at(object.sub_type);
        // AERROR << "[Type:] " << static_cast<int>(object.type);
        if (object.sub_type == base::ObjectSubType::TRAFFICCONE) {
            object.type = base::ObjectType::UNKNOWN;
        }
        object.lidar_supplement.raw_probs.back()[static_cast<int>(object.type)] = 1.0f;
        // copy to type
        object.type_probs.assign(object.lidar_supplement.raw_probs.back().begin(),
                object.lidar_supplement.raw_probs.back().end());        
        // copy to background objects
        std::shared_ptr<base::Object> obj(new base::Object);
        *obj = object;
        fore_objects.push_back(std::move(obj));
    }
    // object builder for fore objects
    frame->segmented_objects.insert(frame->segmented_objects.end(), fore_objects.begin(), fore_objects.end());
}

void CPDetection::SetPointsInROI(std::vector<base::ObjectPtr> *objects) {
    CloudMask roi_mask_;
    roi_mask_.Set(original_cloud_->size(), 0);
    roi_mask_.AddIndices(lidar_frame_ref_->roi_indices, 1);
    for (uint32_t i = 0; i < objects->size(); ++i) {
        auto &object = objects->at(i);
        object->lidar_supplement.num_points_in_roi =
            roi_mask_.ValidIndicesCount(object->lidar_supplement.point_ids);
    }
}

void CPDetection::RemoveForePoints(std::vector<base::ObjectPtr> *objects) {
    CloudMask mask;
    mask.Set(original_cloud_->size(), 0);
    mask.AddIndicesOfIndices(lidar_frame_ref_->roi_indices,
        lidar_frame_ref_->non_ground_indices, 1);
    for (uint32_t i = 0; i < objects->size(); ++i) {
        auto &object = objects->at(i);
        base::PointIndices ind;
        ind.indices = object->lidar_supplement.point_ids;
        mask.RemoveIndices(ind);
    }
    mask.GetValidIndices(&lidar_frame_ref_->secondary_indices);
}

void CPDetection::MapPointsToGridCpu(int* point2grid_data) {
    auto Pc2Pixel = [&](float pc_axis, float voxel_size, float start_range) {
        float fpixel = (pc_axis - start_range) / voxel_size;
        return fpixel < 0 ? -1 : static_cast<int>(fpixel);
    };
    
    int pos_x = -1;
    int pos_y = -1;
    for (int i = 0; i < model_cloud_size_; ++i) {
        auto pt = original_cloud_->at(i);
        float px = pt.x;
        float py = pt.y;
        float pz = pt.z;
        if (enable_rotate_45degree_) {
            px = 0.707107 * pt.x - 0.707107 * pt.y;
            py = 0.707107 * pt.x + 0.707107 * pt.y;
        }
        pos_x = Pc2Pixel(px, voxel_x_size_, x_min_range_);
        pos_y = Pc2Pixel(py, voxel_y_size_, y_min_range_);
        if (pos_y < 0 || pos_y >= grid_y_size_ || pos_x < 0 || pos_x >= grid_x_size_) {
            point2grid_data[i] = -1;
            continue;
        }
        point2grid_data[i] = pos_y * grid_x_size_ + pos_x;
    }
}

void CPDetection::GeneratePfnFeatureCPU() {
    // DO NOT remove this line!!!
    // Otherwise, the gpu_data will not be updated for the later frames.
    // It marks the head at cpu for blob.
    voxels_blob_->mutable_cpu_data();

    float* voxels_cluster_data = canvas_feature_blob_->mutable_cpu_data();
    int* grid2pointnum_data = grid2pointnum_blob_->mutable_cpu_data();
    memset(voxels_cluster_data, 0, map_size_ * 3 * sizeof(float));
    memset(grid2pointnum_data, 0, map_size_ * sizeof(int));

    point2grid_blob_->Reshape(std::vector<int>{model_cloud_size_});
    voxels_blob_->Reshape(std::vector<int>{model_cloud_size_, 1, voxel_feature_dim_, 1});
    int* point2grid_data = point2grid_blob_->mutable_cpu_data();
    float* voxels_data = voxels_blob_->mutable_cpu_data();
    memset(point2grid_data, 0, model_cloud_size_ * sizeof(int));
    memset(voxels_data, 0.f, (model_cloud_size_) * voxel_feature_dim_ * sizeof(float));

    MapPointsToGridCpu(point2grid_data);

    if (use_cnnseg_features_) {
        float* out_data = canvas_feature_blob_->mutable_cpu_data();
        max_height_data_ = out_data + feature_offset_["max_height"];
        mean_height_data_ = out_data + feature_offset_["mean_height"];
        top_intensity_data_ = out_data + feature_offset_["top_intensity"];
        mean_intensity_data_ = out_data + feature_offset_["mean_intensity"];
        count_data_ = out_data + feature_offset_["count"];
        nonempty_data_ = out_data + feature_offset_["nonempty"];
        height_bin_data_ = out_data + feature_offset_["height_bin"];
        
        memset(mean_height_data_, 0, map_size_ * sizeof(float));
        memset(max_height_data_, -5.0, map_size_ * sizeof(float));
        memset(top_intensity_data_, 0, map_size_ * sizeof(float));
        memset(mean_intensity_data_, 0, map_size_ * sizeof(float));
        memset(count_data_, 0, map_size_ * sizeof(float));
        memset(nonempty_data_, 0, map_size_ * sizeof(float));
        memset(height_bin_data_, 0, map_size_ * height_bin_dim_ * sizeof(float));
    }

    // compute features
    for (int i = 0; i < model_cloud_size_; ++i) {
        int idx = point2grid_data[i];
        if (idx == -1) {
            continue;
        }
        auto pt = original_cloud_->at(i);
        float px = pt.x;
        float py = pt.y;
        float pz = pt.z;
        if (enable_rotate_45degree_) {
            px = 0.707107 * pt.x - 0.707107 * pt.y;
            py = 0.707107 * pt.x + 0.707107 * pt.y;
        }
        // calculate voxel cluster 
        voxels_cluster_data[map_size_ * 0 + idx] += px;
        voxels_cluster_data[map_size_ * 1 + idx] += py;
        voxels_cluster_data[map_size_ * 2 + idx] += pz;
        grid2pointnum_data[idx] += 1;
    }

    for (int i = 0; i < model_cloud_size_; ++i) {
        int grid_idx = point2grid_data[i];
        if (grid_idx == -1) {
            continue;
        }
        auto pt = original_cloud_->at(i);
        float px = pt.x;
        float py = pt.y;
        float pz = pt.z;
        if (enable_rotate_45degree_) {
            px = 0.707107 * pt.x - 0.707107 * pt.y;
            py = 0.707107 * pt.x + 0.707107 * pt.y;
        }
        int coord_y = grid_idx / grid_x_size_;
        int coord_x = grid_idx % grid_x_size_;
        int voxel_data_idx = i * voxel_feature_dim_;
        float point_num = std::max(1.f, static_cast<float>(grid2pointnum_data[grid_idx]));

        voxels_data[voxel_data_idx + 4] = px - voxels_cluster_data[map_size_ * 0 + grid_idx] / point_num;
        voxels_data[voxel_data_idx + 5] = py - voxels_cluster_data[map_size_ * 1 + grid_idx] / point_num;
        voxels_data[voxel_data_idx + 6] = pz - voxels_cluster_data[map_size_ * 2 + grid_idx] / point_num;
        voxels_data[voxel_data_idx + 7] = px - (static_cast<float>(coord_x) * voxel_x_size_ + x_offset_);
        voxels_data[voxel_data_idx + 8] = py - (static_cast<float>(coord_y) * voxel_y_size_ + y_offset_);
        if (use_input_norm_) {
            voxels_data[voxel_data_idx + 0] = px / x_max_range_;
            voxels_data[voxel_data_idx + 1] = py / y_max_range_;
            voxels_data[voxel_data_idx + 2] = pz / z_max_range_;
            voxels_data[voxel_data_idx + 3] = pt.intensity / 255.0;
        } else {
            voxels_data[voxel_data_idx + 0] = pt.x;
            voxels_data[voxel_data_idx + 1] = pt.y;
            voxels_data[voxel_data_idx + 2] = pt.z;
            voxels_data[voxel_data_idx + 3] = pt.intensity;
        }
        if (use_cnnseg_features_) {
            if (max_height_data_[grid_idx] < pz) {
                max_height_data_[grid_idx] = pz;
                top_intensity_data_[grid_idx] = pt.intensity / 255.0;
            }
            mean_height_data_[grid_idx] += static_cast<float>(pz);
            mean_intensity_data_[grid_idx] += static_cast<float>(
                pt.intensity / 255.0);
            count_data_[grid_idx] += 1.f;
            int height_bin_index = static_cast<int>(
                (pz - height_bin_min_height_) / height_bin_voxel_size_);
            height_bin_index = height_bin_index < 0 ? 0 : height_bin_index;
            height_bin_index = height_bin_index >= height_bin_dim_ ?
                (height_bin_dim_ - 1) : height_bin_index;
            height_bin_data_[height_bin_index * map_size_ + grid_idx] = 1.f;
        }
    }

    if (use_cnnseg_features_) {
        for (int i = 0; i < map_size_; ++i) {
            if (count_data_[i] <= std::numeric_limits<float>::epsilon()) {
                max_height_data_[i] = 0.f;
            } else {
                mean_height_data_[i] /= count_data_[i];
                mean_intensity_data_[i] /= count_data_[i];
                nonempty_data_[i] = 1.f;
            }
            count_data_[i] = static_cast<int>(log(static_cast<float>(1 + count_data_[i])));
        }
    }
    // output
    pfn_pillar_feature_blob_->Reshape({model_cloud_size_, pillar_feature_dim_});
}

void CPDetection::GenerateBackboneFeatureCPU(const base::Blob<float>* pillar_feature_blob) {
    // DO NOT remove this line!!!
    // Otherwise, the gpu_data will not be updated for the later frames.
    // It marks the head at cpu for blob.
    canvas_feature_blob_->mutable_cpu_data();

    const float* pillar_feat_data = pillar_feature_blob->cpu_data();
    const int* point2grid_data = point2grid_blob_->cpu_data();
    const int* grid2pointnum_data = grid2pointnum_blob_->cpu_data();
    
    // input
    int shape = pillar_feature_dim_ + cnnseg_feature_dim_;
    canvas_feature_blob_->Reshape({1, shape, grid_x_size_, grid_y_size_});
    float* canvas_feat_data = canvas_feature_blob_->mutable_cpu_data() + feature_offset_["canvas_feature"];
    memset(canvas_feat_data, 0.f, map_size_ * pillar_feature_dim_ * sizeof(float));
   
    // max-pooling
    for (int i = 0; i < model_cloud_size_; ++i) {
        int grid_idx = point2grid_data[i];
        if (grid_idx == -1) {
            continue;
        }
        int y = grid_idx / grid_x_size_;
        int x = grid_idx % grid_x_size_;
        for (int c_idx = 0; c_idx < pillar_feature_dim_; c_idx++) {
            int canvas_idx = c_idx * map_size_ + y * grid_x_size_ + x;
            int pillar_idx = i * pillar_feature_dim_ + c_idx;
            canvas_feat_data[canvas_idx] = std::max(
                canvas_feat_data[canvas_idx], pillar_feat_data[pillar_idx]);
        }
    }
}

float CPDetection::CalculateUnionArea(const float *corners1, const float *corners2) {
    auto trangle_area_cpu = [&](float* a, float* b, float* c) {
        return ((a[0] - c[0]) * (b[1] - c[1]) - (a[1] - c[1]) * (b[0] - c[0])) / 2.f;
    };

    auto sort_vertex_in_convex_polygon_cpu = [&](float* int_pts, int num_of_inter) {
        if (num_of_inter == 0) {
            return;
        }
        float center_x = 0.f;
        float center_y = 0.f;
        for (int i = 0; i < num_of_inter; i++) {
            center_x += int_pts[2 * i];
            center_y += int_pts[2 * i + 1];
        }
        center_x /= num_of_inter;
        center_y /= num_of_inter;
        float v0;
        float v1;
        float vs[16];
        for (int i = 0; i < num_of_inter; i++) {
            v0 = int_pts[2 * i] - center_x;
            v1 = int_pts[2 * i + 1] - center_y;
            float d = sqrt(v0 * v0 + v1 * v1);
            v0 = v0 / d;
            v1 = v1 / d;
            if (v1 < 0) {
                v0 = -2 - v0;
            }
            vs[i] = v0;
        }
        int j = 0;
        float temp = 0.f;
        for (int i = 0; i < num_of_inter; i++) {
            if (vs[i - 1] > vs[i]) {
                temp = vs[i];
                float tx = int_pts[2 * i];
                float ty = int_pts[2 * i + 1];
                j = i;
                while (j > 0 && vs[j - 1] > temp) {
                    vs[j] = vs[j - 1];
                    int_pts[j * 2] = int_pts[j * 2 - 2];
                    int_pts[j * 2 + 1] = int_pts[j * 2 - 1];
                    j -= 1;
                }
                vs[j] = temp;
                int_pts[j * 2] = tx;
                int_pts[j * 2 + 1] = ty;
            }
        }
    };

    auto line_segment_intersection_cpu = [&](const float* pts1, const float* pts2, int i, int j, float* temp_pts) {
        float A0 = pts1[2 * i];
        float A1 = pts1[2 * i + 1];

        float B0 = pts1[2 * ((i + 1) % 4)];
        float B1 = pts1[2 * ((i + 1) % 4) + 1];

        float C0 = pts2[2 * j];
        float C1 = pts2[2 * j + 1];

        float D0 = pts2[2 * ((j + 1) % 4)];
        float D1 = pts2[2 * ((j + 1) % 4) + 1];
        float BA0 = B0 - A0;
        float BA1 = B1 - A1;
        float DA0 = D0 - A0;
        float CA0 = C0 - A0;
        float DA1 = D1 - A1;
        float CA1 = C1 - A1;
        bool acd = DA1 * CA0 > CA1 * DA0;
        bool bcd = (D1 - B1) * (C0 - B0) > (C1 - B1) * (D0 - B0);
        if (acd != bcd) {
            bool abc = CA1 * BA0 > BA1 * CA0;
            bool abd = DA1 * BA0 > BA1 * DA0;
            if (abc != abd) {
                float DC0 = D0 - C0;
                float DC1 = D1 - C1;
                float ABBA = A0 * B1 - B0 * A1;
                float CDDC = C0 * D1 - D0 * C1;
                float DH = BA1 * DC0 - BA0 * DC1;
                float Dx = ABBA * DC0 - BA0 * CDDC;
                float Dy = ABBA * DC1 - BA1 * CDDC;
                temp_pts[0] = Dx / DH;
                temp_pts[1] = Dy / DH;
                return true;
            }
        }
        return false;
    };

    auto point_in_quadrilateral_cpu = [&](float pt_x, float pt_y, const float* corners) {
        float ab0 = corners[2] - corners[0];
        float ab1 = corners[3] - corners[1];
        float ad0 = corners[6] - corners[0];
        float ad1 = corners[7] - corners[1];
        float ap0 = pt_x - corners[0];
        float ap1 = pt_y - corners[1];
        float abab = ab0 * ab0 + ab1 * ab1;
        float abap = ab0 * ap0 + ab1 * ap1;
        float adad = ad0 * ad0 + ad1 * ad1;
        float adap = ad0 * ap0 + ad1 * ap1;
        return abab >= abap && abap >= 0 && adad >= adap && adap >= 0;
    };

    float intersection_corners[16];
    // quadrilateral_intersection
    int num_of_inter = 0;
    for (int i = 0; i < 4; i++) {
        if (point_in_quadrilateral_cpu(corners1[2 * i], corners1[2 * i + 1], corners2)) {
            intersection_corners[num_of_inter * 2] = corners1[2 * i];
            intersection_corners[num_of_inter * 2 + 1] = corners1[2 * i + 1];
            num_of_inter += 1;
        }
        if (point_in_quadrilateral_cpu(corners2[2 * i], corners2[2 * i + 1], corners1)) {
            intersection_corners[num_of_inter * 2] = corners2[2 * i];
            intersection_corners[num_of_inter * 2 + 1] = corners2[2 * i + 1];
            num_of_inter += 1;
        }
    }
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            if (line_segment_intersection_cpu(corners1, corners2, i, j, intersection_corners + num_of_inter * 2)) {
                ++num_of_inter;
            }
        }
    }
    sort_vertex_in_convex_polygon_cpu(intersection_corners, num_of_inter);
    // area
    float area_val = 0.f;
    for (int i = 0; i < num_of_inter - 2; i++) {
        area_val += abs(trangle_area_cpu(intersection_corners,
            intersection_corners + 2 * (i + 1),
            intersection_corners + 2 * (i + 2)));
    }
    return area_val;
}

std::vector<ResultsOutput> CPDetection::ApplyNMSCPU(std::vector<ResultsOutput>& boxes) {
    auto get_res_corners = [&](ResultsOutput& o) {
        std::vector<float> box_corner;
        box_corner.resize(8);
        float x = o.x;
        float y = o.y;
        float l = o.l;
        float w = o.w;
        float r = o.yaw;
        // if (length_enlarge_value_ > 0) {
        //     l = l + length_enlarge_value_;
        // }
        // if (width_enlarge_value_ > 0) {
        //     w = w + width_enlarge_value_;
        // }
        float cos_r = cos(r);
        float sin_r = sin(r);
        float hl = l * 0.5;
        float hw = w * 0.5;
        float x1 = (-hl) * cos_r - (-hw) * sin_r + x;
        float y1 = (-hl) * sin_r + (-hw) * cos_r + y;
        float x2 = ( hl) * cos_r - (-hw) * sin_r + x;
        float y2 = ( hl) * sin_r + (-hw) * cos_r + y;
        float x3 = ( hl) * cos_r - ( hw) * sin_r + x;
        float y3 = ( hl) * sin_r + ( hw) * cos_r + y;
        float x4 = (-hl) * cos_r - ( hw) * sin_r + x;
        float y4 = (-hl) * sin_r + ( hw) * cos_r + y;
        box_corner[0] = x1;
        box_corner[1] = y1;
        box_corner[2] = x2;
        box_corner[3] = y2;
        box_corner[4] = x3;
        box_corner[5] = y3;
        box_corner[6] = x4;
        box_corner[7] = y4;
        return box_corner;
    };

/*
    auto trangle_area_cpu = [&](float* a, float* b, float* c) {
        return ((a[0] - c[0]) * (b[1] - c[1]) - (a[1] - c[1]) * (b[0] - c[0])) / 2.f;
    };

    auto sort_vertex_in_convex_polygon_cpu = [&](float* int_pts, int num_of_inter) {
        if (num_of_inter == 0) {
            return;
        }
        float center_x = 0.f;
        float center_y = 0.f;
        for (int i = 0; i < num_of_inter; i++) {
            center_x += int_pts[2 * i];
            center_y += int_pts[2 * i + 1];
        }
        center_x /= num_of_inter;
        center_y /= num_of_inter;
        float v0;
        float v1;
        float vs[16];
        for (int i = 0; i < num_of_inter; i++) {
            v0 = int_pts[2 * i] - center_x;
            v1 = int_pts[2 * i + 1] - center_y;
            float d = sqrt(v0 * v0 + v1 * v1);
            v0 = v0 / d;
            v1 = v1 / d;
            if (v1 < 0) {
                v0 = -2 - v0;
            }
            vs[i] = v0;
        }
        int j = 0;
        float temp = 0.f;
        for (int i = 0; i < num_of_inter; i++) {
            if (vs[i - 1] > vs[i]) {
                temp = vs[i];
                float tx = int_pts[2 * i];
                float ty = int_pts[2 * i + 1];
                j = i;
                while (j > 0 && vs[j - 1] > temp) {
                    vs[j] = vs[j - 1];
                    int_pts[j * 2] = int_pts[j * 2 - 2];
                    int_pts[j * 2 + 1] = int_pts[j * 2 - 1];
                    j -= 1;
                }
                vs[j] = temp;
                int_pts[j * 2] = tx;
                int_pts[j * 2 + 1] = ty;
            }
        }
    };

    auto line_segment_intersection_cpu = [&](const float* pts1, const float* pts2, int i, int j, float* temp_pts) {
        float A0 = pts1[2 * i];
        float A1 = pts1[2 * i + 1];

        float B0 = pts1[2 * ((i + 1) % 4)];
        float B1 = pts1[2 * ((i + 1) % 4) + 1];

        float C0 = pts2[2 * j];
        float C1 = pts2[2 * j + 1];

        float D0 = pts2[2 * ((j + 1) % 4)];
        float D1 = pts2[2 * ((j + 1) % 4) + 1];
        float BA0 = B0 - A0;
        float BA1 = B1 - A1;
        float DA0 = D0 - A0;
        float CA0 = C0 - A0;
        float DA1 = D1 - A1;
        float CA1 = C1 - A1;
        bool acd = DA1 * CA0 > CA1 * DA0;
        bool bcd = (D1 - B1) * (C0 - B0) > (C1 - B1) * (D0 - B0);
        if (acd != bcd) {
            bool abc = CA1 * BA0 > BA1 * CA0;
            bool abd = DA1 * BA0 > BA1 * DA0;
            if (abc != abd) {
                float DC0 = D0 - C0;
                float DC1 = D1 - C1;
                float ABBA = A0 * B1 - B0 * A1;
                float CDDC = C0 * D1 - D0 * C1;
                float DH = BA1 * DC0 - BA0 * DC1;
                float Dx = ABBA * DC0 - BA0 * CDDC;
                float Dy = ABBA * DC1 - BA1 * CDDC;
                temp_pts[0] = Dx / DH;
                temp_pts[1] = Dy / DH;
                return true;
            }
        }
        return false;
    };

    auto point_in_quadrilateral_cpu = [&](float pt_x, float pt_y, const float* corners) {
        float ab0 = corners[2] - corners[0];
        float ab1 = corners[3] - corners[1];
        float ad0 = corners[6] - corners[0];
        float ad1 = corners[7] - corners[1];
        float ap0 = pt_x - corners[0];
        float ap1 = pt_y - corners[1];
        float abab = ab0 * ab0 + ab1 * ab1;
        float abap = ab0 * ap0 + ab1 * ap1;
        float adad = ad0 * ad0 + ad1 * ad1;
        float adap = ad0 * ap0 + ad1 * ap1;
        return abab >= abap && abap >= 0 && adad >= adap && adap >= 0;
    };

    auto rotate_inter = [&](const float *corners1, const float *corners2) {
        float intersection_corners[16];
        // quadrilateral_intersection
        int num_of_inter = 0;
        for (int i = 0; i < 4; i++) {
            if (point_in_quadrilateral_cpu(corners1[2 * i], corners1[2 * i + 1], corners2)) {
                intersection_corners[num_of_inter * 2] = corners1[2 * i];
                intersection_corners[num_of_inter * 2 + 1] = corners1[2 * i + 1];
                num_of_inter += 1;
            }
            if (point_in_quadrilateral_cpu(corners2[2 * i], corners2[2 * i + 1], corners1)) {
                intersection_corners[num_of_inter * 2] = corners2[2 * i];
                intersection_corners[num_of_inter * 2 + 1] = corners2[2 * i + 1];
                num_of_inter += 1;
            }
        }
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                if (line_segment_intersection_cpu(corners1, corners2, i, j, intersection_corners + num_of_inter * 2)) {
                    ++num_of_inter;
                }
            }
        }
        sort_vertex_in_convex_polygon_cpu(intersection_corners, num_of_inter);
        // area
        float area_val = 0.f;
        for (int i = 0; i < num_of_inter - 2; i++) {
            area_val += abs(trangle_area_cpu(intersection_corners,
                intersection_corners + 2 * (i + 1),
                intersection_corners + 2 * (i + 2)));
        }
        return area_val;
    };
*/

    auto BBoxIoU = [&](ResultsOutput& o1, ResultsOutput& o2) {
        std::vector<float> box_corner1 = get_res_corners(o1);
        std::vector<float> box_corner2 = get_res_corners(o2);
        float area_inter = CalculateUnionArea(box_corner1.data(), box_corner2.data());
        float overlap = area_inter / std::max(o1.l * o1.w + o2.l * o2.w - area_inter, 0.001f);
        return overlap;
        // return 0.0;
    };

    auto NmsCompare = [&](const ResultsOutput& a, const ResultsOutput& b) {
        if (a.confidence == b.confidence) {
            return a.model_type_index < b.model_type_index;
        }
        return a.confidence > b.confidence;
    };

    std::vector<ResultsOutput> res;
    std::sort(boxes.begin(), boxes.end(), NmsCompare);
    while (boxes.size() > 0) {
        res.push_back(boxes[0]);
        int index = 1;
        while (index < boxes.size()) {
            float iou = BBoxIoU(boxes[0], boxes[index]);
            if (iou > nms_overlap_thresh_) {
                boxes.erase(boxes.begin() + index);
            } else {
                index++;
            }
        }
        boxes.erase(boxes.begin());
    }
    return res;
}

void CPDetection::GetObjectsCPU() {
    // DO NOT remove this line!!!
    // Otherwise, the gpu_data will not be updated for the later frames.
    // It marks the head at cpu for blob.
    int box_range = head_map_["reg"] + head_map_["hei"] + head_map_["dim"];
    std::vector<int> cls_range {0};
    Timer timer;

    float* all_res_box_data = all_res_box_blob_->mutable_cpu_data();
    float* all_res_conf_data = all_res_conf_blob_->mutable_cpu_data();
    int* all_res_cls_data = all_res_cls_blob_->mutable_cpu_data();
    memset(all_res_box_data, 0.f, sizeof(float) * max_candidate_num_ * kBoxBlockSize);
    memset(all_res_conf_data, 0.f, sizeof(float) * max_candidate_num_);
    memset(all_res_cls_data, -1, sizeof(int) * max_candidate_num_);

    res_outputs_.clear();
    std::vector<ResultsOutput> total_results;
    total_results.clear();
    for (int i = 0; i < num_tasks_; ++i) {
        std::vector<ResultsOutput> task_results;
        task_results.clear();

        const float* reg = output_box_blob_->cpu_data() + output_box_blob_->offset(0, box_range * i);
        const float* hei = output_box_blob_->cpu_data() + output_box_blob_->offset(0, box_range * i + head_map_["reg"]);
        const float* dim = output_box_blob_->cpu_data() + output_box_blob_->offset(0, box_range * i + head_map_["reg"] + head_map_["hei"]);
        const float* cls = output_cls_blob_->cpu_data() + output_cls_blob_->offset(0, cls_range[i]);
        const float* rot = output_dir_blob_->cpu_data() + output_dir_blob_->offset(0, head_map_["rot"] * i);
        cls_range.push_back(cls_range[i] + num_classes_in_task_[i]);
        // float* res_box_data = res_box_blob_->mutable_cpu_data();
        // float* res_conf_data = res_conf_blob_->mutable_cpu_data();
        // int* res_cls_data = res_cls_blob_->mutable_cpu_data();
        // memset(res_box_data, 0.f, sizeof(float) * nms_pre_max_size_ * kBoxBlockSize);
        // memset(res_conf_data, 0.f, sizeof(float) * nms_pre_max_size_);
        // memset(res_cls_data, -1, sizeof(int) * nms_pre_max_size_);

        int box_num = 0;
        for (int idx = 0; idx < head_map_size_; idx++) {
            float max_score = cls[idx];
            int label = cls_range[i];
            for (int index = 1; index < num_classes_in_task_[i]; ++index) {
                float cur_score = cls[idx + index * head_map_size_];
                if (cur_score > max_score) {
                    max_score = cur_score;
                    label = index + cls_range[i];
                }
            }
            int coor_x = idx % head_x_size_;
            int coor_y = idx / head_x_size_;
            float conf = 1.0 / (1.0 + exp(-max_score));
            if (conf > score_thresh_map_[label]) {
                box_num++;
                if (box_num >= nms_pre_max_size_) {
                    continue;
                }
                float x = (reg[idx + 0 * head_map_size_] + coor_x) * downsample_size_ * voxel_x_size_ + x_min_range_;
                float y = (reg[idx + 1 * head_map_size_] + coor_y) * downsample_size_ * voxel_y_size_ + y_min_range_;   
                float z = hei[idx];
                float l = expf(dim[idx + 0 * head_map_size_]);
                float w = expf(dim[idx + 1 * head_map_size_]);
                float h = expf(dim[idx + 2 * head_map_size_]);
                float theta = atan2f(rot[idx], rot[idx + head_map_size_]);
                int type = label;
                // res_box_data[box_num * kBoxBlockSize + 0] = x;
                // res_box_data[box_num * kBoxBlockSize + 1] = y;
                // res_box_data[box_num * kBoxBlockSize + 2] = z;
                // res_box_data[box_num * kBoxBlockSize + 3] = l;
                // res_box_data[box_num * kBoxBlockSize + 4] = w;
                // res_box_data[box_num * kBoxBlockSize + 5] = h;
                // res_box_data[box_num * kBoxBlockSize + 6] = theta;
                // res_conf_data[box_num * kBoxBlockSize + 0] = conf;
                // res_cls_data[box_num * kBoxBlockSize + 0] = type;
                ResultsOutput output;
                output.x = x;
                output.y = y;
                output.z = z;
                output.l = l;
                output.w = w;
                output.h = h;
                output.yaw = theta;
                output.model_type_index = type;
                output.confidence = conf;
                output.point_ids.clear();
                task_results.push_back(output);
            }
        }
        // after 
        // should apply task-NMS
        std::vector<ResultsOutput> nms_results = ApplyNMSCPU(task_results);
        for (int j = 0; j < nms_results.size(); j++) {
            // all_res_box_data[(all_res_num + idx) * kBoxBlockSize + 0] = res_box_data[idx * kBoxBlockSize + 0];
            // all_res_box_data[(all_res_num + idx) * kBoxBlockSize + 1] = res_box_data[idx * kBoxBlockSize + 1];
            // all_res_box_data[(all_res_num + idx) * kBoxBlockSize + 2] = res_box_data[idx * kBoxBlockSize + 2];
            // all_res_box_data[(all_res_num + idx) * kBoxBlockSize + 3] = res_box_data[idx * kBoxBlockSize + 3];
            // all_res_box_data[(all_res_num + idx) * kBoxBlockSize + 4] = res_box_data[idx * kBoxBlockSize + 4];
            // all_res_box_data[(all_res_num + idx) * kBoxBlockSize + 5] = res_box_data[idx * kBoxBlockSize + 5];
            // all_res_box_data[(all_res_num + idx) * kBoxBlockSize + 6] = res_box_data[idx * kBoxBlockSize + 6];
            // all_res_conf_data[all_res_num + idx] = res_conf_data[idx];
            // all_res_cls_data[all_res_num + idx] = res_cls_data[idx];
            ResultsOutput output;
            output.x = nms_results[j].x;
            output.y = nms_results[j].y;
            output.z = nms_results[j].z;
            output.l = nms_results[j].l;
            output.w = nms_results[j].w;
            output.h = nms_results[j].h;
            output.yaw = nms_results[j].yaw;
            output.model_type_index = nms_results[j].model_type_index;
            output.confidence = nms_results[j].confidence;
            output.point_ids.clear();
            total_results.push_back(output);
        }
    }

    // get total outputs
    const int total_box = total_results.size();
    res_outputs_.resize(total_box);
    for (int i = 0; i < total_box; i++) {
        // auto cand_bbox = all_res_box_blob_->cpu_data() + all_res_box_blob_->offset(i);
        // auto cand_conf = all_res_conf_blob_->cpu_data() + i;
        // auto cand_cls = all_res_cls_blob_->cpu_data() + i;
        // float x = cand_bbox[0];
        // float y = cand_bbox[1];
        // float r = cand_bbox[6];
        float x = total_results[i].x;
        float y = total_results[i].y;
        float r = total_results[i].yaw;
        if (enable_rotate_45degree_) {
            x =  0.707107 * total_results[i].x + 0.707107 * total_results[i].y;
            y = -0.707107 * total_results[i].x + 0.707107 * total_results[i].y;
            r -= M_PI / 4;
        }
        r += (r <= -M_PI ? M_PI : 0.0);
        r -= (r >= M_PI ? M_PI : 0.0);
        res_outputs_.at(i).clear();
        res_outputs_.at(i).x = x;
        res_outputs_.at(i).y = y;
        res_outputs_.at(i).z = total_results[i].z - total_results[i].h / 2;
        res_outputs_.at(i).l = total_results[i].l;
        res_outputs_.at(i).w = total_results[i].w;
        res_outputs_.at(i).h = total_results[i].h;
        res_outputs_.at(i).yaw = r;
        res_outputs_.at(i).confidence = total_results[i].confidence;
        res_outputs_.at(i).model_type_index = total_results[i].model_type_index;
    }
    total_results.clear();
    double get_time = timer.toc(true);
    ADEBUG << "[GetObjectsCPU] time: " << get_time;
}

void CPDetection::AssignPointsCPU() {
    const int num_objects = res_outputs_.size();
    const float quantize = voxel_x_size_;
    Timer timer;
    std::vector<float> box_corner(num_objects * 8);
    std::vector<float> box_rectangular(num_objects * 4);
    for (int i = 0; i < num_objects; ++i) {
        float xx = res_outputs_[i].x;
        float yy = res_outputs_[i].y;
        float l = res_outputs_[i].l;
        float w = res_outputs_[i].w;
        float a = res_outputs_[i].yaw;
        // should revert to original format
        float x = xx;
        float y = yy;
        if (enable_rotate_45degree_) {
            x = 0.707107 * xx - 0.707107 * yy;
            y = 0.707107 * xx + 0.707107 * yy;
            a += M_PI / 4;
        } 
        if (quantize > 0) {
            w = ceil(w / quantize) * quantize;
            l = ceil(l / quantize) * quantize;
        }
        if (width_enlarge_value_ > 0) {
            w = w + width_enlarge_value_;
        }
        if (length_enlarge_value_ > 0) {
            l = l + length_enlarge_value_;
        }
        float cos_a = cos(a);
        float sin_a = sin(a);
        float hl = l * 0.5;
        float hw = w * 0.5;

        float x1 = (-hl) * cos_a - (-hw) * sin_a + x;
        float y1 = (-hl) * sin_a + (-hw) * cos_a + y;
        float x2 = ( hl) * cos_a - (-hw) * sin_a + x;
        float y2 = ( hl) * sin_a + (-hw) * cos_a + y;
        float x3 = ( hl) * cos_a - ( hw) * sin_a + x;
        float y3 = ( hl) * sin_a + ( hw) * cos_a + y;
        float x4 = (-hl) * cos_a - ( hw) * sin_a + x;
        float y4 = (-hl) * sin_a + ( hw) * cos_a + y;
        box_corner[i * 8 + 0] = x1;
        box_corner[i * 8 + 1] = y1;
        box_corner[i * 8 + 2] = x2;
        box_corner[i * 8 + 3] = y2;
        box_corner[i * 8 + 4] = x3;
        box_corner[i * 8 + 5] = y3;
        box_corner[i * 8 + 6] = x4;
        box_corner[i * 8 + 7] = y4;
        box_rectangular[i * 4 + 0] = std::min(std::min(std::min(x1, x2), x3), x4);
        box_rectangular[i * 4 + 1] = std::min(std::min(std::min(y1, y2), y3), y4);
        box_rectangular[i * 4 + 2] = std::max(std::max(std::max(x1, x2), x3), x4);
        box_rectangular[i * 4 + 3] = std::max(std::max(std::max(y1, y2), y3), y4);
    }
    double nms_time = timer.toc(true);

    for (int point_idx = 0; point_idx < original_cloud_->size(); ++point_idx) {
        bool semantic_flag = (static_cast<PointSemanticLabel>(
            original_cloud_->points_semantic_label(point_idx) & 15) ==
                PointSemanticLabel::GROUND);
        bool raw_flag = (original_cloud_->points_label(point_idx) ==
            static_cast<uint8_t>(LidarPointLabel::GROUND));
        if ((remove_semantic_ground_ && semantic_flag) ||
            (remove_raw_ground_ && raw_flag)) {
            continue;
        }
        const auto &point = original_cloud_->at(point_idx);
        float px = point.x;
        float py = point.y;
        float pz = point.z;
        if (enable_rotate_45degree_) {
            px = 0.707107 * point.x - 0.707107 * point.y;
            py = 0.707107 * point.x + 0.707107 * point.y;
        }
        for (int i = 0; i < num_objects; i++) {
            if (px < box_rectangular[i * 4 + 0] || px > box_rectangular[i * 4 + 2]) {
                continue;
            }
            if (py < box_rectangular[i * 4 + 1] || py > box_rectangular[i * 4 + 3]) {
                continue;
            }
            float z = res_outputs_[i].z;
            float h = res_outputs_[i].h;
            z = z + h / 2;
            if (pz < (z - h / 2 - bottom_enlarge_value_) || pz > (z + h / 2 + top_enlarge_value_)) {
                continue;
            }
            float x1 = box_corner[i * 8 + 0];
            float x2 = box_corner[i * 8 + 2];
            float x3 = box_corner[i * 8 + 4];
            float x4 = box_corner[i * 8 + 6];
            float y1 = box_corner[i * 8 + 1];
            float y2 = box_corner[i * 8 + 3];
            float y3 = box_corner[i * 8 + 5];
            float y4 = box_corner[i * 8 + 7];
            double angl1 = (px - x1) * (x2 - x1) + (py - y1) * (y2 - y1);
            double angl2 = (px - x2) * (x3 - x2) + (py - y2) * (y3 - y2);
            double angl3 = (px - x3) * (x4 - x3) + (py - y3) * (y4 - y3);
            double angl4 = (px - x4) * (x1 - x4) + (py - y4) * (y1 - y4);
            if ((angl1 < 0 && angl2 < 0 && angl3 < 0 && angl4 < 0) ||
                (angl1 > 0 && angl2 > 0 && angl3 > 0 && angl4 > 0)) {
                res_outputs_.at(i).point_ids.push_back(point_idx);
            }
        }
    }
    double assign_point_time = timer.toc(true);
    ADEBUG << "[AssignPointsCPU] total_point size " << original_cloud_->size()
           << " nms_box_time: " << nms_time
           << " assign point time: " << assign_point_time;
}

void CPDetection::FilterObjectsbyClassNMS(
    std::vector<std::shared_ptr<base::Object>> *objects) {
    auto ordinary_nms_strategy = [&](size_t i, size_t j,
        bool use_strategy = false) {
        // return FILTER_INDEX
        size_t type_i = static_cast<size_t>(objects->at(i)->type);
        size_t type_j = static_cast<size_t>(objects->at(j)->type);
        size_t filter_index =
            (objects->at(i)->confidence < objects->at(j)->confidence) ? i : j;
        // ordinary_nms_strategy: VEHICLE > BICYCLE > PEDESTRIAN > TRAFFICCONE
        if (nms_strategy_ || use_strategy) {
            return type_i > type_j ? j : i;
        }
        return filter_index;
    };

    auto special_nms_strategy = [&](size_t keep_index,
          base::ObjectType keep_type, size_t filter_index,
          base::ObjectType filter_type, float conf_buffer) {
        if (nms_strategy_) {
            return;
        }
        objects->at(keep_index)->lidar_supplement.raw_probs.clear();
        objects->at(keep_index)->type_probs.clear();

        objects->at(keep_index)->lidar_supplement.raw_probs.push_back(
            std::vector<float>(static_cast<int>(
                base::ObjectType::MAX_OBJECT_TYPE), 0.f));
        float sum_confidence = objects->at(keep_index)->confidence +
                               objects->at(filter_index)->confidence;
        float filter_prob = 1.0 * (objects->at(filter_index)->confidence
              + conf_buffer) / (sum_confidence + conf_buffer);
        float keep_prob = 1.0 * objects->at(keep_index)->confidence /
              (sum_confidence + conf_buffer);
        objects->at(keep_index)->lidar_supplement.raw_probs.back()[
            static_cast<int>(filter_type)] = filter_prob;
        objects->at(keep_index)->lidar_supplement.raw_probs.back()[
            static_cast<int>(keep_type)] = keep_prob;
        objects->at(keep_index)->type_probs.assign(
            objects->at(keep_index)->lidar_supplement.raw_probs.back().begin(),
            objects->at(keep_index)->lidar_supplement.raw_probs.back().end());
    };

    auto get_obj_corners = [&](const base::ObjectPtr obj) {
        std::vector<float> box_corner;
        box_corner.resize(8);
        float x = obj->center(0);
        float y = obj->center(1);
        float l = obj->size(0);
        float w = obj->size(1);
        float r = obj->theta;
        // if (length_enlarge_value_ > 0) {
        //     l = l + length_enlarge_value_;
        // }
        // if (width_enlarge_value_ > 0) {
        //     w = w + width_enlarge_value_;
        // }
        float cos_r = cos(r);
        float sin_r = sin(r);
        float hl = l * 0.5;
        float hw = w * 0.5;
        float x1 = (-hl) * cos_r - (-hw) * sin_r + x;
        float y1 = (-hl) * sin_r + (-hw) * cos_r + y;
        float x2 = ( hl) * cos_r - (-hw) * sin_r + x;
        float y2 = ( hl) * sin_r + (-hw) * cos_r + y;
        float x3 = ( hl) * cos_r - ( hw) * sin_r + x;
        float y3 = ( hl) * sin_r + ( hw) * cos_r + y;
        float x4 = (-hl) * cos_r - ( hw) * sin_r + x;
        float y4 = (-hl) * sin_r + ( hw) * cos_r + y;
        box_corner[0] = x1;
        box_corner[1] = y1;
        box_corner[2] = x2;
        box_corner[3] = y2;
        box_corner[4] = x3;
        box_corner[5] = y3;
        box_corner[6] = x4;
        box_corner[7] = y4;
        return box_corner;
    };

    auto calculate_nms = [&](base::ObjectPtr obj1, base::ObjectPtr obj2) {
        std::vector<float> box_corner1 = get_obj_corners(obj1);
        std::vector<float> box_corner2 = get_obj_corners(obj2);
        float area_inter = CalculateUnionArea(box_corner1.data(), box_corner2.data());
        float overlap = area_inter / std::max(obj1->size(0) * obj1->size(1) + obj2->size(0) * obj2->size(1) - area_inter, 0.001f);
        return overlap;
    };

    std::vector<bool> delete_array(objects->size(), false);
    std::vector<bool> nms_visited(objects->size(), false);
    std::vector<std::vector<size_t>> nms_pairs;
    nms_pairs.resize(objects->size());
    // different class nms
    for (size_t i = 0; i < objects->size(); i++) {
        auto &obj_i = objects->at(i);
        if (!obj_i || nms_visited[i]) {
            continue;
        }
        for (size_t j = i + 1; j < objects->size(); j++) {
            auto &obj_j = objects->at(j);
            if (!obj_j) {
                continue;
            }
            if (calculate_nms(obj_i, obj_j) <= class_nms_iou_thres_) {
                continue;
            }
            if (static_cast<int>(obj_i->type) == static_cast<int>(obj_j->type)) {
                continue;
            }
            nms_visited[j] = true;
            nms_pairs[i].push_back(j);
        }
    }
    // do NMS
    for (size_t i = 0; i < objects->size(); i++) {
        if (nms_pairs[i].size() == 0) {
            continue;
        }
        if (nms_pairs[i].size() == 1) {
            size_t filter_index = ordinary_nms_strategy(i, nms_pairs[i][0]);
            size_t keep_index = filter_index == i ? nms_pairs[i][0] : i;

            delete_array[keep_index] = false;
            delete_array[filter_index] = true;

            // special-nms-strategy
            auto filter_type = objects->at(filter_index)->type;
            auto reserve_type = objects->at(keep_index)->type;
            // Only trafficcone reserve but bicycle filter -> add bicycle conf
            if (filter_type == base::ObjectType::BICYCLE &&
                reserve_type == base::ObjectType::UNKNOWN) {
                special_nms_strategy(keep_index, base::ObjectType::UNKNOWN,
                    filter_index, base::ObjectType::BICYCLE, 0.3);
            }
            // Only pedestrian reserve but bicycle filter -> add bicycle conf
            if (filter_type == base::ObjectType::BICYCLE &&
                reserve_type == base::ObjectType::PEDESTRIAN) {
                special_nms_strategy(keep_index, base::ObjectType::PEDESTRIAN,
                    filter_index, base::ObjectType::BICYCLE, 0.2);
            }
            // Only trafficcone reserve but ped filter -> add ped conf
            // if (filter_type == base::ObjectType::PEDESTRIAN &&
            //     reserve_type == base::ObjectType::UNKNOWN) {
            //     special_nms_strategy(keep_index, base::ObjectType::UNKNOWN,
            //         filter_index, base::ObjectType::PEDESTRIAN, 0.2);
            // }
            // Only ped reserve but trafficcone filter -> add trafficcone conf
            if (filter_type == base::ObjectType::UNKNOWN &&
                reserve_type == base::ObjectType::PEDESTRIAN) {
                special_nms_strategy(keep_index, base::ObjectType::PEDESTRIAN,
                  filter_index, base::ObjectType::UNKNOWN, 0.2);
            }
            ADEBUG << "Only Two NMS: reserve " << keep_index
                   << " filter " << filter_index;
        }
        if (nms_pairs[i].size() >= 2) {
            // car strategy
            bool have_car = false;
            if (objects->at(i)->type == base::ObjectType::VEHICLE) {
                have_car = true;
            }
            for (size_t k = 0; k < nms_pairs[i].size() && !have_car; k++) {
                auto type = objects->at(nms_pairs[i][k])->type;
                if (type == base::ObjectType::VEHICLE) {
                    have_car = true;
                    break;
                }
            }
            // no car -> BICYCLE priorTo PEDESTRIAN priorTo TRAFFICCONE
            // have car -> confidence
            if (have_car) {
                auto keep_conf = objects->at(i)->confidence;
                auto keep_index = i;
                for (size_t k = 0; k < nms_pairs[i].size(); k++) {
                    if (objects->at(nms_pairs[i][k])->confidence > keep_conf) {
                        delete_array[keep_index] = true;
                        keep_conf = objects->at(nms_pairs[i][k])->confidence;
                        keep_index = nms_pairs[i][k];
                    } else {
                        delete_array[nms_pairs[i][k]] = true;
                        ADEBUG << "NMS filter_index: " << nms_pairs[i][k];
                    }
                }
            } else {
                auto keep_type = static_cast<size_t>(objects->at(i)->type);
                auto keep_index = i;
                for (size_t k = 0; k < nms_pairs[i].size(); k++) {
                    if (static_cast<size_t>(
                        objects->at(nms_pairs[i][k])->type) > keep_type) {
                        delete_array[keep_index] = true;
                        keep_type = static_cast<size_t>(
                            objects->at(nms_pairs[i][k])->type);
                        keep_index = nms_pairs[i][k];
                    } else {
                        delete_array[nms_pairs[i][k]] = true;
                        ADEBUG << "NMS filter_index: " << nms_pairs[i][k];
                    }
                }
            }
        }
    }
    size_t valid_num = 0;
    for (size_t i = 0; i < objects->size(); i++) {
        auto &object = objects->at(i);
        if (!delete_array[i]) {
            objects->at(valid_num) = object;
            valid_num++;
        }
    }
    objects->resize(valid_num);
}

void CPDetection::FilterObjectsbySemanticType(
    std::vector<std::shared_ptr<base::Object>> *objects) {
    std::vector<bool> filter_flag(objects->size(), false);
    for (size_t i = 0; i < objects->size(); i++) {
        auto object = objects->at(i);
        std::vector<int> type_count(static_cast<int>(PointSemanticLabel::MAX_LABEL), 0);
        for (size_t k = 0; k < object->lidar_supplement.cloud.size(); k++) {
            // uint8_t value = object->lidar_supplement.cloud.points_semantic_label(k);
            // int index = static_cast<int>(value);
            int index = static_cast<int>(GetSemanticLabel(object->lidar_supplement.cloud.points_semantic_label(k)));
            if (index >= 0 && index < static_cast<int>(PointSemanticLabel::MAX_LABEL)) {
                type_count[index]++;
            }
        }
        // get max index
        int max_value = -1;
        int max_index = 0;
        for (size_t j = 0; j < type_count.size(); j++) {
            if (type_count.at(j) > max_value) {
                max_value = type_count.at(j);
                max_index = j;
            }
        }
        if (fore_semantic_filter_map_.find(max_index) == fore_semantic_filter_map_.end()) {
            // Not-write == No-filter
            continue;
        }
        float ratio = static_cast<float>(object->lidar_supplement.num_points_in_roi * 1.0f /
            object->lidar_supplement.cloud.size());
        ratio = 1 - ratio;
        float range = sqrt(object->center(0) * object->center(0) + object->center(1) * object->center(1));
        if (ratio >= fore_semantic_filter_map_.at(max_index).first && 
            range <= fore_semantic_filter_map_.at(max_index).second) {
            ADEBUG << "id = " << object->id << ": total_size: " << object->lidar_supplement.cloud.size() << ", roi_size: "
               << object->lidar_supplement.num_points_in_roi << ", out_ratio: " << ratio
               << ", semantic_type: " << max_index;
            filter_flag.at(i) = true;
        }
    }
    // filter object which semantic type is not unknown
    int valid_size = 0;
    for (size_t i = 0; i < objects->size(); i++) {
        if (filter_flag.at(i)) {
            continue;
        }
        objects->at(valid_size) = objects->at(i);
        valid_size++;
    }
    AINFO << "Filter " << (objects->size() - valid_size) << " objects from " << objects->size() << " objects.";
    objects->resize(valid_size);
}

bool CPDetection::Detect(const CPDetectorOptions &options,
    LidarFrame *frame) {
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
    frame->segmented_objects.clear();
    original_cloud_ = frame->cloud;
    original_world_cloud_ = frame->world_cloud;
    lidar_frame_ref_ = frame;
    cur_cloud_ptr_ = std::shared_ptr<base::PointFCloud>(
        new base::PointFCloud(*original_cloud_));

    // BASE_GPU_CHECK(cudaStreamCreate(&stream_));
    if (cudaSetDevice(cpdet_config_.model_param().preprocess().gpu_id()) != cudaSuccess) {
        AERROR << "Failed to set device to "
               << cpdet_config_.model_param().preprocess().gpu_id();
        return false;
    }

    Timer timer;
    PointCloudPreprocess();
    AINFO << "Downsample PointCloud from " << original_cloud_->size() << " to " << cur_cloud_ptr_->size();
    double pointcloud_pre_time = timer.toc(true);

    model_cloud_size_ = static_cast<int>(cur_cloud_ptr_->size());
    total_cloud_size_ = static_cast<int>(original_cloud_->size());
    GeneratePfnFeature();
    double pfn_feature_generate_time = timer.toc(true);

    // pfn_output
    pfn_inference_->Infer();
    // cudaStreamSynchronize(stream_);
    double pfn_inference_time = timer.toc(true);

    // generate backbone feature
    GenerateBackboneFeature(pfn_pillar_feature_blob_.get());
    double backbone_feature_generate_time = timer.toc(true);

    // backbone output
    backbone_inference_->Infer();
    // cudaStreamSynchronize(stream_);
    double backbone_inference_time = timer.toc(true);

    GetObjectsAndAssignPoints();
    double get_objects_and_assign_time = timer.toc(true);

    GenerateObjects(frame);
    std::stringstream sstr;
    sstr << "[CPDetection BeforeNMS] "
       << std::to_string(frame->timestamp)
       << " objs: " << frame->segmented_objects.size() << std::endl;
    for (auto obj : frame->segmented_objects) {
      sstr << "id = " << obj->id << ": " << obj->center(0) << ", "
           << obj->center(1) << ", " << obj->center(2) << ", "
           << obj->size(0) << ", " << obj->size(1) << ", "
           << obj->size(2) << ", " << obj->theta << ", "
           << static_cast<int>(obj->type) << ", "
           << obj->confidence << std::endl;
    }
    ADEBUG << sstr.str();
    // filter
    if (num_tasks_ > 1 && inter_class_nms_) {
        FilterObjectsbyClassNMS(&frame->segmented_objects);
    }
    std::stringstream ssstr;
    ssstr << "[CenterPointDetection AfterNMS] "
        << std::to_string(frame->timestamp)
        << " objs: " << frame->segmented_objects.size() << std::endl;
    for (auto obj : frame->segmented_objects) {
      ssstr << "id = " << obj->id << ": " << obj->center(0) << ", "
            << obj->center(1) << ", " << obj->center(2) << ", "
            << obj->size(0) << ", " << obj->size(1) << ", "
            << obj->size(2) << ", " << obj->theta << ", "
            << static_cast<int>(obj->type) << std::endl;
    }
    ADEBUG << ssstr.str();

    // filter semantic if it's not unknown
    SetPointsInROI(&frame->segmented_objects);
    if (filter_by_semantic_type_) {
        FilterObjectsbySemanticType(&frame->segmented_objects);
    }
    RemoveForePoints(&frame->segmented_objects);

    res_outputs_.clear();
    double post_process_time = timer.toc(true);

    double total_time = pointcloud_pre_time + pfn_feature_generate_time +
        pfn_inference_time + backbone_feature_generate_time +
        backbone_inference_time + get_objects_and_assign_time + post_process_time;
    AINFO << "[CpDetTime] " << std::to_string(frame->timestamp)
          << ", pointcloud preprocess: " << pointcloud_pre_time
          << ", pfn feature generate: " << pfn_feature_generate_time
          << ", pfn inference: " << pfn_inference_time
          << ", backbone feature generate: " << backbone_feature_generate_time
          << ", backbone inference: " << backbone_inference_time
          << ", getObjects and assignPoints: " << get_objects_and_assign_time
          << ", postProcess: " << post_process_time
          << ", Total: " << total_time;
    return true;
}

PERCEPTION_REGISTER_LIDARDETECTOR(CPDetection);

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
