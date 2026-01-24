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

#include "modules/perception/lidar_segmentation/segmentor/graph_segmentation/graph_segmentation.h"

#include "modules/perception/common/util.h"

namespace apollo {
namespace perception {
namespace lidar {

bool GraphSegmentation::Init(const LidarDetectorInitOptions& options) {
    std::string config_file = GetConfigFile(options.config_path, options.config_file);
    ACHECK(apollo::cyber::common::GetProtoFromFile(config_file, &bg_config_));
    // get config
    // grid_width_ = bg_config_.grid_width();
    // grid_height_ = bg_config_.grid_height();
    resolution_ = bg_config_.resolution();
    threshold_ = bg_config_.threshold();
    min_pt_number_ = bg_config_.min_pt_number();
    search_radius_ = bg_config_.search_radius();
    height_threshold_ = bg_config_.height_threshold();
    semantic_cost_ = bg_config_.semantic_cost();

    // initialize graph segmentor
    graph_segmentor_.Init(threshold_);
    // ACHECK(bg_map_.Init(grid_width_, grid_height_, resolution_, height_threshold_));

    ACHECK(bg_map_.Init(
            bg_config_.xmin(),
            bg_config_.xmax(),
            bg_config_.ymin(),
            bg_config_.ymax(),
            resolution_,
            height_threshold_));
    grid_width_ = static_cast<int>((bg_config_.ymax() - bg_config_.ymin()) * 1.0f / resolution_);
    grid_height_ = static_cast<int>((bg_config_.xmax() - bg_config_.xmin()) * 1.0f / resolution_);

    // clear background objects
    bg_objects_.clear();

    AINFO << "graph segmentation config: " << bg_config_.DebugString();
    return true;
}

bool GraphSegmentation::Detect(const LidarDetectorOptions& options, LidarFrame* frame) {
    // clear background objects buffer
    bg_objects_.clear();

    // reset background map
    if (!bg_map_.Reset(frame->cloud->size())) {
        AERROR << "bg_map_ Reset error.";
        return false;
    }
    // update map
    if (!bg_map_.UpdateMap(frame)) {
        AERROR << "bg_map_ Update error.";
        return false;
    }
    // cluster
    if (!Segment(frame)) {
        AERROR << "Graph Segment error.";
        return false;
    }
    // split
    if (!Split(frame)) {
        AERROR << "Split object error.";
        return false;
    }
    // set default value and id for bg_objects_
    if (!SetBgObjectDefaulVal(frame)) {
        AERROR << "SetBgObjectDefaulVal error.";
        return false;
    }

    // get all background objects
    frame->segmented_objects.insert(frame->segmented_objects.end(), bg_objects_.begin(), bg_objects_.end());
    AINFO << "Graph segment, background objects number: " << bg_objects_.size();
    return true;
}

bool GraphSegmentation::Segment(LidarFrame* frame) {
    std::vector<algorithm::Edge> edge_set;
    edge_set.clear();
    algorithm::Edge edge;
    for (int x = 0; x < grid_height_; ++x) {
        for (int y = 0; y < grid_width_; ++y) {
            int grid_idx = x * grid_width_ + y;
            if (!bg_map_.bg_nodes_.at(grid_idx).is_valid) {
                continue;
            }

            edge.a = bg_map_.bg_nodes_.at(grid_idx).id;
            BgNode n1 = bg_map_.bg_nodes_.at(grid_idx);

            // rows
            for (int ny = y; ny < grid_width_ && ny <= y + search_radius_; ++ny) {
                int grid_ny = x * grid_width_ + ny;
                if (bg_map_.bg_nodes_.at(grid_ny).is_valid) {
                    edge.b = bg_map_.bg_nodes_.at(grid_ny).id;
                    BgNode n2 = bg_map_.bg_nodes_.at(grid_ny);
                    edge.w = NodeDistance(n1, x, y, n2, x, ny);
                    edge_set.push_back(edge);
                }
            }
            // next line, next column
            for (int nx = x + 1; nx < grid_height_ && nx <= x + search_radius_; ++nx) {
                for (int ny = y; ny < grid_width_ && ny < y + search_radius_; ++ny) {
                    int grid_nxny = nx * grid_width_ + ny;
                    if (bg_map_.bg_nodes_.at(grid_nxny).is_valid) {
                        edge.b = bg_map_.bg_nodes_.at(grid_nxny).id;
                        BgNode n2 = bg_map_.bg_nodes_.at(grid_nxny);
                        edge.w = NodeDistance(n1, x, y, n2, nx, ny);
                        edge_set.push_back(edge);
                    }
                }
            }
        }
    }

    if (edge_set.size() == 0) {
        AINFO << "Graph segmentation, edges are empty.";
        return true;
    }

    // graph segment
    graph_segmentor_.SegmentGraph(bg_map_.id(), edge_set.size(), edge_set.data());
    algorithm::Universe* universe = graph_segmentor_.mutable_universe();
    std::vector<size_t> mapping(bg_map_.id(), 0);
    std::map<size_t, size_t> label_map;
    size_t label = 0;
    for (int i = 0; i < bg_map_.id(); ++i) {
        mapping[i] = universe->Find(i);
        auto iter = label_map.find(mapping[i]);
        if (iter == label_map.end()) {
            // key:value --> father:label
            label_map.emplace(mapping[i], label);
            mapping[i] = label;
            label++;
        } else {
            mapping[i] = iter->second;
        }
    }

    // get point clusters
    auto original_cloud = frame->cloud;
    // SppClusterList clusters;
    clusters_.Reset();
    clusters_.resize(label_map.size());
    for (size_t i = 0; i < bg_map_.point_mask_.size(); ++i) {
        if (!bg_map_.point_mask_.at(i)) {
            continue;
        }
        size_t index = bg_map_.point_idx_.at(i);
        size_t cluster_label = mapping.at(bg_map_.bg_nodes_.at(index).id);
        clusters_.AddPointSample(cluster_label, original_cloud->at(i), original_cloud->points_height(i), i);
    }
    GetObjectsFromClusters(frame);

    for (int i = 0; i <= clusters_.size() - 1; i++) {
        clusters_[i]->clear();
    }
    clusters_.Reset();

    return true;
}

float GraphSegmentation::NodeDistance(BgNode n1, int x1, int y1, BgNode n2, int x2, int y2) {
    float x_diff = static_cast<float>(x1 - x2) * resolution_;
    float y_diff = static_cast<float>(y1 - y2) * resolution_;
    float mean_height_diff = n1.mean_height - n2.mean_height;
    float max_height_diff = n1.max_height - n2.max_height;
    float mean_intensity = (n1.mean_intensity - n2.mean_intensity) / 255.0;
    // semantic label
    int max_label_1 = static_cast<int>(std::distance(
            n1.pc_semantic_types.begin(), std::max_element(n1.pc_semantic_types.begin(), n1.pc_semantic_types.end())));
    int max_label_2 = static_cast<int>(std::distance(
            n2.pc_semantic_types.begin(), std::max_element(n2.pc_semantic_types.begin(), n2.pc_semantic_types.end())));
    // motion label
    int max_motion_label_1 = static_cast<int>(std::distance(
            n1.pc_motion_types.begin(), std::max_element(n1.pc_motion_types.begin(), n1.pc_motion_types.end())));
    int max_motion_label_2 = static_cast<int>(std::distance(
            n2.pc_motion_types.begin(), std::max_element(n2.pc_motion_types.begin(), n2.pc_motion_types.end())));
    // total cost
    float total_cost = std::sqrt(
            x_diff * x_diff + y_diff * y_diff + mean_height_diff * mean_height_diff + max_height_diff * max_height_diff
            + mean_intensity * mean_intensity);

    // semantic coefficient
    if ((max_label_1 != static_cast<int>(PointSemanticLabel::UNKNOWN))
        && (max_label_2 != static_cast<int>(PointSemanticLabel::UNKNOWN))) {
        if (max_label_1 == max_label_2) {
            total_cost *= bg_config_.same_semantic_coefficient();
        } else {
            total_cost *= bg_config_.diff_semantic_coefficient();
        }
    }

    // motion coefficient
    if ((max_motion_label_1 != static_cast<int>(PointMotionLabel::UNKNOWN))
        && (max_motion_label_2 != static_cast<int>(PointMotionLabel::UNKNOWN))) {
        if (max_motion_label_1 == max_motion_label_2) {
            total_cost *= bg_config_.same_motion_coefficient();
        } else {
            total_cost *= bg_config_.diff_semantic_coefficient();
        }
    }

    return total_cost;
}

void GraphSegmentation::GetObjectsFromClusters(LidarFrame* frame) {
    // point cloud
    auto original_cloud_ = frame->cloud;
    auto original_world_cloud_ = frame->world_cloud;
    // clear background objects
    bg_objects_.clear();
    // background object
    for (size_t i = 0; i < clusters_.size(); ++i) {
        auto cluster = clusters_[i];
        if (cluster->point_ids.size() < min_pt_number_) {
            continue;
        }
        base::Object object;
        object.confidence = cluster->confidence;
        for (auto idx : cluster->point_ids) {
            object.lidar_supplement.point_ids.push_back(static_cast<int>(idx));
        }
        object.lidar_supplement.num_points_in_roi = cluster->point_ids.size();
        object.lidar_supplement.cloud.CopyPointCloud(*original_cloud_, cluster->point_ids);
        object.lidar_supplement.cloud_world.CopyPointCloud(*original_world_cloud_, cluster->point_ids);
        // copy to background objects
        std::shared_ptr<base::Object> obj(new base::Object);
        *obj = object;
        bg_objects_.push_back(std::move(obj));
    }
    // background object builder
    BgObjectBuilder(&bg_objects_, frame->lidar2novatel_extrinsics);
}

bool GraphSegmentation::Split(LidarFrame* frame) {
    std::vector<std::shared_ptr<base::Object>> split_objects;
    // record object number
    int before_split_num = bg_objects_.size();

    int valid_number = 0;
    for (size_t i = 0; i < bg_objects_.size(); i++) {
        std::shared_ptr<base::Object> obj = bg_objects_.at(i);
        if (NeedSplit(obj)) {
            std::vector<base::ObjectPtr> this_split;
            SplitObject(obj, &this_split, bg_config_.split_distance());
            BgObjectBuilder(&this_split, frame->lidar2novatel_extrinsics);
            split_objects.insert(split_objects.end(), this_split.begin(), this_split.end());
            continue;
        }
        bg_objects_.at(valid_number) = obj;
        valid_number += 1;
    }
    int split_number = bg_objects_.size() - valid_number;
    bg_objects_.resize(valid_number);

    // put all split object to bg_objects_
    bg_objects_.insert(bg_objects_.end(), split_objects.begin(), split_objects.end());
    int after_split_num = bg_objects_.size();

    AINFO << "Before split object number: " << before_split_num << ", split number: " << split_number
          << ", after split number: " << after_split_num;
    return true;
}

bool GraphSegmentation::NeedSplit(std::shared_ptr<base::Object> object) {
    // get length and width
    float length, width;
    if (object->size(0) > object->size(1)) {
        length = object->size(0);
        width = object->size(1);
    } else {
        length = object->size(1);
        width = object->size(0);
    }
    if ((width > kEpsilon) && (length / (width + kEpsilon) > bg_config_.split_aspect_ratio())) {
        return true;
    }
    return false;
}

bool GraphSegmentation::SetBgObjectDefaulVal(LidarFrame* frame) {
    int start_index = frame->segmented_objects.size();
    for (size_t i = 0; i < bg_objects_.size(); i++) {
        std::shared_ptr<base::Object> object = bg_objects_.at(i);
        object->id = start_index++;
        object->lidar_supplement.is_in_roi = true;
        object->lidar_supplement.is_background = true;
        object->lidar_supplement.is_clustered = true;
        // classification
        object->type = base::ObjectType::UNKNOWN;
        object->lidar_supplement.raw_probs.push_back(
                std::vector<float>(static_cast<int>(base::ObjectType::MAX_OBJECT_TYPE), 0.f));
        object->lidar_supplement.raw_probs.back()[static_cast<int>(object->type)] = 1.0f;
        object->lidar_supplement.raw_classification_methods.push_back(Name());
        // copy to type
        object->type_probs.assign(
                object->lidar_supplement.raw_probs.back().begin(), object->lidar_supplement.raw_probs.back().end());
        // lidar supplement detections
        object->lidar_supplement.detections.resize(7);
        for (size_t jj = 0; jj < 7; jj++) {
            object->lidar_supplement.detections[jj] = 0.0;
        }
    }
    return true;
}

PERCEPTION_REGISTER_LIDARDETECTOR(GraphSegmentation);

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
