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

#pragma once

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <limits>
#include <algorithm>

#include "modules/perception/pointcloud_motion/parser/pointwise_unet/proto/motion_unet.pb.h"

#include "modules/perception/common/base/blob.h"
#include "modules/perception/common/inference/inference.h"
#include "modules/perception/common/inference/inference_factory.h"
#include "modules/perception/pointcloud_motion/interface/base_motion_parser.h"

namespace apollo {
namespace perception {
namespace lidar {

struct Unode {
    int point_size = 0;
    // gravity center
    float sum_x = 0;
    float sum_y = 0;
    float sum_z = 0;
    // geometric center
    float geometric_x = 0;
    float geometric_y = 0;
    float geometric_z = 0;
    // unet coord
    int grid_x = -1;
    int grid_y = -1;
};


class MotionUnet : public BaseMotionParser {
public:
    MotionUnet() = default;

    ~MotionUnet() {}

    bool Init(const PointCloudParserInitOptions& options = PointCloudParserInitOptions());

    bool Parse(const PointCloudParserOptions& options, LidarFrame* frame);

    std::string Name() const {
        return "MotionUnet";
    }

private:
    bool InitModel();

    void GetValidPointcloud(base::PointFCloudPtr cloud) {
        // valid point index
        valid_point_index_.clear();
        valid_point_index_.reserve(cloud->size());
        // grid index vector
        grid_index_vec_.clear();
        grid_index_vec_.reserve(cloud->size() * 2);
        for (size_t i = 0; i < cloud->size(); i++) {
            auto pt = cloud->at(i);
            // check valid range
            if (pt.x < min_x_ || pt.x > max_x_ || pt.y < min_y_ || pt.y > max_y_ || pt.z < min_height_
                || pt.z > max_height_) {
                continue;
            }
            // get coordinate of pointcloud bev
            int grid_x = (pt.x - min_x_) / resolution_;
            int grid_y = (pt.y - min_y_) / resolution_;
            if (grid_x >= 0 && grid_x < unet_width_ && grid_y >= 0 && grid_y < unet_length_) {
                // valid point index
                valid_point_index_.push_back(i);
                // valid grid_y, grid_x
                grid_index_vec_.push_back(grid_y);
                grid_index_vec_.push_back(grid_x);

                // max point number
                if (static_cast<int>(valid_point_index_.size()) >= max_point_number_) {
                    break;
                }
            }
        }
    }

    // pfn feature generater
    void GeneratePfnFeature(base::PointFCloudPtr cloud) {
        if (use_gpu_) {
            GeneratePfnFeatureGPU(cloud);
        } else {
            GeneratePfnFeatureCPU(cloud);
        }
    }
    void GeneratePfnFeatureCPU(base::PointFCloudPtr cloud);
    void GeneratePfnFeatureGPU(base::PointFCloudPtr cloud);

    // backbone feature generater
    void GenerateBackboneFeature(std::shared_ptr<base::Blob<float>> feature) {
        if (use_gpu_) {
            GenerateBackboneFeatureGPU(feature);
        } else {
            GenerateBackboneFeatureCPU(feature);
        }
    }
    void GenerateBackboneFeatureCPU(std::shared_ptr<base::Blob<float>> feature);
    void GenerateBackboneFeatureGPU(std::shared_ptr<base::Blob<float>> feature);

    void LabelToPointCloud(LidarFrame* frame);

    void TransformCloud(base::PointFCloudPtr &cloud_ptr,
                        const base::PointDCloudPtr &world_cloud_ptr);

private:
    // reference pointer of lidar frame
    LidarFrame* lidar_frame_ref_ = nullptr;
    base::PointFCloudPtr original_cloud_;
    base::PointDCloudPtr original_world_cloud_;
    // model params
    bool use_gpu_;
    motion_unet::MotionUnetModelParam model_param_;
    std::shared_ptr<inference::Inference> pfn_inference_;
    std::shared_ptr<inference::Inference> inference_;
    // Unet params
    float resolution_;
    int unet_width_;
    int unet_length_;
    float min_height_;
    float max_height_;
    float min_x_;
    float max_x_;
    float min_y_;
    float max_y_;
    int max_point_number_;
    float z_offset_ = 0.0;

    // pfn model blobs
    std::shared_ptr<base::Blob<float>> voxels_;           // (point_size, 10)
    std::shared_ptr<base::Blob<float>> pfn_point_feats_;  // (point_size, 64)
    // backbone model blobs
    // (point_size, 4), pillar coordinate of point
    std::shared_ptr<base::Blob<float>> coors_;
    std::shared_ptr<base::Blob<float>> point_feats_;    // (point_size, 64)
    std::shared_ptr<base::Blob<float>> bev_feature_;    // (1, 64, height, width)
    std::shared_ptr<base::Blob<float>> prev_bev_feature_;    // (1, 64, height, width)
    std::shared_ptr<base::Blob<float>> seg_pred_blob_;  // (point_size, 4)

    // point to pillar mapping
    std::vector<int> valid_point_index_;
    std::vector<int> grid_index_vec_;
    std::vector<int> valid_node_index_;
    // unet nodes, used for feature generating in cpu
    std::vector<Unode> unet_nodes_;

    // segmantic class mapping
    std::vector<uint8_t> motion_class_mapping_;

    // use for gpu feature generate
    base::PointF* pc_gpu_ = nullptr;
    cudaStream_t stream_ = 0;
    int kGPUThreadSize = 512;

    // previous pointcloud info
    std::deque<std::pair<double, base::PointDCloudPtr>> prev_world_clouds_;
};

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
