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

#include "modules/perception/pointcloud_semantics/parser/pointwise_unet/proto/pointwise_unet.pb.h"

#include "modules/perception/common/base/blob.h"
#include "modules/perception/common/inference/inference.h"
#include "modules/perception/common/inference/inference_factory.h"
#include "modules/perception/pointcloud_semantics/interface/base_pointcloud_parser.h"

namespace apollo {
namespace perception {
namespace lidar {

struct UnetNode {
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

class PointWiseUnet : public BasePointCloudParser {
public:
    PointWiseUnet() = default;

    ~PointWiseUnet() {}

    bool Init(const PointCloudParserInitOptions& options = PointCloudParserInitOptions());

    bool Parse(const PointCloudParserOptions& options, LidarFrame* frame);

    std::string Name() const {
        return "PointWiseUnet";
    }

private:
    bool InitModel();

    void GetValidPointcloud() {
        // valid point index
        valid_point_index_.clear();
        valid_point_index_.reserve(original_cloud_->size());
        // grid index vector
        grid_index_vec_.clear();
        grid_index_vec_.reserve(original_cloud_->size() * 2);
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
                // valid point index
                valid_point_index_.push_back(i);
                // valid grid_y, grid_x
                grid_index_vec_.push_back(grid_y);
                grid_index_vec_.push_back(grid_x);

                // max point number
                if (valid_point_index_.size() >= max_point_number_) {
                    break;
                }
            }
        }
    }

    // pfn feature generater
    void GeneratePfnFeature() {
        if (use_gpu_) {
            GeneratePfnFeatureGPU();
        } else {
            GeneratePfnFeatureCPU();
        }
    }
    void GeneratePfnFeatureCPU();
    void GeneratePfnFeatureGPU();

    // backbone feature generater
    void GenerateBackboneFeature() {
        if (use_gpu_) {
            GenerateBackboneFeatureGPU();
        } else {
            GenerateBackboneFeatureCPU();
        }
    }
    void GenerateBackboneFeatureCPU();
    void GenerateBackboneFeatureGPU();

    void LabelToPointCloud(LidarFrame* frame);

private:
    // reference pointer of lidar frame
    LidarFrame* lidar_frame_ref_ = nullptr;
    base::PointFCloudPtr original_cloud_;
    base::PointDCloudPtr original_world_cloud_;
    // model params
    bool use_gpu_;
    pointwise_unet::PointWiseUnetModelParam model_param_;
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

    // pfn model blobs
    std::shared_ptr<base::Blob<float>> voxels_;           // (point_size, 10)
    std::shared_ptr<base::Blob<float>> pfn_point_feats_;  // (point_size, 64)
    // backbone model blobs
    // (point_size, 4), pillar coordinate of point
    std::shared_ptr<base::Blob<float>> coors_;
    std::shared_ptr<base::Blob<float>> point_feats_;    // (point_size, 64)
    std::shared_ptr<base::Blob<float>> bev_feature_;    // (1, 64, height, width)
    std::shared_ptr<base::Blob<float>> seg_pred_blob_;  // (point_size, 7)

    // point to pillar mapping
    std::vector<int> valid_point_index_;
    std::vector<int> grid_index_vec_;
    std::vector<int> valid_node_index_;
    // unet nodes, used for feature generating in cpu
    std::vector<UnetNode> unet_nodes_;

    // segmantic class mapping
    std::vector<uint8_t> sematic_class_mapping_;

    // use for gpu feature generate
    base::PointF* pc_gpu_ = nullptr;
    cudaStream_t stream_ = 0;
    int kGPUThreadSize = 512;
};

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
