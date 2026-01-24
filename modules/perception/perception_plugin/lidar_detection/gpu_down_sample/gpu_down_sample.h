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

#pragma once

#include <limits>
#include <string>
#include <vector>

#include "modules/perception/perception_plugin/lidar_detection/gpu_down_sample/proto/gpu_down_sample.pb.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/perception/common/base/point_cloud.h"
#include "modules/perception/common/interface/base_down_sample.h"
#include "modules/perception/perception_plugin/lidar_detection/gpu_down_sample/voxel_downsample.h"

namespace apollo {
namespace perception {
namespace lidar {

class GpuDownSample : public BaseDownSample {
public:
    GpuDownSample() = default;
    virtual ~GpuDownSample() = default;

    /**
     * @brief Init of GpuDownSample object
     *
     * @param options object down sample options
     * @return true
     * @return false
     */
    bool Init(const DownSampleInitOptions& options = DownSampleInitOptions()) override;

    /**
     * @brief filter mark area objects
     *
     * @param options down sample options
     * @param cloud_ptr point cloud to process
     * @return true
     * @return false
     */
    bool Process(const DownSampleOptions& options, base::PointFCloudPtr& cloud_ptr) override;

    /**
     * @brief Name of GpuDownSample object
     *
     * @return std::string name
     */
    std::string Name() const override {
        return "GpuDownSample";
    }

private:
    void warmUpGPU();
    int getNumberOfAvailableThreads();

    bool downSample(
            base::PointF* point_cloud,
            int& point_cloud_size,
            float resolution_x,
            float resolution_y,
            float resolution_z,
            bool use_centroid_downsample);
    VoxelDownSampleCuda* voxel_downSample_ptr;
    double downsample_voxel_size_x_ = 0.09;
    double downsample_voxel_size_y_ = 0.09;
    double downsample_voxel_size_z_ = 0.09;
    bool downsample_use_centroid;
};  // class GpuDownSample

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::perception::lidar::GpuDownSample, BaseDownSample)

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
