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

#include <string>

#include "cyber/common/macros.h"
#include "modules/perception/common/lib/interface/base_init_options.h"
#include "modules/perception/common/lib/registerer/registerer.h"
#include "modules/perception/common/lidar/common/lidar_frame.h"

namespace apollo {
namespace perception {
namespace lidar {

using apollo::perception::BaseInitOptions;

struct PointCloudParserInitOptions : public BaseInitOptions {
    std::string sensor_name = "velodyne64";
};

struct PointCloudParserOptions {};

class BasePointCloudParser {
public:
    /**
     * @brief Construct a new Base PointCloud Parser object
     *
     */
    BasePointCloudParser() = default;

    /**
     * @brief Destroy the Base PointCloud Parser object
     *
     */
    virtual ~BasePointCloudParser() = default;

    /**
     * @brief Init the Base PointCloud Parser object
     *
     * @param options pointcloud parser init options
     * @return true
     * @return false
     */
    virtual bool Init(const PointCloudParserInitOptions& options = PointCloudParserInitOptions()) = 0;

    /**
     * @brief Detect foreground objects
     *
     * @param options pointcloud parser options
     * @param frame lidar frame
     * @return true
     * @return false
     */
    virtual bool Parse(const PointCloudParserOptions& options, LidarFrame* frame) = 0;

    /**
     * @brief Name of Base PointCloud Parser class
     *
     * @return std::string
     */
    virtual std::string Name() const = 0;

private:
    DISALLOW_COPY_AND_ASSIGN(BasePointCloudParser);
};  // class BasePointCloudParser

PERCEPTION_REGISTER_REGISTERER(BasePointCloudParser);
#define PERCEPTION_REGISTER_POINTCLOUDPARSER(name) PERCEPTION_REGISTER_CLASS(BasePointCloudParser, name)

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
