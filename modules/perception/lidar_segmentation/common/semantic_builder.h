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

#include "modules/perception/common/base/object.h"
#include "modules/perception/common/base/object_types.h"
#include "modules/perception/common/lidar/common/lidar_frame.h"
#include "modules/perception/common/lidar/common/lidar_point_label.h"

namespace apollo {
namespace perception {
namespace lidar {

using apollo::perception::base::ObjectPtr;

struct SemanticBuilderInitOptions {};

struct SemanticBuilderOptions {};

class SemanticBuilder {
public:
    /**
     * @brief Construct a new Semantic Builder
     *
     */
    SemanticBuilder() = default;

    /**
     * @brief Destroy the Semantic Builder
     *
     */
    ~SemanticBuilder() = default;

    /**
     * @brief Init the Semantic Builder with initialization options
     *
     * @param options
     * @return true
     * @return false
     */
    bool Init(const SemanticBuilderInitOptions& options = SemanticBuilderInitOptions());

    /**
     * @brief Get object semantic type.
     *
     * @param options semantic builder options
     * @param frame lidar frame
     * @return true
     * @return false
     */
    bool Build(const SemanticBuilderOptions& options, LidarFrame* frame);

    /**
     * @brief Name of Semantic Builder
     *
     * @return std::string
     */
    std::string Name() const {
        return "SemanticBuilder";
    }

private:
    /**
     * @brief Get the object semantic and motion types
     *
     * @param object
     */
    void GetObjSemanticAndMotionType(ObjectPtr object);
};  // class SemanticBuilder

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
