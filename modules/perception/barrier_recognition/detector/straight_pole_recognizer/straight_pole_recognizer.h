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

#include <iostream>
#include <string>
#include <vector>

#include "cyber/common/log.h"
#include "cyber/common/file.h"
#include "modules/perception/common/util.h"
#include "modules/perception/common/base/object.h"
#include "modules/perception/barrier_recognition/detector/proto/straight_pole_barrier_config.pb.h"
#include "modules/perception/barrier_recognition/interface/base_barrier_recognizer.h"

namespace apollo {
namespace perception {
namespace lidar {

class StraightPoleRecognizer : public BaseBarrierRecognizer {
public:
    StraightPoleRecognizer() = default;
    virtual ~StraightPoleRecognizer() = default;

    /**
     * @brief Init of StraightPoleRecognizer object
     *
     * @param options object filer options
     * @return true
     * @return false
     */
    bool Init(const BarrierRecognizerInitOptions& options = BarrierRecognizerInitOptions()) override;

    /**
     * @brief calculate the relative position of the vehicle based on the reflector
     *
     * @param options object filter options
     * @param message pointcloud message
     * @param frame lidar frame to filter
     * @param dock_relative_points vehicle position relative to reflectors
     * @return true
     * @return false
     */
    bool Recognize(const BarrierRecognizerOptions& options, 
                   LidarFrame* frame, float& open_percent) override;

    /**
     * @brief Name of the class
     *
     * @return std::string name
     */
    std::string Name() const override {
        return "StraightPoleRecognizer";
    }

private:
    /**
     * @brief calculate the angle between 2 points
     *
     * @param pt1 first point
     * @param pt2 second point
     * @return float angle, 0-90
     */
    float AngleBetweenTwoPoints(base::PointF& pt1, base::PointF& pt2);

    /**
     * @brief extend or shrink the boudary of 2d box
     *
     * @param box_corner original box
     * @param extended_box_corner output box
     */
    void ExtendBox(const std::vector<double>& box_corner, 
                   std::vector<double>& extended_box_corner);

    /**
     * @brief filter points cloud inside given box
     *
     * @param frame lidar frame
     * @param world_box_corner box in world coordinate system
     * @param roi_cloud roi point cloud in local coordinate system
     * @param roi_world_cloud roi point cloud in world coordinate system
     * @param local_box_corner box in local coordinate system
     * @return true
     * @return false
     */
    bool Preprocess(
            LidarFrame* frame,
            const std::vector<double>& world_box_corner,
            std::vector<base::PointF>& roi_cloud,
            std::vector<base::PointD>& roi_world_cloud,
            std::vector<float>& local_box_corner);

    /**
     * @brief check whether the point is in box
     *
     * @param pt point
     * @param box_corner box in world coordinate system
     * @return true
     * @return false
     */
    bool PtsInRoi(const base::PointD& pt, 
                  const std::vector<double>& box_corner);

    /**
     * @brief recall points cloud of hover objects
     *
     * @param frame lidar frame
     * @param roi_rect roi rectanguler box
     * @param box_corner roi box corner
     * @param roi_cloud roi point cloud in local coordinate system
     * @param roi_world_cloud roi point cloud in world coordinate system
     */
    void RecallPointsByHoverObjects(
            const lidar::LidarFrame* frame,
            const std::vector<double>& roi_rect,
            const std::vector<double>& box_corner,
            std::vector<base::PointF>& roi_cloud,
            std::vector<base::PointD>& roi_world_cloud);

    /**
     * @brief check whether the object hovers or not
     *
     * @param object object
     * @param box_corner box corner
     * @return true
     * @return false
     */
    bool HoverDetect(const std::shared_ptr<base::Object>& object, 
                     const std::vector<double>& box_corner);

private:
    StraightPoleBarrierConfig barrier_config_;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::perception::lidar::StraightPoleRecognizer, BaseBarrierRecognizer)

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
