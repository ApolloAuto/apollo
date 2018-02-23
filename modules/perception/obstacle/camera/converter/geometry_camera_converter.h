/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

// Convert 2D detections into 3D objects

#ifndef MODULES_PERCEPTION_OBSTACLE_CAMERA_CONVERTER_GEOMETRY_H_
#define MODULES_PERCEPTION_OBSTACLE_CAMERA_CONVERTER_GEOMETRY_H_

#include <cmath>
#include <limits>
#include <string>
#include <vector>
#include <algorithm>
#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>

#include "xlog.h"
#include "lib/config_manager/config_manager.h"

#include "obstacle/common/camera.h"
#include "obstacle/camera/common/util.h"
#include "obstacle/camera/interface/base_camera_transformer.h"

namespace apollo {
namespace perception {

class GeometryCameraConverter : public BaseCameraConverter {
public:

    GeometryCameraConverter() : BaseCameraConverter() {}

    virtual ~GeometryCameraConverter() {}

    virtual bool Init() override;

   // @brief: Convert 2D detected objects into physical 3D objects
   // @param [in/out]: detected object lists, added 3D position and orientation
   virtual bool Convert(std::vector<VisualObjectPtr>* objects) override;

    void SetDebug(bool flag);

    virtual std::string name() const override;

private:

  bool instance_transform(const float &h, const float &w, const float &l, const float &alpha_deg,
                          const Eigen::Vector2f &upper_left, const Eigen::Vector2f &lower_right,
                          float &distance_w, float &distance_h,
                          Eigen::Vector2f &mass_center_pixel);

    bool load_camera_intrinsics(const std::string &file_name);

    void rotate_object(float alpha_deg, std::vector<Eigen::Vector3f > &corners) const;

    float distance_binary_search(const int &target_pixel_length, const bool &use_width,
                                  const Eigen::Matrix<float, 3, 1> &mass_center_v) const;

    void mass_center_search(const Eigen::Matrix<float, 2, 1> &target_box_center_pixel,
                            const double &curr_d, Eigen::Matrix<float, 3, 1> &mass_center_v,
                            Eigen::Matrix<float, 2, 1> &mass_center_pixel) const;

    Eigen::Matrix<float, 3, 1> to_unit_v(const Eigen::Matrix<float, 3, 1> &v) const;

    Camera<float> camera_model_;
    std::vector<Eigen::Vector3f> _corners_;
    static const int max_distance_binary_search_depth = 20;
    static const int max_mass_center_search_depth = 10;
    bool debug_ = false;
};

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_CAMERA_CONVERTER_GEOMETRY_H_
