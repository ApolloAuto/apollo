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

#pragma once

#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "modules/perception/common/base/point.h"
#include "modules/perception/common/base/point_cloud.h"

namespace apollo {
namespace perception {
namespace base {

struct BoundingCube {
  float x;  // center of box
  float y;  // center of box
  float z;  // center of box
  float length;
  float width;
  float height;
  float yaw;
  float trans_x;  // center of points
  float trans_y;  // center of points
  float trans_z;  // center of points
};

bool DownSamplePointCloudBeams(base::PointFCloudPtr cloud_ptr,
                               base::PointFCloudPtr out_cloud_ptr,
                               int downsample_factor);

bool GetPointCloudMinareaBbox(const base::PointFCloud& pc, BoundingCube* box,
                              const int& min_num_points = 5,
                              const bool& verbose = false);

void CloudDemean(base::PointFCloudPtr cloud);

void GetPointCloudCentroid(base::PointFCloudConstPtr cloud, PointF* centroid);

double OrientCloud(const PointFCloud& pc, PointFCloud* pc_out, bool demean);

}  // namespace base
}  // namespace perception
}  // namespace apollo
