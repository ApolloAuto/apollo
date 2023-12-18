/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "modules/perception/camera_detection_single_stage/detector/caddn/postprocess.h"

#include "cyber/common/log.h"
#include "modules/perception/common/base/box.h"

namespace apollo {
namespace perception {
namespace camera {

void GetCaddnObjects(std::vector<base::ObjectPtr> *objects,
                     const caddn::ModelParam &model_param,
                     const std::vector<base::ObjectSubType> &types,
                     const base::BlobPtr<float> &boxes,
                     const base::BlobPtr<float> &labels,
                     const base::BlobPtr<float> &scores) {
  objects->clear();
  const float *boxes_data = boxes->cpu_data();
  const float *labels_data = labels->cpu_data();
  const float *scores_data = scores->cpu_data();

  int len_pred = 7;
  for (int i = 0; i < labels->num(); ++i) {
    const float *bbox = boxes_data + i * len_pred;
    float score = *(scores_data + i);
    if (score < model_param.score_threshold()) {
      continue;
    }

    float label = *(labels_data + i);
    base::ObjectPtr obj = nullptr;
    obj.reset(new base::Object);
    obj->sub_type = GetSubtype(label, types);
    obj->type = base::kSubType2TypeMap.at(obj->sub_type);
    obj->type_probs.assign(static_cast<int>(base::ObjectType::MAX_OBJECT_TYPE),
                           0);
    obj->sub_type_probs.assign(
        static_cast<int>(base::ObjectSubType::MAX_OBJECT_TYPE), 0);
    obj->type_probs[static_cast<int>(obj->type)] = score;
    obj->sub_type_probs[static_cast<int>(obj->sub_type)] = score;
    obj->confidence = score;

    // todo(zero): Need to be converted to camera coordinates and
    // converted to KITTI coordinates
    Eigen::Matrix<float, 3, 4> velo_to_cam;
    velo_to_cam << 0.00753374, -0.9999714, -0.0006166, -0.00406977, 0.01480249,
        0.00072807, -0.9998902, -0.07631618, 0.9998621, 0.00752379, 0.01480755,
        -0.2717806;

    Eigen::Matrix<float, 3, 3> R;
    R << 0.9999239, 0.00983776, -0.00744505, -0.0098698, 0.9999421, -0.00427846,
        0.00740253, 0.00435161, 0.9999631;

    std::vector<float> test_result(7);
    Bbox3dLidar2Camera(velo_to_cam, R, bbox, &test_result);

    FillCaddnBbox3d(obj, test_result.data());

    objects->push_back(obj);
  }
}

void Bbox3dLidar2Camera(const Eigen::Matrix<float, 3, 4> &V2C,
                        const Eigen::Matrix<float, 3, 3> &R,
                        const float *bbox_lidar,
                        std::vector<float> *bbox_camera) {
  float x = *(bbox_lidar + 0);
  float y = *(bbox_lidar + 1);
  float z = *(bbox_lidar + 2);
  float l = *(bbox_lidar + 3);
  float w = *(bbox_lidar + 4);
  float h = *(bbox_lidar + 5);
  float r = *(bbox_lidar + 6);
  r = -r - M_PI / 2.0;
  // 4*1
  Eigen::Vector4f xyz_lidar;
  xyz_lidar << x, y, z - h / 2.0, 1.0;
  Eigen::Matrix<float, 1, 3> pts_rect =
      xyz_lidar.transpose() * (V2C.transpose() * R.transpose());
  std::vector<float> final_result{
      pts_rect(0), pts_rect(1), pts_rect(2), l, h, w, r};
  bbox_camera->assign(final_result.data(),
                      final_result.data() + final_result.size());
}

base::ObjectSubType GetSubtype(int cls,
                               const std::vector<base::ObjectSubType> &types) {
  if (cls < 0 || cls >= static_cast<int>(types.size())) {
    return base::ObjectSubType::UNKNOWN;
  }

  return types[cls];
}

void FillCaddnBbox3d(base::ObjectPtr obj, const float *bbox) {
  obj->camera_supplement.alpha = bbox[6];
  obj->size[2] = bbox[3];
  obj->size[1] = bbox[4];
  obj->size[0] = bbox[5];

  obj->camera_supplement.local_center[0] = bbox[0];
  obj->camera_supplement.local_center[1] = bbox[1];
  obj->camera_supplement.local_center[2] = bbox[2];
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
