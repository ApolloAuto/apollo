/******************************************************************************
* Copyright 2018 The Apollo Authors. All Rights Reserved.
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
#include "modules/perception/camera/lib/obstacle/tracker/common/similar.h"

#include <cblas.h>
#include <memory>

#include "cyber/common/log.h"
#include "modules/perception/camera/common/util.h"
#include "modules/perception/inference/utils/gemm.h"

namespace apollo {
namespace perception {
namespace camera {

bool GPUSimilar::Calc(CameraFrame *frame1,
                      CameraFrame *frame2,
                      base::Blob<float> *sim) {
  int n = static_cast<int>(frame1->detected_objects.size());
  int m = static_cast<int>(frame2->detected_objects.size());
  if ((n && m) == 0) {
    return false;
  }
  sim->Reshape({n, m});

  if (frame1->track_feature_blob == nullptr) {
    AERROR << "No feature blob";
    return false;
  }
  int dim = frame1->track_feature_blob->count(1);
  assert(dim == frame2->track_feature_blob->count(1));

  float *s = sim->mutable_gpu_data();
  float const *feature1 = frame1->track_feature_blob->gpu_data();
  float const *feature2 = frame2->track_feature_blob->gpu_data();
  inference::GPUGemmFloat(CblasNoTrans,
               CblasTrans,
               n,
               m,
               dim,
               1.0,
               feature1,
               feature2,
               0.0,
               s);
  return true;
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
