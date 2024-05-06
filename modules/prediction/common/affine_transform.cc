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

#include "modules/prediction/common/affine_transform.h"

#include <cmath>
#include <npp.h>
#include <nppdefs.h>
#include <cuda_runtime.h>

#include "cyber/common/log.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/common/prediction_system_gflags.h"

namespace apollo {
namespace prediction {

AffineTransform::AffineTransform() {}

AffineTransform::~AffineTransform() {
  for (auto it = pointer_table_.begin(); it != pointer_table_.end(); it++) {
    if (*it) {
      cudaFreeHost(*it);
    }
  }
}

bool AffineTransform::Init(cv::Size size, int type) {
  size_ = size;
  type_ = type;

  auto src_mat = cv::Mat(size_, type_, cv::Scalar(0, 0, 0));
  cv::Mat out_mat;
  cv::Point2i center = cv::Point2i(src_mat.cols / 2, src_mat.rows / 2);
  double angle = -90.0;
  double scale = 1.0;
  AffineTransformsFromMat(src_mat, center, angle, scale, &out_mat);

  return true;
}


int AffineTransform::GetCoeffs(const double heading,
                                const cv::Point2i& center_point,
                                const double scale,
                                double coeffs[2][3]) {
  double alpha, beta, angle;

  angle = heading * M_PI / 180;
  alpha = std::cos(angle) * scale;
  beta = std::sin(angle) * scale;

  *((*(coeffs+0))+0) = alpha;
  *((*(coeffs+0))+1) = beta;
  *((*(coeffs+0))+2) = (1 - alpha) * center_point.x - beta * center_point.y;
  *((*(coeffs+1))+0) = -beta;
  *((*(coeffs+1))+1) = alpha;
  *((*(coeffs+1))+2) = beta * center_point.x + (1 - alpha) * center_point.y;

  return 0;
}

int AffineTransform::AffineTransformsFromMat(const cv::Mat& input_img,
    const cv::Point2i& center_point, const double heading,
    const double scale, cv::Mat* output_img) {
  double coeffs[2][3];
  void* process_buff;
  void* output_buff;

  cudaMalloc(&process_buff, size_.width * size_.height * 3);
  cudaMalloc(&output_buff, size_.width * size_.height * 3);
  // auto output_buff = out_pool_->GetObject();
  // auto stream = stream_pool_->GetObject();

  // perform affine transform of the whole image
  NppiRect src_ROI{0, 0, input_img.size().width, input_img.size().height};
  NppiRect dst_ROI{0, 0, input_img.size().width, input_img.size().height};
  NppiSize image_size{input_img.size().width, input_img.size().height};

  if (GetCoeffs(heading, center_point, scale, coeffs)) {
    AERROR << "Fail to get rotation matrix";
    return -1;
  }

  cudaMemcpy(process_buff, (unsigned char*) input_img.data,
             input_img.size().width * input_img.size().height * 3,
             cudaMemcpyHostToDevice);

  NppStatus status = nppiWarpAffine_8u_C3R(
                      static_cast<Npp8u*>(process_buff),
                      image_size, input_img.step, src_ROI,
                      static_cast<Npp8u*>(output_buff),
                      input_img.step, dst_ROI, coeffs, NPPI_INTER_NN);
  if (status) {
    AERROR << "Affine transform failed, error: " << status;
    return -1;
  }

  *output_img = cv::Mat(input_img.size(),
            input_img.type(), cv::Scalar(0, 0, 0));

  cudaMemcpy((unsigned char*) output_img->data, output_buff,
             input_img.size().width * input_img.size().height * 3,
             cudaMemcpyDeviceToHost);

  cudaFree(process_buff);
  cudaFree(output_buff);

  return 0;
}

}  // namespace prediction
}  // namespace apollo
