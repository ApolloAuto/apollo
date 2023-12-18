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

#include <memory>
#include <vector>
#include <nppi.h>

#include "opencv2/opencv.hpp"

#include "cyber/base/concurrent_object_pool.h"

using apollo::cyber::base::CCObjectPool;

namespace apollo {
namespace prediction {

class AffineTransform {
 public:
  AffineTransform();
  ~AffineTransform();

  /**
   * @brief init function to the class
   * @param size size of the image
   * @param type the image format type
   */
  bool Init(cv::Size size, int type);

  /**
   * @brief calculate the coeffs martix
   * @param heading the heading angle
   * @param center_point rotation point of the image
   * @param scale the scale of affine transform
   * @param coeffs output coeffs martix
   */
  int GetCoeffs(const double heading,
              const cv::Point2i& center_point,
              const double scale,
              double coeffs[2][3]);

  /**
   * @brief perform the affine transform from cv mat
   * @param input_img the input of image
   * @param center_point rotation point of the image
   * @param heading the heading angle
   * @param scale the scale of affine transform
   * @param output_img output buffer
   */
  int AffineTransformsFromMat(const cv::Mat& input_img,
                              const cv::Point2i& center_point,
                              const double heading,
                              const double scale,
                              cv::Mat* output_img);

 private:
  std::shared_ptr<CCObjectPool<unsigned char*>> src_pool_;
  std::shared_ptr<CCObjectPool<unsigned char*>> out_pool_;
  std::shared_ptr<CCObjectPool<NppStreamContext>> stream_pool_;
  std::vector<void*> pointer_table_;
  cv::Size size_;
  int type_;
};

}  // namespace prediction
}  // namespace apollo

