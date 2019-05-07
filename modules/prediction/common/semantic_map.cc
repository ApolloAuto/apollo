/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/prediction/common/semantic_map.h"

#include <string>

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/common/util/string_util.h"
#include "modules/prediction/container/container_manager.h"
#include "modules/prediction/container/pose/pose_container.h"

namespace apollo {
namespace prediction {

SemanticMap::SemanticMap() {}

void SemanticMap::Init() {
  const std::string file_path =
      apollo::common::util::StrCat(FLAGS_map_dir, "/base_map.png");
  if (cyber::common::PathExists(file_path)) {
    base_img_ = cv::imread(file_path, CV_LOAD_IMAGE_COLOR);
  }
  curr_img_ = cv::Mat(2000, 2000, CV_8UC3, cv::Scalar(0, 0, 0));
}

void SemanticMap::RunCurrFrame(const FrameEnv& curr_frame_env) {
  // TODO(Hongyi): moving all these magic numbers to conf
  curr_base_x_ = curr_frame_env.ego_history().feature(0).position().x() - 50.0;
  curr_base_y_ = curr_frame_env.ego_history().feature(0).position().y() - 50.0;
  cv::Rect rect(static_cast<int>((curr_base_x_ - 585950.0) / 0.1),
            static_cast<int>(18000 - (curr_base_y_ - 4140000.0) / 0.1) - 1000,
            1000, 1000);
  curr_img_ = base_img_(rect);
  cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE);
  cv::imshow("Display window", curr_img_);
  cv::waitKey(0);
}

}  // namespace prediction
}  // namespace apollo
