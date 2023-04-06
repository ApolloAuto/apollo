/******************************************************************************
 * Copyright 2022 The Apollo Authors. All Rights Reserved.
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

#pragma once

#include <string>
#include <vector>

#include "opencv2/opencv.hpp"

#include "modules/perception/base/distortion_model.h"
#include "modules/perception/base/object_types.h"
#include "modules/perception/camera/common/camera_frame.h"


namespace apollo {
namespace perception {
namespace camera {

/**
 * @brief Fill camera KMatrix from file
 *
 * @param frame Camera frame object
 * @param file_name Camera intrinsic file
 * @return true
 * @return false
 */
bool FillCameraKMatrix(CameraFrame* frame, const std::string& file_name);

/**
 * @brief Read image from file_name and fill in frame->data_provider
 *
 * @param frame CameraFrame
 * @param file_name image file
 * @return true
 * @return false
 */
bool FillImage(CameraFrame* frame, const std::string& file_name);

/**
 * @brief Recovery cv image from frame->data_provider
 *
 * @param frame CameraFrame
 * @param cv_img cv image
 * @return true
 * @return false
 */
bool RecoveryImage(CameraFrame* frame, cv::Mat* cv_img);

/**
 * @brief Get the File List From Path object
 *
 * @param file_path A file path containing all filenames
 * @param file_list Array of output filenames
 * @return true success
 * @return false fail
 */
bool GetFileListFromPath(
    const std::string& file_path, std::vector<std::string>* file_list);

/**
 * @brief Get the File List From File object
 *
 * @param file_name A file containing all filenames
 * @param file_list Array of output filenames
 * @return true success
 * @return false fail
 */
bool GetFileListFromFile(
    const std::string& file_name, std::vector<std::string>* file_list);

/**
 * @brief Save detection result to file_name
 *
 * @param frame CameraFrame
 * @param file_name result file
 * @return true
 * @return false
 */
bool SaveCameraDetectionResult(CameraFrame* frame,
                               const std::string& file_name);

bool SaveTfDetectionResult(CameraFrame* frame, const std::string& file_name);

bool SaveLaneDetectionResult(CameraFrame* frame, const std::string& file_name);

}  // namespace camera
}  // namespace perception
}  // namespace apollo
