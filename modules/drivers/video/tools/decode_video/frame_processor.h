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

#pragma once

#include <string>
#include <vector>

namespace apollo {
namespace drivers {
namespace video {

/**
 * @class FrameProcessor
 * @brief FrameProcessor is a class to process video streams.
 */
class FrameProcessor {
 public:
  // Constructor
  FrameProcessor(const std::string& input_video_file,
                 const std::string& output_jpg_dir);

  // Process frames according to input data, and output converted data
  bool ProcessStream() const;

  // Destructor
  ~FrameProcessor() = default;

 private:
  void WriteOutputJpgFile(const std::vector<uint8_t>& jpeg_buffer,
                          const std::string& output_jpg_file) const;
  std::string GetOutputFile(const int frame_num) const;

  std::vector<uint8_t> input_video_buffer_;
  const std::string output_jpg_dir_;
};

}  // namespace video
}  // namespace drivers
}  // namespace apollo
