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

#include "gflags/gflags.h"

#include "cyber/common/log.h"
#include "modules/drivers/video/tools/decode_video/frame_processor.h"

using apollo::drivers::video::FrameProcessor;

DECLARE_string(input_video);
DECLARE_string(output_dir);
DEFINE_string(input_video, "", "The input video file");
DEFINE_string(output_dir, "", "The directory to output decoded pictures.");

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  AINFO << "input video: " << FLAGS_input_video
        << ". output dir: " << FLAGS_output_dir;
  FrameProcessor processor(FLAGS_input_video, FLAGS_output_dir);
  if (!processor.ProcessStream()) {
    AERROR << "error: failed to decode file " << FLAGS_input_video;
    return -1;
  }
  return 0;
}
