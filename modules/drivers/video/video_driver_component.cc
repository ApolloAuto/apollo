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

#include "modules/drivers/video/video_driver_component.h"

namespace apollo {
namespace drivers {
namespace video {


bool CompCameraH265Compressed::Init() {
  AINFO << "video driver component init";
  CameraH265Config  video_config;
  if (!GetProtoConfig(&video_config)) {
    return false;
  }

  AINFO << "Velodyne config: " << video_config.DebugString();

  camera_deivce_.reset(new CameraDriver(&video_config));
  camera_deivce_->Init();

  if (camera_deivce_->Record()) {
    char * folder  = getenv("H265_SAVE_FOLDER");
    struct stat st = {0};
    if (folder) {
      record_folder_ = folder;
    } else {
      AINFO <<
        "hasn't find environment H265_SAVE_FOLDER, " <<
        "just used current directory to save record file";
      record_folder_ = ".";
    }
    AINFO << "record_folder is " << record_folder_;
    if (stat(record_folder_.c_str(), &st) == -1) {
      char cmd[256];
      snprintf(cmd, sizeof(cmd), "mkdir -p %s", record_folder_.c_str());
      int ret = system(cmd);
      if (ret == 0) {
        AINFO << "cmd " << " cmd " << " execute sucessfuly";
      }
    }
  }
  pb_image_.reset(new CompressedImage);
  pb_image_->mutable_data()->reserve(1920*1080*4);

  writer_ = node_->CreateWriter<CompressedImage>(
      video_config.compress_conf().output_channel());


  runing_ = true;
  video_thread_ = std::shared_ptr<std::thread>(new std::thread(
        std::bind(&CompCameraH265Compressed::VideoPoll, this)));
  video_thread_->detach();

  return true;
}

void CompCameraH265Compressed::VideoPoll() {
  std::ofstream fout;
  if (camera_deivce_->Record()) {
    char name[256];
    snprintf(name, sizeof(name), "%s/encode_%d.h265",
        record_folder_.c_str(), camera_deivce_->Port());
    AINFO << "output name " << name;
    fout.open(name, std::ios::binary);
    if (!fout) AERROR << "open " << name << "  fail";
  }
  while (!apollo::cyber::IsShutdown()) {
    if (camera_deivce_->Poll(pb_image_)) {
      pb_image_->mutable_header()->set_timestamp_sec(
          cyber::Time::Now().ToSecond());
      AINFO << "Send CompressedImage";
      writer_->Write(pb_image_);
    } else {
      AERROR << "port " << camera_deivce_->Port() << " h265 poll fail....";
      continue;
    }

    if (camera_deivce_->Record()) {
      fout.write(pb_image_->data().c_str(), pb_image_->data().size());
    }
  }

  if (camera_deivce_->Record()) {
    fout.close();
  }
}

}  // namespace video
}  // namespace drivers
}  // namespace apollo
