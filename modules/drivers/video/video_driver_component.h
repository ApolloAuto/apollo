/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include <memory>
#include <string>
#include <thread>

#include "cyber/base/concurrent_object_pool.h"
#include "cyber/cyber.h"
#include "modules/drivers/video/driver.h"
#include "modules/drivers/video/proto/video_h265cfg.pb.h"
 #include "modules/drivers/proto/sensor_image.pb.h"


namespace apollo {
namespace drivers {
namespace video {


using apollo::cyber::Writer;
using apollo::cyber::Component;
using apollo::cyber::base::CCObjectPool;
using apollo::drivers::CompressedImage;
using apollo::drivers::video::config::CameraH265Config ;

class CompCameraH265Compressed : public Component<>{
public:
	~CompCameraH265Compressed()
	{
    if (video_thread_->joinable()) {
      video_thread_->join();
    }
  }
  bool Init() {
    AINFO << "video driver component init";
    CameraH265Config  video_config;
    if (!GetProtoConfig(&video_config)) {
      return false;
    }

    AINFO << "Velodyne config: " << video_config.DebugString();

    _camera_device.reset(new CameraDriver(video_config));
    _camera_device->Init();

    if (_camera_device->Record()) {
      char * folder  = getenv("H265_SAVE_FOLDER");
      struct stat st = {0};
      if (folder) {
        _record_folder = folder;
      } else {
        AINFO << 
          "hasn't find environment H265_SAVE_FOLDER, just used current directory to save record file";
        _record_folder = ".";
      }
      AINFO << "record_folder is " << _record_folder;
      if (stat(_record_folder.c_str(), &st) == -1) {
        char cmd[256];
        sprintf(cmd, "mkdir -p %s", _record_folder.c_str());
        int ret = system(cmd);
        if (ret == 0) {
          AINFO << "cmd " << " cmd " << " execute sucessfuly";
        }
      }
    }
    _pb_image.reset(new CompressedImage);
    _pb_image->mutable_data()->reserve(1920*1080*4);

    writer_ = node_->CreateWriter<CompressedImage>(
        video_config.compress_conf().output_channel());


    runing_ = true;
    video_thread_ = std::shared_ptr<std::thread>(
        new std::thread(std::bind(&CompCameraH265Compressed::video_poll, this)));
    video_thread_->detach();

    return true;
  }

private:
  void video_poll() {
    std::ofstream fout;
    if (_camera_device->Record()) {
      char name[256];
      sprintf(name, "%s/encode_%d.h265", _record_folder.c_str(), _camera_device->Port());
      AINFO << "output name " << name;
      fout.open(name, std::ios::binary);
      if (!fout) AERROR << "open " << name << "  fail";
    }
    while (!apollo::cyber::IsShutdown()) {
      if (_camera_device->poll(_pb_image)) {
        _pb_image->mutable_header()->set_timestamp_sec(cyber::Time::Now().ToSecond());
        AINFO << "Send CompressedImage";
        writer_->Write(_pb_image);
      } else {
        AERROR << "port " << _camera_device->Port() << " h265 poll fail....";
        continue;
      }

      if (_camera_device->Record()) {
        fout.write((char*)_pb_image->data().c_str(), _pb_image->data().size());
      }

    }

    if (_camera_device->Record()) {
      fout.close();
    }
  }
  std::shared_ptr<apollo::cyber::Writer<CompressedImage>> writer_;
  std::shared_ptr<std::thread> video_thread_;
  volatile bool runing_; 
  std::unique_ptr<CameraDriver> _camera_device;
  std::string _record_folder;
  std::shared_ptr<CompressedImage> _pb_image = nullptr;
};

CYBER_REGISTER_COMPONENT(CompCameraH265Compressed);

} //video
} //drivers
} //apollo


