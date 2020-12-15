/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/monitor/software/camera_monitor.h"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/strings/str_cat.h"
#include "google/protobuf/compiler/parser.h"
#include "google/protobuf/descriptor.h"
#include "google/protobuf/dynamic_message.h"

#include "modules/drivers/proto/sensor_image.pb.h"

#include "cyber/common/log.h"
#include "cyber/cyber.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/map_util.h"
#include "modules/monitor/common/monitor_manager.h"
#include "modules/monitor/software/summary_monitor.h"

DEFINE_string(camera_monitor_name, "CameraMonitor",
              "Name of the camera monitor.");
DEFINE_double(camera_monitor_interval, 5,
              "Camera monitor checking interval in seconds.");
DEFINE_string(camera_component_name, "camera",
              "Camera component name.");

namespace apollo {
namespace monitor {
namespace {

using ReaderAndMessagePair =
    std::pair<std::shared_ptr<cyber::ReaderBase>,
              std::shared_ptr<google::protobuf::Message>>;

template <typename T>
ReaderAndMessagePair CreateReaderAndLatestsMessage(const std::string& camera) {
  const auto reader = MonitorManager::Instance()->CreateReader<T>(camera);
  reader->Observe();
  const auto message = reader->GetLatestObserved();
  return {reader, message};
}

static const auto camera_function_map =
      std::unordered_map<std::string, std::function<ReaderAndMessagePair(
                                          const std::string& camera)>>{
          {FLAGS_image_front_topic,
           &CreateReaderAndLatestsMessage<drivers::Image>},
          {FLAGS_image_short_topic,
           &CreateReaderAndLatestsMessage<drivers::Image>},
          {FLAGS_image_long_topic,
           &CreateReaderAndLatestsMessage<drivers::Image>},
          {FLAGS_camera_image_long_topic,
           &CreateReaderAndLatestsMessage<drivers::Image>},
          {FLAGS_camera_image_short_topic,
           &CreateReaderAndLatestsMessage<drivers::Image>},
          {FLAGS_camera_front_6mm_compressed_topic,
           &CreateReaderAndLatestsMessage<drivers::CompressedImage>},
          {FLAGS_camera_front_12mm_compressed_topic,
           &CreateReaderAndLatestsMessage<drivers::CompressedImage>},
          {FLAGS_camera_left_fisheye_compressed_topic,
           &CreateReaderAndLatestsMessage<drivers::CompressedImage>},
          {FLAGS_camera_right_fisheye_compressed_topic,
           &CreateReaderAndLatestsMessage<drivers::CompressedImage>},
          {FLAGS_camera_rear_6mm_compressed_topic,
           &CreateReaderAndLatestsMessage<drivers::CompressedImage>},
          {FLAGS_camera_front_6mm_video_compressed_topic,
           &CreateReaderAndLatestsMessage<drivers::CompressedImage>},
          {FLAGS_camera_front_12mm_video_compressed_topic,
           &CreateReaderAndLatestsMessage<drivers::CompressedImage>},
          {FLAGS_camera_left_fisheye_video_compressed_topic,
           &CreateReaderAndLatestsMessage<drivers::CompressedImage>},
          {FLAGS_camera_right_fisheye_video_compressed_topic,
           &CreateReaderAndLatestsMessage<drivers::CompressedImage>},
          {FLAGS_camera_rear_6mm_video_compressed_topic,
           &CreateReaderAndLatestsMessage<drivers::CompressedImage>},
          // Add more cameras here if you want to monitor.
      };


}  // namespace

CameraMonitor::CameraMonitor()
    : RecurrentRunner(FLAGS_camera_monitor_name, 
                      FLAGS_camera_monitor_interval){

                        AINFO << "step into Camera Initial";
                      }

void CameraMonitor::RunOnce(const double current_time) {
  AINFO << "step into Camera RunOnce";
  auto manager = MonitorManager::Instance();
  auto* component = apollo::common::util::FindOrNull(
      *manager->GetStatus()->mutable_components(),
      FLAGS_camera_component_name);
  auto* status = component->mutable_other_status();
  UpdateStatus(status);

}

void UpdateStatus(ComponentStatus* status) {
  AINFO << "step into Camera update status";
  status->clear_status();
  std::string frame_id;
  for (auto iter = camera_function_map.begin(); iter != camera_function_map.end(); ++iter){
    const auto camera_type = iter->first;
    AINFO << "camera_type" << camera_type;
    const auto reader_message_pair = (iter->second)(camera_type);
    const auto reader = reader_message_pair.first;
    const auto message = reader_message_pair.second;
    if (reader != nullptr) {
        if (frame_id.empty()){
          const google::protobuf::Descriptor* desc = (*message).GetDescriptor();
          const google::protobuf::Reflection* ref = (*message).GetReflection();
          const auto field_count = desc->field_count();
          for (int field_idx = 0; field_idx < field_count; ++field_idx) {
            const auto* field_desc = desc->field(field_idx);
            if (field_desc->name() == "frame_id") {
              frame_id = ref->GetString((*message), field_desc);
            }
          }
        }
        else{
          SummaryMonitor::EscalateStatus(
            ComponentStatus::ERROR,
            absl::StrCat("only one camera is permitted"), status);
        }
    }
  }  
  if(frame_id.empty()){
    SummaryMonitor::EscalateStatus(
            ComponentStatus::ERROR,
            absl::StrCat("no camera is detected"), status);
  }
  else
  {
    SummaryMonitor::EscalateStatus(
            ComponentStatus::OK,
            absl::StrCat("detected one camera: ", frame_id), status);
  }

}

}  // namespace monitor
}  // namespace apollo
