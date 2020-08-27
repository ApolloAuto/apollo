/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#include <string>

#include "cyber/common/file.h"
#include "cyber/record/record_reader.h"
#include "modules/audio/proto/audio_conf.pb.h"
#include "modules/audio/common/message_process.h"

namespace apollo {
namespace audio {

using apollo::cyber::record::RecordReader;
using apollo::cyber::record::RecordMessage;
using apollo::drivers::microphone::config::AudioData;

void ProcessOfflineData(const AudioConf& audio_conf,
    const std::string& record_filepath,
    AudioInfo* audio_info,
    DirectionDetection* direction_detection,
    MovingDetection* moving_detection,
    SirenDetection* siren_detection) {
  RecordReader reader(record_filepath);
  RecordMessage message;
  while (reader.ReadMessage(&message)) {
    if (message.channel_name ==
        audio_conf.topic_conf().audio_data_topic_name()) {
      AudioData audio_data;
      if (audio_data.ParseFromString(message.content)) {
        AudioDetection audio_detection;
        std::string respeaker_extrinsics_file =
            audio_conf.respeaker_extrinsics_path();
        MessageProcess::OnMicrophone(audio_data, respeaker_extrinsics_file,
            audio_info, direction_detection, moving_detection, siren_detection,
            &audio_detection);
        AINFO << audio_detection.ShortDebugString();
      }
    }
  }
}

void Profiling() {
  // TODO(kechxu) implement
}

}  // namespace audio
}  // namespace apollo

int main(int argc, char* argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  apollo::audio::Profiling();
  return 0;
}
