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

#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>

#include "cyber/common/file.h"
#include "cyber/record/record_reader.h"
#include "cyber/record/record_writer.h"
#include "modules/audio/common/audio_gflags.h"
#include "modules/audio/common/message_process.h"
#include "modules/audio/proto/audio_conf.pb.h"
#include "modules/common_msgs/audio_msgs/audio_event.pb.h"
#include "modules/common_msgs/localization_msgs/localization.pb.h"
#include "modules/common_msgs/perception_msgs/perception_obstacle.pb.h"

namespace apollo {
namespace audio {

using apollo::cyber::record::RecordReader;
using apollo::cyber::record::RecordMessage;
using apollo::cyber::record::RecordWriter;
using apollo::drivers::microphone::config::AudioData;
using apollo::localization::LocalizationEstimate;
using apollo::perception::PerceptionObstacles;

void GetRecordFileNames(const boost::filesystem::path& p,
                        std::vector<std::string>* record_files) {
  if (!boost::filesystem::exists(p)) {
    return;
  }
  if (boost::filesystem::is_regular_file(p)) {
    AINFO << "Found record file: " << p.c_str();
    record_files->push_back(p.c_str());
    return;
  }
  if (boost::filesystem::is_directory(p)) {
    for (auto& entry : boost::make_iterator_range(
             boost::filesystem::directory_iterator(p), {})) {
      GetRecordFileNames(entry.path(), record_files);
    }
  }
}

void ProcessSingleRecordFile(const AudioConf& audio_conf,
    const std::string& input_record_filepath,
    const std::string& output_record_filepath,
    AudioInfo* audio_info,
    DirectionDetection* direction_detection,
    MovingDetection* moving_detection,
    SirenDetection* siren_detection) {
  RecordReader reader(input_record_filepath);
  RecordMessage message;
  RecordWriter writer;
  writer.Open(output_record_filepath);
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
        writer.WriteMessage<AudioDetection>(
            audio_conf.topic_conf().audio_detection_topic_name(),
            audio_detection, message.time);
        AINFO << "Generate a new audio detection message.";
      }
    } else if (message.channel_name ==
               audio_conf.topic_conf().audio_event_topic_name()) {
      AudioEvent audio_event;
      if (audio_event.ParseFromString(message.content)) {
        writer.WriteMessage<AudioEvent>(message.channel_name, audio_event,
                                        message.time);
        AINFO << "Save an audio even message.";
      }
    } else if (message.channel_name ==
                 audio_conf.topic_conf().localization_topic_name()) {
      LocalizationEstimate localization;
      if (localization.ParseFromString(message.content)) {
        writer.WriteMessage<LocalizationEstimate>(
            message.channel_name, localization, message.time);
        AINFO << "Save a localization message.";
      }
    } else if (message.channel_name ==
                 audio_conf.topic_conf().perception_topic_name()) {
      PerceptionObstacles perception_obstacles;
      if (perception_obstacles.ParseFromString(message.content)) {
        writer.WriteMessage<PerceptionObstacles>(
            message.channel_name, perception_obstacles, message.time);
        AINFO << "Save a perception message.";
      }
    }
  }
  writer.Close();
}

void ProcessFolder() {
  if (FLAGS_audio_records_dir.empty()) {
    AERROR << "The input folder is empty";
    return;
  }
  AudioConf audio_conf;
  if (!cyber::common::GetProtoFromFile(FLAGS_audio_conf_file,
                                       &audio_conf)) {
    return;
  }
  AudioInfo audio_info;
  DirectionDetection direction_detection;
  MovingDetection moving_detection;
  SirenDetection siren_detection;
  std::vector<std::string> offline_bags;
  GetRecordFileNames(boost::filesystem::path(FLAGS_audio_records_dir),
                     &offline_bags);
  std::sort(offline_bags.begin(), offline_bags.end());
  for (std::size_t i = 0; i < offline_bags.size(); ++i) {
    const std::string& input_record_filepath = offline_bags[i];
    std::string output_record_filepath =
        input_record_filepath + ".new_audio_detection";
    ProcessSingleRecordFile(audio_conf, input_record_filepath,
        output_record_filepath, &audio_info, &direction_detection,
        &moving_detection, &siren_detection);
  }
}

}  // namespace audio
}  // namespace apollo

int main(int argc, char* argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  apollo::audio::ProcessFolder();
  return 0;
}
