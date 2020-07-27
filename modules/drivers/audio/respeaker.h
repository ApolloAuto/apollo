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

#include <google/protobuf/message.h>
#include <portaudio.h>

#include <memory>
#include <string>

#include "cyber/cyber.h"
#include "modules/drivers/audio/proto/speaker_config.pb.h"

namespace apollo {
namespace drivers {
namespace audio {

using apollo::drivers::audio::config::SpeakerConfig;

/* An incomplete version of PA Stream used only for respeaker as input.
 * Refer to http://portaudio.com/docs/v19-doxydocs/ for more information
 */
class Stream {
 private:
  PaStream *pastream_ptr_;
  PaStreamParameters *inputParameters_ptr_;

 public:
  Stream() {}
  ~Stream();
  void init_stream(int rate, int channels, int chunk, int input_device_index,
                   PaSampleFormat format);
  void read_stream(int n_frames, char *buffer);
  int get_chunk_size(int n_frames);
};

class Respeaker {
 public:
  std::unique_ptr<Stream> stream_ptr_;

  Respeaker() {}
  ~Respeaker();
  void init(std::shared_ptr<SpeakerConfig> speaker_config);
  PaSampleFormat get_format_from_width(int width, bool is_unsigned = true);
  PaDeviceIndex get_respeaker_index();
  const PaDeviceInfo *get_device_info(PaDeviceIndex index);
  PaDeviceIndex host_api_device_index_to_device_index(PaHostApiIndex hostApi,
                                                      int hostApiDeviceIndex);
  const PaHostApiInfo *get_host_api_info(PaHostApiIndex index);
};

}  // namespace audio
}  // namespace drivers
}  // namespace apollo
