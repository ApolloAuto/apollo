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

#pragma once

#include <memory>
#include <string>

#include <google/protobuf/message.h>
#include <portaudio.h>
#include "cyber/cyber.h"
#include "modules/drivers/microphone/proto/microphone_config.pb.h"

namespace apollo {
namespace drivers {
namespace microphone {

using apollo::drivers::microphone::config::MicrophoneConfig;

/* An incomplete version of PA Stream used only for respeaker as input.
 * Refer to http://portaudio.com/docs/v19-doxydocs/ for more information
 */
class Stream {
 private:
  PaStream *pastream_ptr_;
  PaStreamParameters *input_parameters_ptr_;

 public:
  Stream() {}
  ~Stream();
  void init_stream(int rate, int channels, int chunk, int input_device_index,
                   PaSampleFormat format);
  void read_stream(int n_frames, char *buffer) const;
};

class Respeaker {
 private:
  std::unique_ptr<Stream> stream_ptr_;
  const PaDeviceInfo *get_device_info(const PaDeviceIndex index) const;
  const PaDeviceIndex host_api_device_index_to_device_index(
      const PaHostApiIndex host_api, const int host_api_device_index) const;
  const PaHostApiInfo *get_host_api_info(const PaHostApiIndex index) const;
  const PaDeviceIndex get_respeaker_index() const;
  const PaSampleFormat get_format_from_width(int width,
                                             bool is_unsigned = true) const;

 public:
  Respeaker() {}
  ~Respeaker();
  void init(const std::shared_ptr<const MicrophoneConfig> &microphone_config);
  void read_stream(int n_frames, char *buffer) const;
};

}  // namespace microphone
}  // namespace drivers
}  // namespace apollo
