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

#include <stdio.h>
#include <string>

#include "cyber/cyber.h"
#include "portaudio.h"

/* A Partial version of PA Stream used only for respeaker as input.
 * Refer to http://portaudio.com/docs/v19-doxydocs/
 */
class Stream {
 private:
  PaStream *pastream;
  PaStreamInfo *pastreamInfo;
  PaStreamParameters *inputParameters;

 public:
  Stream(){};
  void init_stream(int rate, int channels, int input_device_index,
                   PaSampleFormat format);
  void read_stream(int n_frames, char *buffer);
  int get_chunk_size(int n_frames);
};

class Respeaker {
 public:
  std::unique_ptr<Stream> stream_;

  Respeaker(){};
  void init(int sample_rate, int sample_width, int n_channels);
  PaSampleFormat get_format_from_width(int width, bool is_unsigned = true);
  PaDeviceIndex get_respeaker_index();
  const PaDeviceInfo *get_device_info(PaDeviceIndex index);
  PaDeviceIndex host_api_device_index_to_device_index(PaHostApiIndex hostApi,
                                                      int hostApiDeviceIndex);
  const PaHostApiInfo *get_host_api_info(PaHostApiIndex index);
};
