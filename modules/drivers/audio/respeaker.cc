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

#include "modules/drivers/audio/respeaker.h"

namespace apollo {
namespace drivers {
namespace audio {

// Helper functions
void report_error(PaError err, const std::string &func_name) {
  AERROR << "an error occured while calling " << func_name;
  AERROR << "error number: " << err;
  AERROR << "error message: " << Pa_GetErrorText(err);
  throw std::runtime_error("");
}

// Stream
Stream::~Stream() { Pa_CloseStream(pastream_ptr_); }

void Stream::init_stream(int rate, int channels, int chunk,
                         int input_device_index, PaSampleFormat format) {
  PaError err;

  // Init parameters of input device
  inputParameters_ptr_ = reinterpret_cast<PaStreamParameters *>(
      malloc(sizeof(PaStreamParameters)));
  inputParameters_ptr_->device = input_device_index;
  inputParameters_ptr_->channelCount = channels;
  inputParameters_ptr_->sampleFormat = format;
  inputParameters_ptr_->suggestedLatency =
      Pa_GetDeviceInfo(inputParameters_ptr_->device)->defaultLowInputLatency;
  inputParameters_ptr_->hostApiSpecificStreamInfo = nullptr;

  err = Pa_OpenStream(
      &pastream_ptr_, inputParameters_ptr_, nullptr, rate, chunk,
      paClipOff,  // we only use input so don't bother clipping them *
      nullptr, nullptr);
  if (err != paNoError) report_error(err, "Pa_OpenStream");
  err = Pa_StartStream(pastream_ptr_);
  if (err != paNoError) report_error(err, "Pa_StartStream");
}

void Stream::read_stream(int n_frames, char *buffer) {
  int err =
      Pa_ReadStream(pastream_ptr_, reinterpret_cast<void *>(buffer), n_frames);
  if (err != paNoError) {
    report_error(err, "Pa_ReadStream");
  }
}

int Stream::get_chunk_size(int n_frames) {
  return (n_frames) * (inputParameters_ptr_->channelCount) *
         (Pa_GetSampleSize(inputParameters_ptr_->sampleFormat));
}

// Respeaker
Respeaker::~Respeaker() { Pa_Terminate(); }
void Respeaker::init(std::shared_ptr<SpeakerConfig> speaker_config) {
  if (speaker_config->speaker_model() != SpeakerConfig::RESPEAKER) {
    AERROR << "respeaker driver only supports respeaker model in config file";
  }
  int err = Pa_Initialize();
  if (err != paNoError) {
    Pa_Terminate();
    report_error(err, "Pa_Initialize");
  }

  PaDeviceIndex device_index = get_respeaker_index();
  stream_ptr_.reset(new Stream());
  stream_ptr_->init_stream(
      speaker_config->sample_rate(), speaker_config->channel_type_size(),
      speaker_config->chunk(), device_index,
      get_format_from_width(speaker_config->sample_width()));
}

PaSampleFormat Respeaker::get_format_from_width(int width, bool is_unsigned) {
  switch (width) {
    case 1:
      return is_unsigned ? paUInt8 : paInt8;
    case 2:
      return paInt16;
    case 3:
      return paInt24;
    case 4:
      return paFloat32;
  }
  AERROR << "invalid width: " << width;
  throw std::runtime_error("");
}

PaDeviceIndex Respeaker::get_respeaker_index() {
  // return index of respeaker
  const PaHostApiInfo *host_api_info = get_host_api_info(0);
  const PaDeviceInfo *device_info = nullptr;
  PaDeviceIndex real_index;
  for (PaDeviceIndex i = 0; i < host_api_info->deviceCount; ++i) {
    real_index = host_api_device_index_to_device_index(0, i);
    device_info = get_device_info(real_index);
    if (std::string(device_info->name).find("ReSpeaker") != std::string::npos) {
      return real_index;
    }
  }
  AERROR << "respeaker device not found";
  throw std::runtime_error("");
}

const PaDeviceInfo *Respeaker::get_device_info(PaDeviceIndex index) {
  const PaDeviceInfo *_info =
      reinterpret_cast<const PaDeviceInfo *>(Pa_GetDeviceInfo(index));
  if (!_info) {
    AERROR << "internal error: invalid device index" << index;
    throw std::runtime_error("");
  }

  return _info;
}

PaDeviceIndex Respeaker::host_api_device_index_to_device_index(
    PaHostApiIndex hostApi, int hostApiDeviceIndex) {
  // Get standard device index from host-API-specific device index
  PaDeviceIndex device_index =
      Pa_HostApiDeviceIndexToDeviceIndex(hostApi, hostApiDeviceIndex);
  if (device_index < 0) {
    report_error(device_index, "Pa_HostApiDeviceIndexToDeviceIndex");
  }
  return device_index;
}

const PaHostApiInfo *Respeaker::get_host_api_info(PaHostApiIndex index) {
  // Get host api info by it's index
  const PaHostApiInfo *_info =
      reinterpret_cast<const PaHostApiInfo *>(Pa_GetHostApiInfo(index));
  if (!_info) {
    AERROR << "internal error: invalid Host Api Index " << index;
    throw std::runtime_error("");
  }
  return _info;
}

}  // namespace audio
}  // namespace drivers
}  // namespace apollo
