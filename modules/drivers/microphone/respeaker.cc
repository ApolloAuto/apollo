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

#include "modules/drivers/microphone/respeaker.h"

namespace apollo {
namespace drivers {
namespace microphone {

PaError err;

// Helper functions
void report_error(PaError err, const std::string &func_name) {
  AERROR << "an error occured while calling " << func_name;
  AERROR << "error number: " << err;
  AERROR << "error message: " << Pa_GetErrorText(err);
}

// Stream
Stream::~Stream() {
  Pa_CloseStream(pastream_ptr_);
  free(input_parameters_ptr_);
}

void Stream::init_stream(int rate, int channels, int chunk,
                         int input_device_index, PaSampleFormat format) {
  // Init parameters of input device
  input_parameters_ptr_ = new PaStreamParameters;
  input_parameters_ptr_->device = input_device_index;
  input_parameters_ptr_->channelCount = channels;
  input_parameters_ptr_->sampleFormat = format;
  input_parameters_ptr_->suggestedLatency =
      Pa_GetDeviceInfo(input_parameters_ptr_->device)->defaultLowInputLatency;
  input_parameters_ptr_->hostApiSpecificStreamInfo = nullptr;

  err = Pa_OpenStream(
      &pastream_ptr_, input_parameters_ptr_, nullptr, rate, chunk,
      paClipOff,  // we only use input so don't bother clipping them *
      nullptr, nullptr);
  if (err != paNoError) {
    report_error(err, "Pa_OpenStream");
  }
  err = Pa_StartStream(pastream_ptr_);
  if (err != paNoError) {
    report_error(err, "Pa_StartStream");
  }
}

void Stream::read_stream(int n_frames, char *buffer) const {
  err =
      Pa_ReadStream(pastream_ptr_, reinterpret_cast<void *>(buffer), n_frames);
  if (err != paNoError) {
    report_error(err, "Pa_ReadStream");
    throw std::runtime_error("");
  }
}

// Respeaker
Respeaker::~Respeaker() { Pa_Terminate(); }
void Respeaker::init(
    const std::shared_ptr<const MicrophoneConfig> &microphone_config) {
  if (microphone_config->microphone_model() != MicrophoneConfig::RESPEAKER) {
    AERROR << "Microphone driver only supports respeaker model in config file";
  }
  err = Pa_Initialize();
  if (err != paNoError) {
    Pa_Terminate();
    report_error(err, "Pa_Initialize");
  }

  const PaDeviceIndex device_index = get_respeaker_index();
  stream_ptr_.reset(new Stream());
  stream_ptr_->init_stream(
      microphone_config->sample_rate(), microphone_config->channel_type_size(),
      microphone_config->chunk(), device_index,
      get_format_from_width(microphone_config->sample_width()));
}

const PaSampleFormat Respeaker::get_format_from_width(int width,
                                                      bool is_unsigned) const {
  switch (width) {
    case 1:
      return is_unsigned ? paUInt8 : paInt8;
    case 2:
      return paInt16;
    case 3:
      return paInt24;
    case 4:
      return paFloat32;
    default:
      break;
  }
  AERROR << "invalid width: " << width;
  return -1;
}

const PaDeviceIndex Respeaker::get_respeaker_index() const {
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
  return -1;
}

const PaDeviceInfo *Respeaker::get_device_info(
    const PaDeviceIndex index) const {
  const PaDeviceInfo *device_info =
      reinterpret_cast<const PaDeviceInfo *>(Pa_GetDeviceInfo(index));
  if (!device_info) {
    AERROR << "internal error: invalid device index" << index;
  }

  return device_info;
}

const PaDeviceIndex Respeaker::host_api_device_index_to_device_index(
    const PaHostApiIndex host_api, const int host_api_device_index) const {
  // Get standard device index from host-API-specific device index
  PaDeviceIndex device_index =
      Pa_HostApiDeviceIndexToDeviceIndex(host_api, host_api_device_index);
  if (device_index < 0) {
    report_error(device_index, "Pa_HostApiDeviceIndexToDeviceIndex");
  }
  return device_index;
}

const PaHostApiInfo *Respeaker::get_host_api_info(
    const PaHostApiIndex index) const {
  // Get host api info by it's index
  const PaHostApiInfo *pa_host_api_info =
      reinterpret_cast<const PaHostApiInfo *>(Pa_GetHostApiInfo(index));
  if (!pa_host_api_info) {
    AERROR << "internal error: invalid Host Api Index " << index;
  }
  return pa_host_api_info;
}

void Respeaker::read_stream(int n_frames, char *buffer) const {
  stream_ptr_->read_stream(n_frames, buffer);
}

}  // namespace microphone
}  // namespace drivers
}  // namespace apollo
