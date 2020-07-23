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

#define DEFAULT_FRAMES_PER_BUFFER 1024

// Helper functions
void report_error_and_throw(PaError err) {
  AERROR << "An error occured while using the portaudio stream";
  AERROR << "Error number: " << err;
  AERROR << "Error message: " << Pa_GetErrorText(err);
  throw std::runtime_error("");
}

// Stream
void Stream::init_stream(int rate, int channels, int input_device_index,
                         PaSampleFormat format) {
  int frames_per_buffer = DEFAULT_FRAMES_PER_BUFFER;
  PaError err;

  // Init parameters of input device
  inputParameters = (PaStreamParameters *)malloc(sizeof(PaStreamParameters));
  inputParameters->device = input_device_index;
  inputParameters->channelCount = channels;
  inputParameters->sampleFormat = format;
  inputParameters->suggestedLatency =
      Pa_GetDeviceInfo(inputParameters->device)->defaultLowInputLatency;
  inputParameters->hostApiSpecificStreamInfo = nullptr;

  err = Pa_OpenStream(
      &pastream, inputParameters, nullptr, rate, frames_per_buffer,
      /* we only use input so don't bother clipping them */
      paClipOff,
      /* no callback (& context): default to 'blocking read/write' mode */
      nullptr, nullptr);
  if (err != paNoError) {
    report_error_and_throw(err);
  }

  pastreamInfo = (PaStreamInfo *)Pa_GetStreamInfo(pastream);
  if (!pastreamInfo) {
    AERROR << "Could not get stream information: " << paInternalError;
    throw std::runtime_error("");
  }
}

void Stream::read_stream(int n_frames, char *buffer) {
  int err = Pa_ReadStream(this->pastream, buffer, n_frames);
  if (err != paNoError) {
    report_error_and_throw(err);
  }
}

int Stream::get_chunk_size(int n_frames) {
  return (n_frames) * (inputParameters->channelCount) *
         (Pa_GetSampleSize(inputParameters->sampleFormat));
}

// Respeaker
void Respeaker::init(int sample_rate, int sample_width, int n_channels) {
  int err = Pa_Initialize();
  if (err != paNoError) {
    Pa_Terminate();
    report_error_and_throw(err);
  }

  PaDeviceIndex device_index = get_respeaker_index();
  stream_.reset(new Stream());
  stream_->init_stream(sample_rate, n_channels, device_index,
                     get_format_from_width(sample_width));
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
  AERROR << "Invalid width: " << width;
  throw std::runtime_error("");
}

PaDeviceIndex Respeaker::get_respeaker_index() {
  // return index of respeaker
  const PaHostApiInfo *host_api_info = get_host_api_info(0);
  const PaDeviceInfo *device_info = nullptr;
  for (PaDeviceIndex i = 0; i < host_api_info->deviceCount; ++i) {
    device_info = get_device_info(host_api_device_index_to_device_index(0, i));
    if (std::string(device_info->name).find("ReSpeaker") != std::string::npos) {
      return i;
    }
  }
  AERROR << "Error: Respeaker device not found";
  throw std::runtime_error("");
}

const PaDeviceInfo *Respeaker::get_device_info(PaDeviceIndex index) {
  PaDeviceInfo *_info;

  _info = (PaDeviceInfo *)Pa_GetDeviceInfo(index);
  if (!_info) {
    AERROR << "Invalid device index" << index;
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
    report_error_and_throw(device_index);
  }
  return device_index;
}

const PaHostApiInfo *Respeaker::get_host_api_info(PaHostApiIndex index) {
  // Get host api info by it's index
  PaHostApiInfo *_info = (PaHostApiInfo *)Pa_GetHostApiInfo(index);
  if (!_info) {
    AERROR << "Invalid Host Api Index " << index;
    throw std::runtime_error("");
  }
  return _info;
}