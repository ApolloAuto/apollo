/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Robert Bosch LLC.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Robert Bosch nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include "modules/drivers/audio/respeaker.h"

#define DEFAULT_FRAMES_PER_BUFFER 1024

// Helper functions
void report_error_and_throw(PaError err) {
  AERROR << "An error occured while using the portaudio stream";
  AERROR << "Error number: " << err;
  AERROR << "Error message: " << Pa_GetErrorText(err);
  throw std::runtime_error();
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
    throw std::runtime_error();
  }
}

void Stream::read_stream(int n_frames, char *buffer) {
  int err;
  short *sampleBlock;
  int num_bytes = calculate_size(n_frames);

  err = Pa_ReadStream(this->pastream, buffer, n_frames);
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

  devIndex device_index = get_respeaker_index();
  stream.init_stream(sample_rate, n_channels, device_index,
                     get_format_from_width(sample_width));
}

PaSampleFormat Respeaker::get_format_from_width(int width,
                                                bool is_unsigned = true) {
  switch (width) {
    case 1:
      return is_unsigned ? paUInt8 : paInt8;
    case 2:
      return paInt16 case 3 : return paInt24 case 4 : return paFloat32 default
          : AERROR
            << "Invalid width: "
            << width;
      throw std::runtime_error();
  }
}

devIndex Respeaker::get_respeaker_index() {
  // return index of respeaker
  const PaHostApiInfo *host_api_info = get_host_api_info(0);
  const PaDeviceInfo *device_info = nullptr;
  for (devIndex i = 0; i < _info.deviceCount; ++i) {
    device_info = get_device_info(host_api_device_index_to_device_index(0, i));
    if (string(device_info->name).find("ReSpeaker") != string::npos) {
      return devIndex;
    }
  }
  AERROR << "Error: Respeaker device not found";
  throw std::runtime_error();
}

const PaDeviceInfo *Respeaker::get_device_info(PaDeviceIndex index) {
  PaDeviceInfo *_info;

  _info = (PaDeviceInfo *)Pa_GetDeviceInfo(index);
  if (!_info) {
    AERROR << "Invalid device index" << index;
    throw std::runtime_error();
  }

  return _info;
}

devIndex Respeaker::host_api_device_index_to_device_index(
    PaHostApiIndex hostApi, int hostApiDeviceIndex) {
  // Get standard device index from host-API-specific device index
  devIndex device_index =
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
    throw std::runtime_error();
  }
  return _info;
}