/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

#include <vector>

/**
 * @class H265Decoder
 * @brief H265Decoder is a class to actually decode videos.
 */
class H265Decoder {
 public:
  H265Decoder() = default;

  // Init decoder by acquiring resources
  bool Init();

  // Process frames according to input data, and output converted data
  std::vector<uint8_t> Process(const uint8_t* indata, const int32_t insize);

  // Destructor, releasing the resources
  ~H265Decoder() { Release(); }

  // Getter of codec_ctx_h265_
  AVCodecContext* get_codec_ctx_h265() { return codec_ctx_h265_; }

 private:
  void Release();

  AVCodec* codec_h265_ = nullptr;
  AVCodecContext* codec_ctx_h265_ = nullptr;
  AVCodec* codec_jpeg_ = nullptr;
  AVCodecContext* codec_ctx_jpeg_ = nullptr;
  AVFrame* yuv_frame_ = nullptr;
};
