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

#include <algorithm>

#include "cyber/common/log.h"
#include "modules/drivers/video/tools/decode_video/h265_decoder.h"

namespace apollo {
namespace drivers {
namespace video {

// Using to-be-deprecated avcodec_decode_video2 for now until its replacement
// gets stable
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

bool H265Decoder::Init() {
  avcodec_register_all();
  AVCodec* codec_h265 = avcodec_find_decoder(AV_CODEC_ID_H265);
  if (codec_h265 == nullptr) {
    AERROR << "error: codec not found";
    return false;
  }
  codec_ctx_h265_ = avcodec_alloc_context3(codec_h265);
  if (codec_ctx_h265_ == nullptr) {
    AERROR << "error: codec context alloc fail";
    return false;
  }
  if (avcodec_open2(codec_ctx_h265_, codec_h265, nullptr) < 0) {
    AERROR << "error: could not open codec";
    return false;
  }
  yuv_frame_ = av_frame_alloc();
  if (yuv_frame_ == nullptr) {
    AERROR << "error: could not alloc yuv frame";
    return false;
  }
  AVCodec* codec_jpeg = avcodec_find_encoder(AV_CODEC_ID_MJPEG);
  if (codec_jpeg == nullptr) {
    AERROR << "error: jpeg Codec not found";
    return false;
  }
  codec_ctx_jpeg_ = avcodec_alloc_context3(codec_jpeg);
  if (codec_ctx_jpeg_ == nullptr) {
    AERROR << "error: jpeg ctx allco fail";
    return false;
  }
  // Put sample parameters and open it
  codec_ctx_jpeg_->bit_rate = 400000;
  codec_ctx_jpeg_->codec_type = AVMEDIA_TYPE_VIDEO;
  codec_ctx_jpeg_->codec_id = AV_CODEC_ID_MJPEG;
  codec_ctx_jpeg_->width = 1920;
  codec_ctx_jpeg_->height = 1080;
  codec_ctx_jpeg_->time_base = (AVRational){1, 15};
  codec_ctx_jpeg_->pix_fmt = AV_PIX_FMT_YUVJ422P;
  if (avcodec_open2(codec_ctx_jpeg_, codec_jpeg, nullptr) < 0) {
    AERROR << "error: could not open jpeg context";
    return false;
  }
  return true;
}

void H265Decoder::Release() {
  if (codec_ctx_h265_ != nullptr) {
    avcodec_free_context(&codec_ctx_h265_);
    codec_ctx_h265_ = nullptr;
  }
  if (yuv_frame_ != nullptr) {
    av_frame_free(&yuv_frame_);
    yuv_frame_ = nullptr;
  }
  if (codec_ctx_jpeg_ != nullptr) {
    avcodec_free_context(&codec_ctx_jpeg_);
    codec_ctx_jpeg_ = nullptr;
  }
}

H265Decoder::DecodingResult H265Decoder::Process(
    const uint8_t* indata, const int32_t insize,
    std::vector<uint8_t>* outdata) const {
  AVPacket apt;
  outdata->clear();
  av_init_packet(&apt);
  int got_picture = 0;
  apt.data = const_cast<uint8_t*>(indata);
  apt.size = insize;
  if (apt.size == 0) {
    apt.data = nullptr;
  }
  int ret =
      avcodec_decode_video2(codec_ctx_h265_, yuv_frame_, &got_picture, &apt);
  if (ret < 0) {
    AERROR << "error: decode failed: input_framesize = " << apt.size
           << ". error code = " << ret;
    return H265Decoder::DecodingResult::FATAL;
  }
  if (!got_picture) {
    // Not an error, but just did not read pictures out
    AWARN << "warn: failed to get yuv picture";
    return H265Decoder::DecodingResult::WARN;
  }
  av_packet_unref(&apt);
  got_picture = 0;
  ret = avcodec_encode_video2(codec_ctx_jpeg_, &apt, yuv_frame_, &got_picture);
  if (ret < 0) {
    AERROR << "error: jpeg encode failed, error code = " << ret;
    return H265Decoder::DecodingResult::FATAL;
  }
  if (!got_picture) {
    AERROR << "error: failed to get jpeg picture from yuyv";
    return H265Decoder::DecodingResult::FATAL;
  }
  outdata->resize(apt.size);
  std::copy(apt.data, apt.data + apt.size, outdata->begin());
  return H265Decoder::DecodingResult::SUCCESS;
}

}  // namespace video
}  // namespace drivers
}  // namespace apollo
