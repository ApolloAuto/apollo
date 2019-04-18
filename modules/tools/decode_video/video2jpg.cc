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

#include <fstream>
#include <iostream>
#include <memory>

#include "gflags/gflags.h"

#include "cyber/common/log.h"
#include "modules/tools/decode_video/h265_decoder.h"

DECLARE_string(input_video);
DECLARE_string(output_dir);
DEFINE_string(input_video, "", "The input video file");
DEFINE_string(output_dir, "", "The directory to output decoded pictures.");

std::vector<uint8_t> read_video_file(const std::string file_name) {
  std::ifstream in(file_name, std::ios::binary);
  std::vector<uint8_t> file_buffer((std::istreambuf_iterator<char>(in)),
                                   std::istreambuf_iterator<char>());
  return file_buffer;
}

void write_jpg_file(const std::vector<uint8_t>& jpeg_buffer,
                    const std::string output_dir, const int frame_num) {
  const int JPEG_FILE_LEN = 256;
  const int FILE_SUFFIX_LEN = 16;
  const std::string output_file_pattern = output_dir + "/%05d.jpg";
  char jpeg_file_path[JPEG_FILE_LEN];
  snprintf(jpeg_file_path, output_file_pattern.length() + FILE_SUFFIX_LEN,
           output_file_pattern.c_str(), frame_num);
  std::ofstream out(jpeg_file_path, std::ios::binary);
  for (auto&& current : jpeg_buffer) out << static_cast<char>(current);
}

int stream_decoder_process_frame(const std::vector<uint8_t>& indata,
                                 const std::string output_dir) {
  AVCodecParserContext* codec_parser = av_parser_init(AV_CODEC_ID_H265);
  if (codec_parser == nullptr) {
    AERROR << "error: failed to init AVCodecParserContext";
    return -1;
  }
  AVPacket apt;
  av_init_packet(&apt);
  const std::unique_ptr<H265Decoder> decoder(new H265Decoder());
  if (!decoder->Init()) {
    AERROR << "error: failed to init decoder";
    return -1;
  }
  uint32_t local_size = static_cast<uint32_t>(indata.size());
  uint8_t* local_data = const_cast<uint8_t*>(indata.data());
  AINFO << "decoding: video size = " << local_size;
  int frame_num = 0;
  int error_frame_num = 0;
  while (local_size > 0) {
    int frame_len = av_parser_parse2(
        codec_parser, decoder->get_codec_ctx_h265(), &(apt.data), &(apt.size),
        local_data, local_size, AV_NOPTS_VALUE, AV_NOPTS_VALUE, AV_NOPTS_VALUE);
    if (apt.data == nullptr) {
      apt.data = local_data;
      apt.size = local_size;
    }
    AINFO << "frame " << frame_num << ": frame_len=" << frame_len
          << ". left_size=" << local_size;
    std::vector<uint8_t> jpeg_buffer = decoder->Process(apt.data, apt.size);
    if (!jpeg_buffer.empty()) {
      write_jpg_file(jpeg_buffer, output_dir, frame_num);
      frame_num++;
    } else {
      error_frame_num++;
    }
    local_data += frame_len;
    local_size -= frame_len;
  }
  // Trying to decode the left over frames from buffer
  for (int i = error_frame_num; i >= 0; i--) {
    std::vector<uint8_t> jpeg_buffer = decoder->Process(nullptr, 0);
    if (!jpeg_buffer.empty()) {
      write_jpg_file(jpeg_buffer, output_dir, frame_num);
      AINFO << "frame " << frame_num << ": read from buffer";
      frame_num++;
    }
  }
  AINFO << "total frames: " << frame_num;
  av_parser_close(codec_parser);
  return 0;
}

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  AINFO << "input video: " << FLAGS_input_video
        << ". output dir: " << FLAGS_output_dir;
  std::vector<uint8_t> video_file = read_video_file(FLAGS_input_video);
  if (video_file.empty()) {
    AERROR << "invalid input video file: " << FLAGS_input_video;
    return -1;
  }
  if (stream_decoder_process_frame(video_file, FLAGS_output_dir) != 0) {
    AERROR << "error: failed to decode file " << FLAGS_input_video;
    return -1;
  }
  return 0;
}
