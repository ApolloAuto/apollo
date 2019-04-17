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

#include <iostream>
#include <memory>

#include "h265_decoder.h"

#include "gflags/gflags.h"

DECLARE_string(input_video);
DECLARE_string(output_dir);
DEFINE_string(input_video, "", "The input video file");
DEFINE_string(output_dir, "", "The directory to output decoded pictures.");

void write_jpg_file(const char* jpeg_buffer, const int jpeg_size,
                    const std::string output_dir, const int frame_num) {
  const int JPEG_FILE_LEN = 256;
  const int FILE_SUFFIX_LEN = 16;
  const std::string output_file_pattern = output_dir + "/%05d.jpg";
  char jpeg_file_path[JPEG_FILE_LEN];
  snprintf(jpeg_file_path, output_file_pattern.length() + FILE_SUFFIX_LEN,
           output_file_pattern.c_str(), frame_num);
  FILE* jpeg_file = fopen(jpeg_file_path, "wb");
  if (jpeg_file == nullptr) {
    std::cerr << "error: failed to create jpeg file" << jpeg_file_path
              << std::endl;
    return;
  }
  fwrite(jpeg_buffer, jpeg_size, 1, jpeg_file);
  fclose(jpeg_file);
}

int stream_decoder_process_frame(const uint8_t* indata, const uint32_t insize,
                                 const std::string output_dir) {
  const uint32_t JPEG_BUF_SIZE = 1920 * 1080 * 2;
  AVCodecParserContext* codec_parser = av_parser_init(AV_CODEC_ID_H265);
  if (codec_parser == nullptr) {
    std::cerr << "error: failed to init AVCodecParserContext" << std::endl;
    return -1;
  }
  AVPacket apt;
  av_init_packet(&apt);
  const std::unique_ptr<H265Decoder> decoder(new H265Decoder());
  if (!decoder->Init()) {
    std::cerr << "error: failed to init decoder" << std::endl;
    return -1;
  }
  uint32_t local_size = insize;
  uint8_t* local_data = const_cast<uint8_t*>(indata);
  std::cout << "decoding: video size = " << local_size << std::endl;
  int frame_num = 0;
  int error_frame_num = 0;
  char* jpeg_buffer = new char[JPEG_BUF_SIZE];
  while (local_size > 0) {
    int frame_len = av_parser_parse2(
        codec_parser, decoder->get_codec_ctx_h265(), &(apt.data), &(apt.size),
        local_data, local_size, AV_NOPTS_VALUE, AV_NOPTS_VALUE, AV_NOPTS_VALUE);
    if (apt.data == nullptr) {
      apt.data = local_data;
      apt.size = local_size;
    }
    std::cout << "frame " << frame_num << ": frame_len=" << frame_len
              << ". left_size=" << local_size << std::endl;
    int jpeg_size =
        decoder->Process((char*)apt.data, apt.size, jpeg_buffer, JPEG_BUF_SIZE);
    if (jpeg_size > 0) {
      write_jpg_file(jpeg_buffer, jpeg_size, output_dir, frame_num);
      frame_num++;
    } else {
      error_frame_num++;
    }
    local_data += frame_len;
    local_size -= frame_len;
  }
  // Trying to decode the left over frames from buffer
  for (int i = error_frame_num; i >= 0; i--) {
    int jpeg_size = decoder->Process(nullptr, 0, jpeg_buffer, JPEG_BUF_SIZE);
    if (jpeg_size > 0) {
      write_jpg_file(jpeg_buffer, jpeg_size, output_dir, frame_num);
      frame_num++;
    }
    std::cout << "frame " << frame_num << ": read from buffer" << std::endl;
  }
  std::cout << "total frames: " << frame_num << std::endl;
  delete[] jpeg_buffer;
  av_parser_close(codec_parser);
  return 0;
}

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  std::cout << "input video: " << FLAGS_input_video
            << ". output dir: " << FLAGS_output_dir << std::endl;
  FILE* fp = fopen(FLAGS_input_video.c_str(), "rb");
  if (fp == nullptr) {
    std::cerr << "error: failed to open file " << FLAGS_input_video
              << std::endl;
    return -1;
  }
  fseek(fp, 0, SEEK_END);
  uint32_t insize = (uint32_t)ftell(fp);
  fseek(fp, 0, SEEK_SET);
  uint8_t* indata = (uint8_t*)malloc(insize);
  fread(indata, insize, 1, fp);
  if (stream_decoder_process_frame(indata, insize, FLAGS_output_dir) < 0) {
    std::cerr << "error: failed to decode file " << FLAGS_input_video
              << std::endl;
    return -1;
  }
  fclose(fp);
  fp = nullptr;
  return 0;
}
