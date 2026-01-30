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

#pragma once

#include <asm/types.h> /* for videodev2.h */
#include <malloc.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#ifndef __aarch64__
#include <immintrin.h>
#include <x86intrin.h>
#else
#include "modules/drivers/camera/format/yuv2rgb.h"
#endif

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/mem.h>
#include <libswscale/swscale.h>
#include <linux/videodev2.h>
}

#include <libavcodec/version.h>
#if LIBAVCODEC_VERSION_MAJOR < 55
#define AV_CODEC_ID_MJPEG CODEC_ID_MJPEG
#endif

#include <memory>
#include <sstream>
#include <string>

#include "cyber/cyber.h"

#include "modules/drivers/camera/proto/config.pb.h"

namespace apollo {
namespace drivers {
namespace camera {

enum CameraDeviceState : std::uint8_t {
  STATE_SUCCESS = 0,
  STATE_DROP,
  STATE_INTR,
  STATE_DEVICE_ERROR,
  STATE_OTHER,
};

// camera raw image struct
struct CameraImage {
  int width;
  int height;
  int bytes_per_pixel;
  int image_size;
  int is_new;
  int tv_sec;
  int tv_usec;
  char* image;

  ~CameraImage() {
    if (image != nullptr) {
      free(reinterpret_cast<void*>(image));
      image = nullptr;
    }
  }
};

typedef std::shared_ptr<CameraImage> CameraImagePtr;

struct buffer {
  void* start;
  size_t length;
  void* cuda_data;
};

class UsbCam {
 public:
  UsbCam();
  virtual ~UsbCam();

  virtual bool init(const std::shared_ptr<
    apollo::drivers::camera::config::Config>& camera_config);
  // user use this function to get camera frame data
  virtual CameraDeviceState poll(const CameraImagePtr& raw_image, 
                    const CameraImagePtr& sensor_raw_image);

  bool is_capturing();
  bool wait_for_device(void);

 private:
  int xioctl(int fd, int request, void* arg);
  bool init_device(void);
  bool uninit_device(void);

  void set_device_config();
  // enables/disable auto focus
  void set_auto_focus(int value);
  // set video device parameters
  void set_v4l_parameter(const std::string& param, int value);
  void set_v4l_parameter(const std::string& param, const std::string& value);

  int init_mjpeg_decoder(int image_width, int image_height);
  void mjpeg2rgb(char* mjepg_buffer, int len, char* rgb_buffer, int pixels);

#ifdef __aarch64__
  /**
  * @brief Non-floating conversion of YUV to RGB for arch without the neon instruction
  * @param y Luma of pixel
  * @param u Chroma of pixel
  * @param v Chroma of pixel
  * @return rgb pixel 
  */
  int convert_yuv_to_rgb_pixel(int y, int u, int v);

  /**
  * @brief Convert the encoding in the image from YUV to RGB
  * @param yuv image encoding by YUV format
  * @param rgb image encoding by RGB format
  * @param width width of the image
  * @param height height of the image
  * @return Status code, 0 is success
  */
  int convert_yuv_to_rgb_buffer(unsigned char* yuv, unsigned char* rgb,
                                unsigned int width, unsigned int height);

  /**
  * @brief Compensate the time from orin camera sensor to linux system kernel monotonic time
  * @param rtcpu_time_ns Monotonic time output by orin camera
  * @return Compensated monotonic time
  * @throw boost::bad_lexical_cast when the format of offset_ns is invalid
  */
  uint64_t compensate_rtcpu_to_kernel_time(uint64_t rtcpu_time_ns);
#endif

  bool init_read(unsigned int buffer_size);
  bool init_mmap(void);
  bool init_userp(unsigned int buffer_size);
  bool set_adv_trigger(void);
  bool close_device(void);
  bool open_device(void);
  CameraDeviceState read_frame(CameraImagePtr raw_image, CameraImagePtr sensor_raw_image);
  bool process_image(void* src, int len, CameraImagePtr dest, void* zero_copy_src=nullptr);
  bool start_capturing(void);
  bool stop_capturing(void);
  void reconnect();
  void reset_device();

  std::shared_ptr<
    apollo::drivers::camera::config::Config> config_;
  int pixel_format_;
  int fd_;
#ifdef __aarch64__
  int time_compensator_fd_;
  CudaConvertHandler handler_;
#endif
  buffer* buffers_;
  unsigned int n_buffers_;
  bool is_capturing_;
  uint64_t image_seq_;
  fd_set fds_;
  struct timeval tv_;

  AVFrame* avframe_camera_;
  AVFrame* avframe_rgb_;
  AVCodec* avcodec_;
  AVDictionary* avoptions_;
  AVCodecContext* avcodec_context_;
  int avframe_camera_size_;
  int avframe_rgb_size_;
  struct SwsContext* video_sws_;

  float frame_warning_interval_ = 0.0;
  float device_wait_sec_ = 0.0;
  uint64_t last_nsec_ = 0;
  float frame_drop_interval_ = 0.0;
  uint64_t last_error_time_ns_ = 0;
  uint64_t frame_drop_interval_ns_ = 0;
};
}  // namespace camera
}  // namespace drivers
}  // namespace apollo
