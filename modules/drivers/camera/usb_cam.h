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
#include <unistd.h>

#include <immintrin.h>
#include <x86intrin.h>

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

using apollo::drivers::camera::config::Config;
using apollo::drivers::camera::config::IO_METHOD_MMAP;
using apollo::drivers::camera::config::IO_METHOD_READ;
using apollo::drivers::camera::config::IO_METHOD_UNKNOWN;
using apollo::drivers::camera::config::IO_METHOD_USERPTR;
using apollo::drivers::camera::config::RGB;
using apollo::drivers::camera::config::YUYV;

// Camera raw image struct
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
};

class UsbCam {
  public:
    UsbCam();
    virtual ~UsbCam();

    virtual bool init(const std::shared_ptr<Config>& camera_config);
    // User use this function to get camera frame data
    virtual bool poll(const CameraImagePtr& raw_image);

    bool is_capturing() const;
    bool wait_for_device();

  private:
    int xioctl(int fd, int request, void* arg);
    bool init_device();
    bool uninit_device();

    void set_device_config();
    // Enables/disable auto focus
    void set_auto_focus(int value);
    // Set video device parameters
    void set_v4l_parameter(const std::string& param, int value);
    void set_v4l_parameter(const std::string& param, const std::string& value);

    bool init_mjpeg_decoder(int image_width, int image_height);
    void mjpeg2rgb(char* mjepg_buffer, int len, char* rgb_buffer, int pixels);

    bool init_read(unsigned int buffer_size);
    bool init_mmap();
    bool init_userp(unsigned int buffer_size);
    bool set_adv_trigger();
    bool close_device();
    bool open_device();
    bool read_frame(CameraImagePtr raw_image);
    bool process_image(const void* src, ssize_t len, CameraImagePtr dest);
    bool start_capturing();
    bool stop_capturing();
    void reconnect();
    void reset_device();

  private:
    std::shared_ptr<Config> config_;
    int pixel_format_;
    int fd_;
    buffer* buffers_;
    unsigned int n_buffers_;
    bool is_capturing_;
    uint64_t image_seq_;

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
};
}  // namespace camera
}  // namespace drivers
}  // namespace apollo
