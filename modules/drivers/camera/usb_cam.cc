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

#include <cmath>
#include <string>

#ifndef __aarch64__
#include "adv_plat/include/adv_trigger.h"
#include "modules/drivers/camera/util.h"
#endif
#include "modules/drivers/camera/usb_cam.h"

#define __STDC_CONSTANT_MACROS

#define CLEAR(x) memset(&(x), 0, sizeof(x))

namespace apollo {
namespace drivers {
namespace camera {

UsbCam::UsbCam()
    : fd_(-1),
      buffers_(NULL),
      n_buffers_(0),
      is_capturing_(false),
      image_seq_(0),
      device_wait_sec_(2),
      last_nsec_(0),
      frame_drop_interval_(0.0) {}

UsbCam::~UsbCam() {
  stop_capturing();
  uninit_device();
  close_device();
}

bool UsbCam::init(const std::shared_ptr<Config>& cameraconfig) {
  config_ = cameraconfig;

  if (config_->pixel_format() == "yuyv") {
    pixel_format_ = V4L2_PIX_FMT_YUYV;
  } else if (config_->pixel_format() == "uyvy") {
    pixel_format_ = V4L2_PIX_FMT_UYVY;
  } else if (config_->pixel_format() == "mjpeg") {
    pixel_format_ = V4L2_PIX_FMT_MJPEG;
  } else if (config_->pixel_format() == "yuvmono10") {
    pixel_format_ = V4L2_PIX_FMT_YUYV;
    config_->set_monochrome(true);
  } else if (config_->pixel_format() == "rgb24") {
    pixel_format_ = V4L2_PIX_FMT_RGB24;
  } else {
    pixel_format_ = V4L2_PIX_FMT_YUYV;
    AERROR << "Wrong pixel fromat:" << config_->pixel_format()
           << ",must be yuyv | uyvy | mjpeg | yuvmono10 | rgb24";
    return false;
  }
  if (pixel_format_ == V4L2_PIX_FMT_MJPEG) {
    init_mjpeg_decoder(config_->width(), config_->height());
  }

  // Warning when diff with last > 1.5* interval
  frame_warning_interval_ = static_cast<float>(1.5 / config_->frame_rate());
  // now max fps 30, we use a appox time 0.9 to drop image.
  frame_drop_interval_ = static_cast<float>(0.9 / config_->frame_rate());

  return true;
}

int UsbCam::init_mjpeg_decoder(int image_width, int image_height) {
  avcodec_register_all();

  avcodec_ = avcodec_find_decoder(AV_CODEC_ID_MJPEG);
  if (!avcodec_) {
    AERROR << "Could not find MJPEG decoder";
    return 0;
  }

  avcodec_context_ = avcodec_alloc_context3(avcodec_);

#if LIBAVCODEC_VERSION_INT >= AV_VERSION_INT(57, 0, 0)
  avframe_camera_ = av_frame_alloc();
  avframe_rgb_ = av_frame_alloc();

  avpicture_alloc(reinterpret_cast<AVPicture*>(avframe_rgb_), AV_PIX_FMT_RGB24,
                  image_width, image_height);
#else
  avframe_camera_ = avcodec_alloc_frame();
  avframe_rgb_ = avcodec_alloc_frame();

  avpicture_alloc(reinterpret_cast<AVPicture*>(avframe_rgb_), PIX_FMT_RGB24,
                  image_width, image_height);
#endif

  avcodec_context_->codec_id = AV_CODEC_ID_MJPEG;
  avcodec_context_->width = image_width;
  avcodec_context_->height = image_height;

#if LIBAVCODEC_VERSION_MAJOR > 52
#if LIBAVCODEC_VERSION_INT >= AV_VERSION_INT(57, 0, 0)
  avcodec_context_->pix_fmt = AV_PIX_FMT_YUV422P;
#else
  avcodec_context_->pix_fmt = PIX_FMT_YUV422P;
#endif
  avcodec_context_->codec_type = AVMEDIA_TYPE_VIDEO;
#endif

#if LIBAVCODEC_VERSION_INT >= AV_VERSION_INT(57, 0, 0)
  avframe_camera_size_ =
      avpicture_get_size(AV_PIX_FMT_YUV422P, image_width, image_height);
  avframe_rgb_size_ =
      avpicture_get_size(AV_PIX_FMT_RGB24, image_width, image_height);
#else
  avframe_camera_size_ =
      avpicture_get_size(PIX_FMT_YUV422P, image_width, image_height);
  avframe_rgb_size_ =
      avpicture_get_size(PIX_FMT_RGB24, image_width, image_height);
#endif

  /* open it */
  if (avcodec_open2(avcodec_context_, avcodec_, &avoptions_) < 0) {
    AERROR << "Could not open MJPEG Decoder";
    return 0;
  }
  return 1;
}

void UsbCam::mjpeg2rgb(char* mjpeg_buffer, int len, char* rgb_buffer,
                       int NumPixels) {
  (void)NumPixels;
  int got_picture = 0;

  memset(rgb_buffer, 0, avframe_rgb_size_);

#if LIBAVCODEC_VERSION_MAJOR > 52
  int decoded_len;
  AVPacket avpkt;
  av_init_packet(&avpkt);

  avpkt.size = len;
  avpkt.data = (unsigned char*)mjpeg_buffer;
  decoded_len = avcodec_decode_video2(avcodec_context_, avframe_camera_,
                                      &got_picture, &avpkt);

  if (decoded_len < 0) {
    AERROR << "Error while decoding frame.";
    return;
  }
#else
  avcodec_decode_video(avcodec_context_, avframe_camera_, &got_picture,
                       reinterpret_cast<uint8_t*>(mjpeg_buffer), len);
#endif

  if (!got_picture) {
    AERROR << "Webcam: expected picture but didn't get it...";
    return;
  }

  int xsize = avcodec_context_->width;
  int ysize = avcodec_context_->height;
  int pic_size = avpicture_get_size(avcodec_context_->pix_fmt, xsize, ysize);
  if (pic_size != avframe_camera_size_) {
    AERROR << "outbuf size mismatch.  pic_size:" << pic_size
           << ",buffer_size:" << avframe_camera_size_;
    return;
  }

#if LIBAVCODEC_VERSION_INT >= AV_VERSION_INT(57, 0, 0)
  video_sws_ =
      sws_getContext(xsize, ysize, avcodec_context_->pix_fmt, xsize, ysize,
                     AV_PIX_FMT_RGB24, SWS_BILINEAR, nullptr, nullptr, nullptr);
#else
  video_sws_ =
      sws_getContext(xsize, ysize, avcodec_context_->pix_fmt, xsize, ysize,
                     PIX_FMT_RGB24, SWS_BILINEAR, nullptr, nullptr, nullptr);
#endif

  sws_scale(video_sws_, avframe_camera_->data, avframe_camera_->linesize, 0,
            ysize, avframe_rgb_->data, avframe_rgb_->linesize);
  sws_freeContext(video_sws_);

#if LIBAVCODEC_VERSION_INT >= AV_VERSION_INT(57, 0, 0)
  int size = avpicture_layout(
      reinterpret_cast<AVPicture*>(avframe_rgb_), AV_PIX_FMT_RGB24,
      xsize, ysize,
      reinterpret_cast<uint8_t*>(rgb_buffer), avframe_rgb_size_);
#else
  int size = avpicture_layout(
      reinterpret_cast<AVPicture*>(avframe_rgb_), PIX_FMT_RGB24, xsize, ysize,
      reinterpret_cast<uint8_t*>(rgb_buffer), avframe_rgb_size_);
#endif
  if (size != avframe_rgb_size_) {
    AERROR << "webcam: avpicture_layout error: " << size;
    return;
  }
}

bool UsbCam::poll(const CameraImagePtr& raw_image) {
  raw_image->is_new = 0;
  // free memory in this struct desturctor
  memset(raw_image->image, 0, raw_image->image_size * sizeof(char));

  fd_set fds;
  struct timeval tv;
  int r = 0;

  FD_ZERO(&fds);
  FD_SET(fd_, &fds);

  /* Timeout. */
  tv.tv_sec = 2;
  tv.tv_usec = 0;

  r = select(fd_ + 1, &fds, nullptr, nullptr, &tv);

  if (-1 == r) {
    if (EINTR == errno) {
      return false;
    }

    // errno_exit("select");
    reconnect();
  }

  if (0 == r) {
    AERROR << "select timeout";
    reconnect();
  }

  int get_new_image = read_frame(raw_image);

  if (!get_new_image) {
    return false;
  }

  raw_image->is_new = 1;
  return true;
}

bool UsbCam::open_device(void) {
  struct stat st;

  if (-1 == stat(config_->camera_dev().c_str(), &st)) {
    AERROR << "Cannot identify '" << config_->camera_dev() << "': " << errno
           << ", " << strerror(errno);
    return false;
  }

  if (!S_ISCHR(st.st_mode)) {
    AERROR << config_->camera_dev() << " is no device";
    return false;
  }

  fd_ = open(config_->camera_dev().c_str(), O_RDWR /* required */ | O_NONBLOCK,
             0);

  if (-1 == fd_) {
    AERROR << "Cannot open '" << config_->camera_dev() << "': " << errno << ", "
           << strerror(errno);
    return false;
  }

  return true;
}

bool UsbCam::init_device(void) {
  struct v4l2_capability cap;
  struct v4l2_cropcap cropcap;
  struct v4l2_crop crop;
  struct v4l2_format fmt;
  unsigned int min = 0;

  if (-1 == xioctl(fd_, VIDIOC_QUERYCAP, &cap)) {
    if (EINVAL == errno) {
      AERROR << config_->camera_dev() << " is no V4L2 device";
      return false;
    }
    AERROR << "VIDIOC_QUERYCAP";
    return false;
  }

  if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
    AERROR << config_->camera_dev() << " is no video capture device";
    return false;
  }

  switch (config_->io_method()) {
    case IO_METHOD_READ:
      if (!(cap.capabilities & V4L2_CAP_READWRITE)) {
        AERROR << config_->camera_dev() << " does not support read i/o";
        return false;
      }

      break;

    case IO_METHOD_MMAP:
    case IO_METHOD_USERPTR:
      if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
        AERROR << config_->camera_dev() << " does not support streaming i/o";
        return false;
      }

      break;
    case IO_METHOD_UNKNOWN:
      AERROR << config_->camera_dev() << " does not support unknown i/o";
      return false;
      break;
  }

  /* Select video input, video standard and tune here. */
  CLEAR(cropcap);

  cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  if (0 == xioctl(fd_, VIDIOC_CROPCAP, &cropcap)) {
    crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    crop.c = cropcap.defrect; /* reset to default */

    if (-1 == xioctl(fd_, VIDIOC_S_CROP, &crop)) {
      switch (errno) {
        case EINVAL:
          /* Cropping not supported. */
          break;
      }
    }
  }

  CLEAR(fmt);

  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = config_->width();
  fmt.fmt.pix.height = config_->height();
  fmt.fmt.pix.pixelformat = pixel_format_;
  fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;

  if (-1 == xioctl(fd_, VIDIOC_S_FMT, &fmt)) {
    AERROR << "VIDIOC_S_FMT";
    return false;
  }

  /* Note VIDIOC_S_FMT may change width and height. */

  /* Buggy driver paranoia. */
  min = fmt.fmt.pix.width * 2;

  if (fmt.fmt.pix.bytesperline < min) {
    fmt.fmt.pix.bytesperline = min;
  }

  min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;

  if (fmt.fmt.pix.sizeimage < min) {
    fmt.fmt.pix.sizeimage = min;
  }

  config_->set_width(fmt.fmt.pix.width);
  config_->set_height(fmt.fmt.pix.height);

  struct v4l2_streamparm stream_params;
  memset(&stream_params, 0, sizeof(stream_params));
  stream_params.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  // if (xioctl(fd_, VIDIOC_G_PARM, &stream_params) < 0) {
  //   // errno_exit("Couldn't query v4l fps!");
  //   AERROR << "Couldn't query v4l fps!";
  //   // reconnect();
  //   return false;
  // }

  AINFO << "Capability flag: 0x" << stream_params.parm.capture.capability;

  stream_params.parm.capture.timeperframe.numerator = 1;
  stream_params.parm.capture.timeperframe.denominator = config_->frame_rate();

  if (xioctl(fd_, VIDIOC_S_PARM, &stream_params) < 0) {
    AINFO << "Couldn't set camera framerate";
  } else {
    AINFO << "Set framerate to be " << config_->frame_rate();
  }

  switch (config_->io_method()) {
    case IO_METHOD_READ:
      init_read(fmt.fmt.pix.sizeimage);
      break;

    case IO_METHOD_MMAP:
      init_mmap();
      break;

    case IO_METHOD_USERPTR:
      init_userp(fmt.fmt.pix.sizeimage);
      break;

    case IO_METHOD_UNKNOWN:
      AERROR << " does not support unknown i/o";
      break;
  }

  return true;
}

#ifndef __aarch64__
bool UsbCam::set_adv_trigger() {
  AINFO << "Trigger enable, dev:" << config_->camera_dev()
        << ", fps:" << config_->trigger_fps()
        << ", internal:" << config_->trigger_internal();
  int ret = adv_trigger_enable(
      config_->camera_dev().c_str(),
      static_cast<unsigned char>(config_->trigger_fps()),
      static_cast<unsigned char>(config_->trigger_internal()));
  if (ret != 0) {
    AERROR << "trigger failed:" << ret;
    return false;
  }
  return true;
}
#endif

int UsbCam::xioctl(int fd, int request, void* arg) {
  int r = 0;
  do {
    r = ioctl(fd, request, arg);
  } while (-1 == r && EINTR == errno);

  return r;
}

bool UsbCam::init_read(unsigned int buffer_size) {
  buffers_ = reinterpret_cast<buffer*>(calloc(1, sizeof(*buffers_)));

  if (!buffers_) {
    AERROR << "Out of memory";
    // exit(EXIT_FAILURE);
    // reconnect();
    return false;
  }

  buffers_[0].length = buffer_size;
  buffers_[0].start = malloc(buffer_size);

  if (!buffers_[0].start) {
    AERROR << "Out of memory";
    return false;
  }

  return true;
}

bool UsbCam::init_mmap(void) {
  struct v4l2_requestbuffers req;
  CLEAR(req);

  req.count = 1;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;

  if (-1 == xioctl(fd_, VIDIOC_REQBUFS, &req)) {
    if (EINVAL == errno) {
      AERROR << config_->camera_dev() << " does not support memory mapping";
      return false;
    }
    return false;
  }

  buffers_ = reinterpret_cast<buffer*>(calloc(req.count, sizeof(*buffers_)));

  if (!buffers_) {
    AERROR << "Out of memory";
    return false;
  }

  for (n_buffers_ = 0; n_buffers_ < req.count; ++n_buffers_) {
    struct v4l2_buffer buf;
    CLEAR(buf);

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = n_buffers_;

    if (-1 == xioctl(fd_, VIDIOC_QUERYBUF, &buf)) {
      AERROR << "VIDIOC_QUERYBUF";
      return false;
    }

    buffers_[n_buffers_].length = buf.length;
    buffers_[n_buffers_].start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE,
                                      MAP_SHARED, fd_, buf.m.offset);

    if (MAP_FAILED == buffers_[n_buffers_].start) {
      AERROR << "mmap";
      return false;
    }
  }

  return true;
}

bool UsbCam::init_userp(unsigned int buffer_size) {
  struct v4l2_requestbuffers req;
  unsigned int page_size = 0;

  page_size = getpagesize();
  buffer_size = (buffer_size + page_size - 1) & ~(page_size - 1);

  CLEAR(req);

  req.count = 4;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_USERPTR;

  if (-1 == xioctl(fd_, VIDIOC_REQBUFS, &req)) {
    if (EINVAL == errno) {
      AERROR << config_->camera_dev()
             << " does not support "
                "user pointer i/o";
      return false;
    }
    AERROR << "VIDIOC_REQBUFS";
    return false;
  }

  buffers_ = reinterpret_cast<buffer*>(calloc(4, sizeof(*buffers_)));

  if (!buffers_) {
    AERROR << "Out of memory";
    return false;
  }

  for (n_buffers_ = 0; n_buffers_ < 4; ++n_buffers_) {
    buffers_[n_buffers_].length = buffer_size;
    buffers_[n_buffers_].start =
        memalign(/* boundary */ page_size, buffer_size);

    if (!buffers_[n_buffers_].start) {
      AERROR << "Out of memory";
      return false;
    }
  }

  return true;
}

bool UsbCam::start_capturing(void) {
  if (is_capturing_) {
    return true;
  }

  unsigned int i = 0;
  enum v4l2_buf_type type;

  switch (config_->io_method()) {
    case IO_METHOD_READ:
      /* Nothing to do. */
      break;

    case IO_METHOD_MMAP:
      for (i = 0; i < n_buffers_; ++i) {
        struct v4l2_buffer buf;
        CLEAR(buf);

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        if (-1 == xioctl(fd_, VIDIOC_QBUF, &buf)) {
          AERROR << "VIDIOC_QBUF";
          return false;
        }
      }

      type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

      if (-1 == xioctl(fd_, VIDIOC_STREAMON, &type)) {
        AERROR << "VIDIOC_STREAMON";
        return false;
      }

      break;

    case IO_METHOD_USERPTR:
      for (i = 0; i < n_buffers_; ++i) {
        struct v4l2_buffer buf;

        CLEAR(buf);

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_USERPTR;
        buf.index = i;
        buf.m.userptr = reinterpret_cast<uint64_t>(buffers_[i].start);
        buf.length = static_cast<unsigned int>(buffers_[i].length);

        if (-1 == xioctl(fd_, VIDIOC_QBUF, &buf)) {
          AERROR << "VIDIOC_QBUF";
          return false;
        }
      }

      type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

      if (-1 == xioctl(fd_, VIDIOC_STREAMON, &type)) {
        AERROR << "VIDIOC_STREAMON";
        return false;
      }

      break;

    case IO_METHOD_UNKNOWN:
      AERROR << "unknown IO";
      return false;
      break;
  }

  is_capturing_ = true;
  return true;
}

void UsbCam::set_device_config() {
  if (config_->brightness() >= 0) {
    set_v4l_parameter("brightness", config_->brightness());
  }

  if (config_->contrast() >= 0) {
    set_v4l_parameter("contrast", config_->contrast());
  }

  if (config_->saturation() >= 0) {
    set_v4l_parameter("saturation", config_->saturation());
  }

  if (config_->sharpness() >= 0) {
    set_v4l_parameter("sharpness", config_->sharpness());
  }

  if (config_->gain() >= 0) {
    set_v4l_parameter("gain", config_->gain());
  }

  // check auto white balance
  if (config_->auto_white_balance()) {
    set_v4l_parameter("white_balance_temperature_auto", 1);
  } else {
    set_v4l_parameter("white_balance_temperature_auto", 0);
    set_v4l_parameter("white_balance_temperature", config_->white_balance());
  }

  // check auto exposure
  if (!config_->auto_exposure()) {
    // turn down exposure control (from max of 3)
    set_v4l_parameter("auto_exposure", 1);
    // change the exposure level
    set_v4l_parameter("exposure_absolute", config_->exposure());
  }

  // check auto focus
  if (config_->auto_focus()) {
    set_auto_focus(1);
    set_v4l_parameter("focus_auto", 1);
  } else {
    set_v4l_parameter("focus_auto", 0);
    if (config_->focus() >= 0) {
      set_v4l_parameter("focus_absolute", config_->focus());
    }
  }
}

bool UsbCam::uninit_device(void) {
  unsigned int i = 0;

  switch (config_->io_method()) {
    case IO_METHOD_READ:
      free(buffers_[0].start);
      break;

    case IO_METHOD_MMAP:
      for (i = 0; i < n_buffers_; ++i) {
        if (-1 == munmap(buffers_[i].start, buffers_[i].length)) {
          AERROR << "munmap";
          return false;
        }
      }

      break;

    case IO_METHOD_USERPTR:
      for (i = 0; i < n_buffers_; ++i) {
        free(buffers_[i].start);
      }

      break;

    case IO_METHOD_UNKNOWN:
      AERROR << "unknown IO";
      break;
  }

  return true;
}

bool UsbCam::close_device(void) {
  if (-1 == close(fd_)) {
    AERROR << "close";
    return false;
  }

  fd_ = -1;
  return true;
}

bool UsbCam::stop_capturing(void) {
  if (!is_capturing_) {
    return true;
  }

  is_capturing_ = false;
  enum v4l2_buf_type type;

  switch (config_->io_method()) {
    case IO_METHOD_READ:
      /* Nothing to do. */
      break;

    case IO_METHOD_MMAP:
    case IO_METHOD_USERPTR:
      type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

      if (-1 == xioctl(fd_, VIDIOC_STREAMOFF, &type)) {
        AERROR << "VIDIOC_STREAMOFF";
        return false;
      }

      break;
    case IO_METHOD_UNKNOWN:
      AERROR << "unknown IO";
      return false;
      break;
  }

  return true;
}

bool UsbCam::read_frame(CameraImagePtr raw_image) {
  struct v4l2_buffer buf;
  unsigned int i = 0;
  int len = 0;

  switch (config_->io_method()) {
    case IO_METHOD_READ:
      len = static_cast<int>(read(fd_, buffers_[0].start, buffers_[0].length));

      if (len == -1) {
        switch (errno) {
          case EAGAIN:
            AINFO << "EAGAIN";
            return false;

          case EIO:

            /* Could ignore EIO, see spec. */

            /* fall through */

          default:
            AERROR << "read";
            return false;
        }
      }

      process_image(buffers_[0].start, len, raw_image);

      break;

    case IO_METHOD_MMAP:
      CLEAR(buf);

      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;

      if (-1 == xioctl(fd_, VIDIOC_DQBUF, &buf)) {
        switch (errno) {
          case EAGAIN:
            return false;

          case EIO:

            /* Could ignore EIO, see spec. */

            /* fall through */

          default:
            AERROR << "VIDIOC_DQBUF";
            reconnect();
            return false;
        }
      }

      assert(buf.index < n_buffers_);
      len = buf.bytesused;
      raw_image->tv_sec = static_cast<int>(buf.timestamp.tv_sec);
      raw_image->tv_usec = static_cast<int>(buf.timestamp.tv_usec);

      {
        cyber::Time image_time(raw_image->tv_sec, 1000 * raw_image->tv_usec);
        uint64_t camera_timestamp = image_time.ToNanosecond();
        if (last_nsec_ == 0) {
          last_nsec_ = camera_timestamp;
        } else {
          double diff =
              static_cast<double>(camera_timestamp - last_nsec_) / 1e9;
          // drop image by frame_rate
          if (diff < frame_drop_interval_) {
            AINFO << "drop image:" << camera_timestamp;
            if (-1 == xioctl(fd_, VIDIOC_QBUF, &buf)) {
              AERROR << "VIDIOC_QBUF ERROR";
            }
            return false;
          }
          if (frame_warning_interval_ < diff) {
            AWARN << "stamp jump.last stamp:" << last_nsec_
                  << " current stamp:" << camera_timestamp;
          }
          last_nsec_ = camera_timestamp;
        }

        double now_s = static_cast<double>(cyber::Time::Now().ToSecond());
        double image_s = static_cast<double>(camera_timestamp) / 1e9;
        double diff = now_s - image_s;
        if (diff > 0.5 || diff < 0) {
          std::stringstream warning_stream;
          std::string warning_str;
          warning_stream << "camera time diff exception,diff:" << diff
                         << ";dev:" << config_->camera_dev();
          warning_stream >> warning_str;
          AWARN << warning_str;
        }
      }
      if (len < raw_image->width * raw_image->height) {
        AERROR << "Wrong Buffer Len: " << len
               << ", dev: " << config_->camera_dev();
      } else {
        process_image(buffers_[buf.index].start, len, raw_image);
      }

      if (-1 == xioctl(fd_, VIDIOC_QBUF, &buf)) {
        AERROR << "VIDIOC_QBUF";
        return false;
      }

      break;

    case IO_METHOD_USERPTR:
      CLEAR(buf);

      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_USERPTR;

      if (-1 == xioctl(fd_, VIDIOC_DQBUF, &buf)) {
        switch (errno) {
          case EAGAIN:
            return false;

          case EIO:

            /* Could ignore EIO, see spec. */

            /* fall through */

          default:
            AERROR << "VIDIOC_DQBUF";
            return false;
        }
      }

      for (i = 0; i < n_buffers_; ++i) {
        if (buf.m.userptr == reinterpret_cast<uint64_t>(buffers_[i].start) &&
            buf.length == buffers_[i].length) {
          break;
        }
      }

      assert(i < n_buffers_);
      len = buf.bytesused;
      process_image(reinterpret_cast<void*>(buf.m.userptr), len, raw_image);

      if (-1 == xioctl(fd_, VIDIOC_QBUF, &buf)) {
        AERROR << "VIDIOC_QBUF";
        return false;
      }

      break;

    case IO_METHOD_UNKNOWN:
      AERROR << "unknown IO";
      return false;
      break;
  }

  return true;
}

bool UsbCam::process_image(const void* src, int len, CameraImagePtr dest) {
  if (src == nullptr || dest == nullptr) {
    AERROR << "process image error. src or dest is null";
    return false;
  }
  if (pixel_format_ == V4L2_PIX_FMT_YUYV ||
      pixel_format_ == V4L2_PIX_FMT_UYVY) {
    if (config_->output_type() == YUYV) {
      memcpy(dest->image, src, dest->width * dest->height * 2);
    } else if (config_->output_type() == RGB) {
#ifdef __aarch64__
      convert_yuv_to_rgb_buffer((unsigned char*)src, (unsigned char*)dest->image,
                   dest->width, dest->height);
#else
      yuyv2rgb_avx((unsigned char*)src, (unsigned char*)dest->image,
                   dest->width * dest->height);
#endif
    } else {
      AERROR << "unsupported output format:" << config_->output_type();
      return false;
    }
  } else {
    AERROR << "unsupported pixel format:" << pixel_format_;
    return false;
  }
  return true;
}

bool UsbCam::is_capturing() { return is_capturing_; }

// enables/disables auto focus
void UsbCam::set_auto_focus(int value) {
  struct v4l2_queryctrl queryctrl;
  struct v4l2_ext_control control;

  memset(&queryctrl, 0, sizeof(queryctrl));
  queryctrl.id = V4L2_CID_FOCUS_AUTO;

  if (-1 == xioctl(fd_, VIDIOC_QUERYCTRL, &queryctrl)) {
    if (errno != EINVAL) {
      perror("VIDIOC_QUERYCTRL");
      return;
    } else {
      AINFO << "V4L2_CID_FOCUS_AUTO is not supported";
      return;
    }
  } else if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED) {
    AINFO << "V4L2_CID_FOCUS_AUTO is not supported";
    return;
  } else {
    memset(&control, 0, sizeof(control));
    control.id = V4L2_CID_FOCUS_AUTO;
    control.value = value;

    if (-1 == xioctl(fd_, VIDIOC_S_CTRL, &control)) {
      perror("VIDIOC_S_CTRL");
      return;
    }
  }
}

/**
 * Set video device parameter via call to v4l-utils.
 *
 * @param param The name of the parameter to set
 * @param param The value to assign
 */
void UsbCam::set_v4l_parameter(const std::string& param, int value) {
  set_v4l_parameter(param, std::to_string(value));
}
/**
 * Set video device parameter via call to v4l-utils.
 *
 * @param param The name of the parameter to set
 * @param param The value to assign
 */
void UsbCam::set_v4l_parameter(const std::string& param,
                               const std::string& value) {
  // build the command
  std::stringstream ss;
  ss << "v4l2-ctl --device=" << config_->camera_dev() << " -c " << param << "="
     << value << " 2>&1";
  std::string cmd = ss.str();

  // capture the output
  std::string output;
  char buffer[256];
  FILE* stream = popen(cmd.c_str(), "r");
  if (stream) {
    while (!feof(stream)) {
      if (fgets(buffer, 256, stream) != nullptr) {
        output.append(buffer);
      }
    }

    pclose(stream);
    // any output should be an error
    if (output.length() > 0) {
      AERROR << output.c_str();
    }
  } else {
    AERROR << "usb_cam_node could not run " << cmd.c_str();
  }
}

bool UsbCam::wait_for_device() {
  if (is_capturing_) {
    ADEBUG << "is capturing";
    return true;
  }
  if (!open_device()) {
    return false;
  }
  if (!init_device()) {
    close_device();
    return false;
  }
  if (!start_capturing()) {
    uninit_device();
    close_device();
    return false;
  }
#ifndef __aarch64__
  // will continue when trigger failed for self-trigger camera
  set_adv_trigger();
#endif
  return true;
}

void UsbCam::reconnect() {
  stop_capturing();
  uninit_device();
  close_device();
}

#ifdef __aarch64__
int UsbCam::convert_yuv_to_rgb_pixel(int y, int u, int v) {
    unsigned int pixel32 = 0;
    unsigned char *pixel = (unsigned char *)&pixel32;
    int r, g, b;
    r = (int)((double)y + (1.370705 * ((double)v-128.0)));
    g = (int)((double)y - (0.698001 * ((double)v-128.0)) - (0.337633 * ((double)u-128.0)));
    b = (int)((double)y + (1.732446 * ((double)u-128.0)));
    if(r > 255) r = 255;
    if(g > 255) g = 255;
    if(b > 255) b = 255;
    if(r < 0) r = 0;
    if(g < 0) g = 0;
    if(b < 0) b = 0;
    pixel[0] = (unsigned char)r ;
    pixel[1] = (unsigned char)g ;
    pixel[2] = (unsigned char)b ;
    return pixel32;
}


int UsbCam::convert_yuv_to_rgb_buffer(unsigned char *yuv, unsigned char *rgb, unsigned int width, unsigned int height) {
    unsigned int in, out = 0;
    unsigned int pixel_16;
    unsigned char pixel_24[3];
    unsigned int pixel32;
    int y0, u, y1, v;

    for(in = 0; in < width * height * 2; in += 4)
    {
        pixel_16 = yuv[in + 3] << 24 | yuv[in + 2] << 16 | yuv[in + 1] <<  8 | yuv[in + 0];
        y0 = (pixel_16 & 0x000000ff);
        u  = (pixel_16 & 0x0000ff00) >>  8;
        y1 = (pixel_16 & 0x00ff0000) >> 16;
        v  = (pixel_16 & 0xff000000) >> 24;
        pixel32 = convert_yuv_to_rgb_pixel(y0, u, v);
        pixel_24[0] = (unsigned char)(pixel32 & 0x000000ff);
        pixel_24[1] = (unsigned char)((pixel32 & 0x0000ff00) >> 8);
        pixel_24[2] = (unsigned char)((pixel32 & 0x00ff0000) >> 16);
        rgb[out++] = pixel_24[0];
        rgb[out++] = pixel_24[1];
        rgb[out++] = pixel_24[2];
        pixel32 = convert_yuv_to_rgb_pixel(y1, u, v);
        pixel_24[0] = (unsigned char)(pixel32 & 0x000000ff);
        pixel_24[1] = (unsigned char)((pixel32 & 0x0000ff00) >> 8);
        pixel_24[2] = (unsigned char)((pixel32 & 0x00ff0000) >> 16);
        rgb[out++] = pixel_24[0];
        rgb[out++] = pixel_24[1];
        rgb[out++] = pixel_24[2];
     }
     return 0;
}
#endif

}  // namespace camera
}  // namespace drivers
}  // namespace apollo
