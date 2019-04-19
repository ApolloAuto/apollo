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

#include "adv_plat/include/adv_trigger.h"
#include "modules/drivers/camera/usb_cam.h"
#include "modules/drivers/camera/util.h"

#define __STDC_CONSTANT_MACROS

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
  // Now max fps 30, we use a appox time 0.9 to drop image.
  frame_drop_interval_ = static_cast<float>(0.9 / config_->frame_rate());

  return true;
}

bool UsbCam::init_mjpeg_decoder(int image_width, int image_height) {
  avcodec_register_all();

  avcodec_ = avcodec_find_decoder(AV_CODEC_ID_MJPEG);
  if (!avcodec_) {
    AERROR << "Could not find MJPEG decoder.";
    return false;
  }

  avcodec_context_ = avcodec_alloc_context3(avcodec_);
  avframe_camera_ = avcodec_alloc_frame();
  avframe_rgb_ = avcodec_alloc_frame();

  avpicture_alloc(reinterpret_cast<AVPicture*>(avframe_rgb_), PIX_FMT_RGB24,
                  image_width, image_height);

  avcodec_context_->codec_id = AV_CODEC_ID_MJPEG;
  avcodec_context_->width = image_width;
  avcodec_context_->height = image_height;

#if LIBAVCODEC_VERSION_MAJOR > 52
  avcodec_context_->pix_fmt = PIX_FMT_YUV422P;
  avcodec_context_->codec_type = AVMEDIA_TYPE_VIDEO;
#endif

  avframe_camera_size_ =
      avpicture_get_size(PIX_FMT_YUV422P, image_width, image_height);
  avframe_rgb_size_ =
      avpicture_get_size(PIX_FMT_RGB24, image_width, image_height);

  if (avcodec_open2(avcodec_context_, avcodec_, &avoptions_) < 0) {
    AERROR << "Cannot open MJPEG Decoder.";
    return false;
  }

  return true;
}

void UsbCam::mjpeg2rgb(char* mjpeg_buffer, int len, char* rgb_buffer,
                       int NumPixels) {
  (void)NumPixels;
  int got_picture = 0;


#if LIBAVCODEC_VERSION_MAJOR > 52
  int decoded_len;
  AVPacket avpkt;
  av_init_packet(&avpkt);

  avpkt.data = (unsigned char*)mjpeg_buffer;
  avpkt.size = len;
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
    AERROR << "Output buffer size mismatch. picture size: " << pic_size
           << ", buffer size: " << avframe_camera_size_;
    return;
  }

  video_sws_ = sws_getContext(xsize, ysize, avcodec_context_->pix_fmt, xsize,
		              ysize, PIX_FMT_RGB24, SWS_BILINEAR, nullptr,
			      nullptr, nullptr);
  sws_scale(video_sws_, avframe_camera_->data, avframe_camera_->linesize, 0,
            ysize, avframe_rgb_->data, avframe_rgb_->linesize);
  sws_freeContext(video_sws_);


  memset(rgb_buffer, '\0', avframe_rgb_size_);
  int size = avpicture_layout(
      reinterpret_cast<AVPicture*>(avframe_rgb_), PIX_FMT_RGB24, xsize, ysize,
      reinterpret_cast<uint8_t*>(rgb_buffer), avframe_rgb_size_);
  if (size != avframe_rgb_size_) {
    AERROR << "Webcam: avpicture_layout error! size: " << size;
  }
}

bool UsbCam::poll(const CameraImagePtr& raw_image) {
  raw_image->is_new = 0;
  // Free memory in this struct desturctor
  memset(raw_image->image, '\0', raw_image->image_size * sizeof(char));

  fd_set fds;
  FD_ZERO(&fds);
  FD_SET(fd_, &fds);

  // Wait up to 2 seconds for timeout expires
  struct timeval tv;
  tv.tv_sec = 2;
  tv.tv_usec = 0;
  int ret = select(fd_ + 1, &fds, nullptr, nullptr, &tv);
  if (ret < 0) {
    if (errno == EINTR) {
      return false;
    }
    reconnect();
  }

  if (ret == 0) {
    AERROR << "Timeout in 2 seconds.";
    reconnect();
  }

  int get_new_image = read_frame(raw_image);
  if (!get_new_image) {
    return false;
  }

  raw_image->is_new = 1;
  return true;
}

bool UsbCam::open_device() {
  struct stat st;
  if (stat(config_->camera_dev().c_str(), &st) < 0) {
    AERROR << "Cannot identify camera device: " << config_->camera_dev()
           << ", due to error: " << strerror(errno);
    return false;
  }

  if (!S_ISCHR(st.st_mode)) {
    AERROR << config_->camera_dev() << " is no device.";
    return false;
  }

  fd_ = open(config_->camera_dev().c_str(), O_RDWR|O_NONBLOCK, 0);
  if (fd_ < 0) {
    AERROR << "Cannot open camera device: " << config_->camera_dev()
           << ", due to error: " << strerror(errno);
    return false;
  }

  return true;
}

bool UsbCam::init_device() {
  struct v4l2_capability cap;

  if (xioctl(fd_, VIDIOC_QUERYCAP, &cap) < 0) {
    if (errno == EINVAL) {
      AERROR << config_->camera_dev() << " is no V4L2 device.";
      return false;
    }
    AERROR << "Failed to execute VIDIOC_QUERYCAP.";
    return false;
  }

  if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
    AERROR << config_->camera_dev() << " is no video capture device.";
    return false;
  }

  switch (config_->io_method()) {
    case IO_METHOD_READ:
      if (!(cap.capabilities & V4L2_CAP_READWRITE)) {
        AERROR << config_->camera_dev() << " does not support read i/o.";
        return false;
      }
      break;

    case IO_METHOD_MMAP:
    case IO_METHOD_USERPTR:
      if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
        AERROR << config_->camera_dev() << " does not support streaming i/o.";
        return false;
      }
      break;

    case IO_METHOD_UNKNOWN:
      AERROR << config_->camera_dev() << " does not support unknown i/o.";
      return false;
  }

  // Select video input, video standard and tune here.
  struct v4l2_cropcap cropcap;
  memset(&cropcap, '\0', sizeof(cropcap));
  cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(fd_, VIDIOC_CROPCAP, &cropcap) == 0) {
    struct v4l2_crop crop;
    crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    // Reset to default
    crop.c = cropcap.defrect;
    if (xioctl(fd_, VIDIOC_S_CROP, &crop) < 0) {
      if (errno == EINVAL) {
        AERROR << "Failed to set video device with cropping.";
      }
    }
  }

  struct v4l2_format fmt;
  memset(&fmt, '\0', sizeof(fmt));
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = config_->width();
  fmt.fmt.pix.height = config_->height();
  fmt.fmt.pix.pixelformat = pixel_format_;
  fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;
  if (xioctl(fd_, VIDIOC_S_FMT, &fmt) < 0) {
    AERROR << "VIDIOC_S_FMT";
    return false;
  }

  // Note that VIDIOC_S_FMT may change width and height.
  // Buggy driver paranoia.
  unsigned int min = fmt.fmt.pix.width * 2;
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
      AERROR << "Config does not support unknown i/o.";
      break;
  }

  return true;
}

bool UsbCam::set_adv_trigger() {
  AINFO << "Trigger enable, dev:" << config_->camera_dev()
        << ", fps:" << config_->trigger_fps()
        << ", internal:" << config_->trigger_internal();
  int ret = adv_trigger_enable(
      config_->camera_dev().c_str(),
      static_cast<unsigned char>(config_->trigger_fps()),
      static_cast<unsigned char>(config_->trigger_internal()));
  if (ret != 0) {
    AERROR << "Failed to trigger: " << ret;
    return false;
  }

  return true;
}

int UsbCam::xioctl(int fd, int request, void* arg) {
  int ret;
  do {
    ret = ioctl(fd, request, arg);
  } while (ret < 0 && errno == EINTR);

  return ret;
}

bool UsbCam::init_read(unsigned int buffer_size) {
  buffers_ = reinterpret_cast<buffer*>(calloc(1, sizeof(*buffers_)));
  if (!buffers_) {
    AERROR << "Out of memory.";
    // exit(EXIT_FAILURE);
    // reconnect();
    return false;
  }

  buffers_[0].length = buffer_size;
  buffers_[0].start = malloc(buffer_size);
  if (!buffers_[0].start) {
    AERROR << "Out of memory.";
    return false;
  }

  return true;
}

bool UsbCam::init_mmap() {
  struct v4l2_requestbuffers req;
  memset(&req, '\0', sizeof(req));
  req.count = 1;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;
  if (xioctl(fd_, VIDIOC_REQBUFS, &req) < 0) {
    if (errno == EINVAL) {
      AERROR << config_->camera_dev() << " does not support memory mapping.";
    }
    return false;
  }

  buffers_ = reinterpret_cast<buffer*>(calloc(req.count, sizeof(*buffers_)));
  if (!buffers_) {
    AERROR << "Out of memory.";
    return false;
  }

  for (n_buffers_ = 0; n_buffers_ < req.count; ++n_buffers_) {
    struct v4l2_buffer buf;
    memset(&buf, '\0', sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = n_buffers_;
    if (xioctl(fd_, VIDIOC_QUERYBUF, &buf) < 0) {
      AERROR << "VIDIOC_QUERYBUF";
      return false;
    }

    buffers_[n_buffers_].length = buf.length;
    buffers_[n_buffers_].start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE,
                                      MAP_SHARED, fd_, buf.m.offset);
    if (buffers_[n_buffers_].start == MAP_FAILED) {
      AERROR << "Failed to execute memory mapping in shared mode.";
      return false;
    }
  }

  return true;
}

bool UsbCam::init_userp(unsigned int buffer_size) {
  struct v4l2_requestbuffers req;
  memset(&req, '\0', sizeof(req));
  req.count = 4;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_USERPTR;
  if (xioctl(fd_, VIDIOC_REQBUFS, &req) < 0) {
    if (errno == EINVAL) {
      AERROR << config_->camera_dev() << " does not support "
             << "user pointer i/o.";
    }
    AERROR << "Failed to execute VIDIOC_REQBUFS.";
    return false;
  }

  buffers_ = reinterpret_cast<buffer*>(calloc(4, sizeof(*buffers_)));
  if (!buffers_) {
    AERROR << "Out of memory.";
    return false;
  }

  unsigned int page_size = static_cast<unsigned int>(sysconf(_SC_PAGESIZE));
  buffer_size = (buffer_size + page_size - 1) & ~(page_size - 1);
  for (n_buffers_ = 0; n_buffers_ < 4; ++n_buffers_) {
    buffers_[n_buffers_].length = buffer_size;
    buffers_[n_buffers_].start = memalign(page_size, buffer_size);
    if (!buffers_[n_buffers_].start) {
      AERROR << "Out of memory.";
      return false;
    }
  }

  return true;
}

bool UsbCam::start_capturing() {
  if (is_capturing_) {
    return true;
  }

  struct v4l2_buffer buf;
  enum v4l2_buf_type type;
  switch (config_->io_method()) {
    case IO_METHOD_READ:
      // Nothing to do.
      break;

    case IO_METHOD_MMAP:
      for (unsigned int i = 0; i < n_buffers_; ++i) {
        memset(&buf, '\0', sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        if (xioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
          AERROR << "Failed to execute VIDIOC_QBUF.";
          return false;
        }
      }

      type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      if (xioctl(fd_, VIDIOC_STREAMON, &type) < 0) {
        AERROR << "Failed to execute VIDIOC_STREAMON.";
        return false;
      }
      break;

    case IO_METHOD_USERPTR:
      for (unsigned int i = 0; i < n_buffers_; ++i) {
        memset(&buf, '\0', sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_USERPTR;
        buf.index = i;
        buf.m.userptr = reinterpret_cast<uint64_t>(buffers_[i].start);
        buf.length = static_cast<unsigned int>(buffers_[i].length);
        if (xioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
          AERROR << "Failed to execute VIDIOC_QBUF.";
          return false;
        }
      }

      type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      if (xioctl(fd_, VIDIOC_STREAMON, &type) < 0) {
        AERROR << "Failed to execute VIDIOC_STREAMON.";
        return false;
      }
      break;

    case IO_METHOD_UNKNOWN:
      AERROR << "Unknown I/O method.";
      return false;
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

  // Check auto exposure
  if (!config_->auto_exposure()) {
    // Turn down exposure control (from max of 3)
    set_v4l_parameter("auto_exposure", 1);
    // Change the exposure level
    set_v4l_parameter("exposure_absolute", config_->exposure());
  }

  // Check auto focus
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

bool UsbCam::uninit_device() {
  switch (config_->io_method()) {
    case IO_METHOD_READ:
      free(buffers_[0].start);
      break;

    case IO_METHOD_MMAP:
      for (unsigned int i = 0; i < n_buffers_; ++i) {
        if (munmap(buffers_[i].start, buffers_[i].length) < 0) {
          AERROR << "Failed to delete maping from: " << buffers_[i].start
                 << " with length: " << buffers_[i].length;
          return false;
        }
      }
      break;

    case IO_METHOD_USERPTR:
      for (unsigned int i = 0; i < n_buffers_; ++i) {
        free(buffers_[i].start);
      }
      break;

    case IO_METHOD_UNKNOWN:
      AERROR << "Unknown I/O method.";
      break;
  }

  return true;
}

bool UsbCam::close_device() {
  if (close(fd_) < 0) {
    AERROR << "Failed to close usb camera device.";
    return false;
  }

  fd_ = -1;
  return true;
}

bool UsbCam::stop_capturing() {
  if (!is_capturing_) {
    return true;
  }

  is_capturing_ = false;
  enum v4l2_buf_type type;

  switch (config_->io_method()) {
    // Nothing to do.
    case IO_METHOD_READ:
      break;
    case IO_METHOD_MMAP:
    case IO_METHOD_USERPTR:
      type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      if (xioctl(fd_, VIDIOC_STREAMOFF, &type) < 0) {
        AERROR << "VIDIOC_STREAMOFF";
        return false;
      }
      break;
    case IO_METHOD_UNKNOWN:
      AERROR << "Unknown I/O method.";
      return false;
  }

  return true;
}

bool UsbCam::read_frame(CameraImagePtr raw_image) {
  struct v4l2_buffer buf;

  ssize_t len;
  switch (config_->io_method()) {
    case IO_METHOD_READ:
      len = read(fd_, buffers_[0].start, buffers_[0].length);
      if (len == -1) {
        switch (errno) {
          case EAGAIN:
            AERROR << "No data were ready to read.";
            return false;
          // Could ignore EIO, see spec. fall through.
          case EIO:
          default:
            AERROR << "Failed to read frame from fd: " << fd_;
            return false;
        }
      }
      process_image(buffers_[0].start, len, raw_image);
      break;

    case IO_METHOD_MMAP:
      memset(&buf, '\0', sizeof(buf));
      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;
      if (xioctl(fd_, VIDIOC_DQBUF, &buf) < 0) {
        switch (errno) {
          case EAGAIN:
            return false;
          // Could ignore EIO, see spec. Fall through.
          case EIO:
          default:
            AERROR << "Failed to execute VIDIOC_DQBUF.";
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
          // Drop image by frame_rate
          if (diff < frame_drop_interval_) {
            AINFO << "Drop image: " << camera_timestamp;
            if (xioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
              AERROR << "VIDIOC_QBUF ERROR.";
            }
            return false;
          }
          if (frame_warning_interval_ < diff) {
            AWARN << "Timestamp jump.last stamp: " << last_nsec_
                  << ", current stamp:" << camera_timestamp;
          }
          last_nsec_ = camera_timestamp;
        }

        double now_s = static_cast<double>(cyber::Time::Now().ToSecond());
        double image_s = static_cast<double>(camera_timestamp) / 1e9;
        double diff = now_s - image_s;
        if (diff < 0 || diff > 0.5) {
          std::stringstream warning_stream;
          std::string warning_str;
          warning_stream << "Camera time diff exception. diff: " << diff
                         << ", device: " << config_->camera_dev();
          warning_stream >> warning_str;
          AWARN << warning_str;
        }
      }
      if (len < raw_image->width * raw_image->height) {
        AERROR << "Wrong Buffer Length: " << len
               << ", camera device: " << config_->camera_dev();
      } else {
        process_image(buffers_[buf.index].start, len, raw_image);
      }

      if (xioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
        AERROR << "Failed to execute VIDIOC_QBUF.";
        return false;
      }
      break;

    case IO_METHOD_USERPTR:
      memset(&buf, '\0', sizeof(buf));
      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_USERPTR;
      if (xioctl(fd_, VIDIOC_DQBUF, &buf) < 0) {
        switch (errno) {
          case EAGAIN:
            return false;
          // Could ignore EIO, see spec. Fall through.
          case EIO:
          default:
            AERROR << "Failed to execute VIDIOC_DQBUF.";
            return false;
        }
      }

      unsigned int i;
      for (i = 0; i < n_buffers_; ++i) {
        if (buf.m.userptr == reinterpret_cast<uint64_t>(buffers_[i].start) &&
            buf.length == buffers_[i].length) {
          break;
        }
      }

      assert(i < n_buffers_);
      len = buf.bytesused;
      process_image(reinterpret_cast<void*>(buf.m.userptr), len, raw_image);
      if (xioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
        AERROR << "Failed to execute VIDIOC_QBUF.";
        return false;
      }
      break;

    case IO_METHOD_UNKNOWN:
      AERROR << "Unknown I/O method.";
      return false;
  }

  return true;
}

bool UsbCam::process_image(const void* src, ssize_t len, CameraImagePtr dest) {
  if (src == nullptr || dest == nullptr) {
    AERROR << "Process image error. src or dest is null.";
    return false;
  }
  if (pixel_format_ == V4L2_PIX_FMT_YUYV ||
      pixel_format_ == V4L2_PIX_FMT_UYVY) {
    if (config_->output_type() == YUYV) {
      memcpy(dest->image, src, dest->width * dest->height * 2);
    } else if (config_->output_type() == RGB) {
      yuyv2rgb_avx((unsigned char*)src, (unsigned char*)dest->image,
                   dest->width * dest->height);
    } else {
      AERROR << "Unsupported output format: " << config_->output_type();
      return false;
    }
  } else {
    AERROR << "Unsupported pixel format: " << pixel_format_;
    return false;
  }

  return true;
}

bool UsbCam::is_capturing() const { return is_capturing_; }

// Enables/disables auto focus
void UsbCam::set_auto_focus(int value) {
  struct v4l2_queryctrl queryctrl;

  memset(&queryctrl, '\0', sizeof(queryctrl));
  queryctrl.id = V4L2_CID_FOCUS_AUTO;
  if (xioctl(fd_, VIDIOC_QUERYCTRL, &queryctrl) < 0) {
    if (errno != EINVAL) {
      AERROR << "Failed to execute VIDIOC_QUERYCTRL.";
    } else {
      AINFO << "V4L2_CID_FOCUS_AUTO is not supported.";
    }
    return;
  }
  if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED) {
    AINFO << "V4L2_CID_FOCUS_AUTO is not supported.";
    return;
  }

  struct v4l2_ext_control control;
  memset(&control, '\0', sizeof(control));
  control.id = V4L2_CID_FOCUS_AUTO;
  control.value = value;
  if (xioctl(fd_, VIDIOC_S_CTRL, &control) < 0) {
    AERROR << "Failed to execute VIDIOC_S_CTRL.";
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
  // Build the command
  std::stringstream ss;
  ss << "v4l2-ctl --device=" << config_->camera_dev() << " -c " << param << "="
     << value << " 2>&1";
  std::string cmd = ss.str();

  // Capture the output
  std::string output;
  FILE* stream = popen(cmd.c_str(), "r");
  if (!stream) {
    AERROR << "Failed to run usb_cam_node: " << cmd.c_str();
    return;
  }
  char buffer[256];
  while (!feof(stream)) {
    if (fgets(buffer, sizeof(buffer), stream) != nullptr) {
      output.append(buffer);
    }

    // Any output should be an error
    if (output.length() > 0) {
      AERROR << output.c_str();
    }
  }

  pclose(stream);
}

bool UsbCam::wait_for_device() {
  if (is_capturing_) {
    ADEBUG << "USB camera is in capturing.";
    return true;
  }
  if (!open_device()) {
    return false;
  }
  if (!init_device()) {
    goto close_camera_device;
  }
  if (!start_capturing()) {
    goto uninit_camera_device;
  }

  // Continue when trigger failed for self-trigger camera
  set_adv_trigger();
  return true;

uninit_camera_device:
  uninit_device();

close_camera_device:
  close_device();

  return false;
}

void UsbCam::reconnect() {
  stop_capturing();
  uninit_device();
  close_device();
}

}  // namespace camera
}  // namespace drivers
}  // namespace apollo
