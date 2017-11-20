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
#define __STDC_CONSTANT_MACROS

#include <usb_cam/usb_cam.h>
#include <iostream>
#include <sstream>
#include <sensor_msgs/fill_image.h>

#define CLEAR(x) memset (&(x), 0, sizeof (x))

namespace usb_cam {

UsbCam::UsbCam()
    : _io_type(IO_METHOD_MMAP), _fd(-1), _n_buffers(0), _avframe_camera(NULL),
    _avframe_rgb(NULL), _avcodec(NULL), _avoptions(NULL), _avcodec_context(NULL),
    _avframe_camera_size(0), _avframe_rgb_size(0), _video_sws(NULL), _image(NULL),
    _is_capturing(false), _error_code(11) {
}
UsbCam::~UsbCam()
{
    shutdown();
}

int UsbCam::init_mjpeg_decoder(int image_width, int image_height)
{
    avcodec_register_all();

    _avcodec = avcodec_find_decoder(AV_CODEC_ID_MJPEG);
    if (!_avcodec)
    {
        ROS_ERROR("Could not find MJPEG decoder");
        return 0;
    }

    _avcodec_context = avcodec_alloc_context3(_avcodec);
    _avframe_camera = avcodec_alloc_frame();
    _avframe_rgb = avcodec_alloc_frame();

    avpicture_alloc((AVPicture *)_avframe_rgb, PIX_FMT_RGB24, image_width, image_height);

    _avcodec_context->codec_id = AV_CODEC_ID_MJPEG;
    _avcodec_context->width = image_width;
    _avcodec_context->height = image_height;

#if LIBAVCODEC_VERSION_MAJOR > 52
    _avcodec_context->pix_fmt = PIX_FMT_YUV422P;
    _avcodec_context->codec_type = AVMEDIA_TYPE_VIDEO;
#endif

    _avframe_camera_size = avpicture_get_size(PIX_FMT_YUV422P, image_width, image_height);
    _avframe_rgb_size = avpicture_get_size(PIX_FMT_RGB24, image_width, image_height);

    /* open it */
    if (avcodec_open2(_avcodec_context, _avcodec, &_avoptions) < 0)
    {
        ROS_ERROR("Could not open MJPEG Decoder");
        return 0;
    }
    return 1;
}

void UsbCam::mjpeg2rgb(char *MJPEG, int len, char *RGB, int NumPixels)
{
    int got_picture = 0;

    memset(RGB, 0, _avframe_rgb_size);

#if LIBAVCODEC_VERSION_MAJOR > 52
    int decoded_len;
    AVPacket avpkt;
    av_init_packet(&avpkt);

    avpkt.size = len;
    avpkt.data = (unsigned char*)MJPEG;
    decoded_len = avcodec_decode_video2(_avcodec_context, _avframe_camera, &got_picture, &avpkt);

    if (decoded_len < 0)
    {
        ROS_ERROR("Error while decoding frame.");
        return;
    }
#else
    avcodec_decode_video(_avcodec_context, _avframe_camera, &got_picture, (uint8_t *) MJPEG, len);
#endif

    if (!got_picture)
    {
        ROS_ERROR("Webcam: expected picture but didn't get it...");
        return;
    }

    int xsize = _avcodec_context->width;
    int ysize = _avcodec_context->height;
    int pic_size = avpicture_get_size(_avcodec_context->pix_fmt, xsize, ysize);
    if (pic_size != _avframe_camera_size)
    {
        ROS_ERROR("outbuf size mismatch.  pic_size: %d bufsize: %d", pic_size,
                 _avframe_camera_size);
        return;
    }

    _video_sws = sws_getContext(xsize, ysize, _avcodec_context->pix_fmt, xsize, ysize, 
                                PIX_FMT_RGB24, SWS_BILINEAR, NULL, NULL,  NULL);
    sws_scale(_video_sws, _avframe_camera->data, _avframe_camera->linesize, 0, ysize, 
            _avframe_rgb->data, _avframe_rgb->linesize);
    sws_freeContext(_video_sws);

    int size = avpicture_layout((AVPicture *)_avframe_rgb, PIX_FMT_RGB24, xsize, ysize, 
                                (uint8_t *)RGB, _avframe_rgb_size);
    if (size != _avframe_rgb_size)
    {
        ROS_ERROR("webcam: avpicture_layout error: %d", size);
        return;
    }
}

bool UsbCam::process_image(const void * src, int len, boost::shared_ptr<CameraImage> dest)
{
    if (src == NULL || dest == NULL) {
        ROS_ERROR("process image error. len: %d, width: %d, height: %d", len, dest->width, dest->height);
        return false;
    }
    if (_pixelformat == V4L2_PIX_FMT_YUYV || _pixelformat == V4L2_PIX_FMT_UYVY) {
       memcpy(dest->image, src, dest->width * dest->height * 2);
    } else {
      ROS_ERROR("unsupported pixel format: %d", _pixelformat);
      return false;
    }
    return true;
}

int UsbCam::read_frame()
{
    struct v4l2_buffer buf;
    unsigned int i = 0;
    int len = 0;
    bool result = false;

    switch (_io_type)
    {
        case IO_METHOD_READ:
            len = read(_fd, _buffers[0].start, _buffers[0].length);
            if (len == -1)
            {
                switch (errno)
                {
                    case EAGAIN:
                        std::cout << "EAGAIN" << std::endl;
                        return 0;

                    case EIO:
                        /* Could ignore EIO, see spec. */

                        /* fall through */

                    default:
                        errno_exit("read");
                }
            }

            result = process_image(_buffers[0].start, len, _image);
            if (!result) {
                return 0;
            }

            break;

        case IO_METHOD_MMAP:
            CLEAR(buf);
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;

            if (-1 == xioctl(_fd, VIDIOC_DQBUF, &buf))
            {
                switch (errno)
                {
                    case EAGAIN:
                        return 0;

                    case EIO:
                        /* Could ignore EIO, see spec. */

                        /* fall through */

                    default:
                        //reset_device();
                        errno_exit("VIDIOC_DQBUF");
                }
            }

            assert(buf.index < _n_buffers);
            len = buf.bytesused;
            _image->tv_sec = buf.timestamp.tv_sec;
            _image->tv_usec = buf.timestamp.tv_usec;
            ROS_DEBUG("new image timestamp: %d.%d", _image->tv_sec, _image->tv_usec);

            result = process_image(_buffers[buf.index].start, len, _image);
            if (!result) {
                return 0;
            }

            if (-1 == xioctl(_fd, VIDIOC_QBUF, &buf)) {
                errno_exit("VIDIOC_QBUF");
            }
            break;

        case IO_METHOD_USERPTR:
            CLEAR(buf);

            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_USERPTR;

            if (-1 == xioctl(_fd, VIDIOC_DQBUF, &buf))
            {
                switch (errno)
                {
                    case EAGAIN:
                        return 0;

                    case EIO:
                        /* Could ignore EIO, see spec. */
                        /* fall through */

                    default:
                        errno_exit("VIDIOC_DQBUF");
                }
            }

            for (i = 0; i < _n_buffers; ++i) {
                if (buf.m.userptr == (unsigned long)_buffers[i].start 
                    && buf.length == _buffers[i].length) {
                    break;
                }
            }

            assert(i < _n_buffers);
            len = buf.bytesused;
            result = process_image((void *)buf.m.userptr, len, _image);
            if (!result) {
                return 0;
            }

            if (-1 == xioctl(_fd, VIDIOC_QBUF, &buf)) {
                errno_exit("VIDIOC_QBUF");
            }

            break;
    }

    return 1;
}

bool UsbCam::is_capturing() {
    return _is_capturing;
}

void UsbCam::stop_capturing(void)
{
    if (!_is_capturing)
    {
        return;
    }

    _is_capturing = false;
    enum v4l2_buf_type type;

    switch (_io_type)
    {
        case IO_METHOD_READ:
            /* Nothing to do. */
            break;

        case IO_METHOD_MMAP:
        case IO_METHOD_USERPTR:
            type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

            if (-1 == xioctl(_fd, VIDIOC_STREAMOFF, &type)) {
                errno_exit("VIDIOC_STREAMOFF");
            }
            break;
    }
}

void UsbCam::start_capturing(void)
{
    if (_is_capturing)
    {
        return;
    }

    unsigned int i = 0;
    enum v4l2_buf_type type;

    switch (_io_type)
    {
        case IO_METHOD_READ:
            /* Nothing to do. */
            break;

        case IO_METHOD_MMAP:
            for (i = 0; i < _n_buffers; ++i)
            {
                struct v4l2_buffer buf;

                CLEAR(buf);

                buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory = V4L2_MEMORY_MMAP;
                buf.index = i;

                if (-1 == xioctl(_fd, VIDIOC_QBUF, &buf))
                {
                    errno_exit("VIDIOC_QBUF");
                }
            }

            type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

            if (-1 == xioctl(_fd, VIDIOC_STREAMON, &type))
            {
                errno_exit("VIDIOC_STREAMON");
            }

            break;

        case IO_METHOD_USERPTR:
            for (i = 0; i < _n_buffers; ++i)
            {
                struct v4l2_buffer buf;

                CLEAR(buf);

                buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory = V4L2_MEMORY_USERPTR;
                buf.index = i;
                buf.m.userptr = (unsigned long)_buffers[i].start;
                buf.length = _buffers[i].length;

                if (-1 == xioctl(_fd, VIDIOC_QBUF, &buf))
                {
                    errno_exit("VIDIOC_QBUF");
                }
            }

            type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

            if (-1 == xioctl(_fd, VIDIOC_STREAMON, &type))
            {
                errno_exit("VIDIOC_STREAMON");
            }

            break;
    }

    _is_capturing = true;
}

void UsbCam::uninit_device(void)
{
    unsigned int i = 0;

    switch (_io_type)
    {
        case IO_METHOD_READ:
            if (_buffers[0].start) {
                delete [] (unsigned char*)_buffers[0].start;
                _buffers[0].start = NULL;
            }
            break;

        case IO_METHOD_MMAP:
            for (i = 0; i < _n_buffers; ++i) {
                if (-1 == munmap(_buffers[i].start, _buffers[i].length)) {
                    errno_exit("munmap");
                }
            }
            break;

        case IO_METHOD_USERPTR:
            for (i = 0; i < _n_buffers; ++i) {
                if (_buffers[i].start) {
                    free(_buffers[i].start);
                    _buffers[i].start = NULL;
                }
            }
            break;
    }
}

void UsbCam::init_read(unsigned int buffer_size)
{
    //buffers_ = (buffer*)calloc(1, sizeof(*buffers_));
    _buffers.resize(1);

    _buffers[0].length = buffer_size;
    _buffers[0].start = (void*)new unsigned char[buffer_size];

    if (!_buffers[0].start)
    {
        ROS_ERROR("Out of memory");
        exit(EXIT_FAILURE);
    }
}

void UsbCam::init_mmap(void)
{
    struct v4l2_requestbuffers req;

    CLEAR(req);

    req.count = 1;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (-1 == xioctl(_fd, VIDIOC_REQBUFS, &req))
    {
        if (EINVAL == errno)
        {
            ROS_ERROR_STREAM(_camera_dev << " does not support memory mapping");
            exit(EXIT_FAILURE);
        }
        else
        {
            errno_exit("VIDIOC_REQBUFS");
        }
    }

    //if (req.count < 2)
    //{
    //    ROS_ERROR_STREAM("Insufficient buffer memory on " << _camera_dev);
    //    exit(EXIT_FAILURE);
    //}

    //buffers_ = (buffer*)calloc(req.count, sizeof(*buffers_));
    _buffers.resize(req.count);

    for (_n_buffers = 0; _n_buffers < req.count; ++_n_buffers)
    {
        struct v4l2_buffer buf;

        CLEAR(buf);

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = _n_buffers;

        if (-1 == xioctl(_fd, VIDIOC_QUERYBUF, &buf)) {
            errno_exit("VIDIOC_QUERYBUF");
        }

        _buffers[_n_buffers].length = buf.length;
        _buffers[_n_buffers].start = mmap(NULL /* start anywhere */, buf.length, 
                PROT_READ | PROT_WRITE /* required */,
                MAP_SHARED /* recommended */,
                _fd, buf.m.offset);

        if (MAP_FAILED == _buffers[_n_buffers].start) {
            errno_exit("mmap");
        }
    }
}

void UsbCam::init_userp(unsigned int buffer_size)
{
    struct v4l2_requestbuffers req;
    unsigned int page_size = 0;

    page_size = getpagesize();
    buffer_size = (buffer_size + page_size - 1) & ~(page_size - 1);

    CLEAR(req);

    req.count = 4;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_USERPTR;

    if (-1 == xioctl(_fd, VIDIOC_REQBUFS, &req))
    {
        if (EINVAL == errno)
        {
            ROS_ERROR_STREAM(_camera_dev << " does not support "
                    "user pointer i/o");
            exit(EXIT_FAILURE);
        }
        else
        {
            errno_exit("VIDIOC_REQBUFS");
        }
    }

    //buffers_ = (buffer*)calloc(4, sizeof(*buffers_));
    _buffers.resize(4);

    for (_n_buffers = 0; _n_buffers < 4; ++_n_buffers)
    {
        _buffers[_n_buffers].length = buffer_size;
        _buffers[_n_buffers].start = memalign(/* boundary */page_size, buffer_size);

        if (!_buffers[_n_buffers].start)
        {
            ROS_ERROR("Out of memory");
            exit(EXIT_FAILURE);
        }
    }
}

void UsbCam::init_device(int image_width, int image_height, int framerate)
{
    struct v4l2_capability cap;
    struct v4l2_cropcap cropcap;
    struct v4l2_crop crop;
    struct v4l2_format fmt;
    unsigned int min = 0;

    if (-1 == xioctl(_fd, VIDIOC_QUERYCAP, &cap))
    {
        if (EINVAL == errno)
        {
            ROS_ERROR_STREAM(_camera_dev << " is no V4L2 device");
            exit(EXIT_FAILURE);
        }
        else
        {
            errno_exit("VIDIOC_QUERYCAP");
        }
    }

    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
    {
        ROS_ERROR_STREAM(_camera_dev << " is no video capture device");
        exit(EXIT_FAILURE);
    }

    switch (_io_type)
    {
        case IO_METHOD_READ:
            if (!(cap.capabilities & V4L2_CAP_READWRITE))
            {
                ROS_ERROR_STREAM(_camera_dev << " does not support read i/o");
                exit(EXIT_FAILURE);
            }

            break;

        case IO_METHOD_MMAP:
        case IO_METHOD_USERPTR:
            if (!(cap.capabilities & V4L2_CAP_STREAMING))
            {
                ROS_ERROR_STREAM(_camera_dev << " does not support streaming i/o");
                exit(EXIT_FAILURE);
            }

            break;
    }

    /* Select video input, video standard and tune here. */

    CLEAR(cropcap);

    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (0 == xioctl(_fd, VIDIOC_CROPCAP, &cropcap))
    {
        crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        crop.c = cropcap.defrect; /* reset to default */

        if (-1 == xioctl(_fd, VIDIOC_S_CROP, &crop))
        {
            switch (errno)
            {
                case EINVAL:
                    /* Cropping not supported. */
                    break;
                default:
                    /* Errors ignored. */
                    break;
            }
        }
    }
    else
    {
        /* Errors ignored. */
    }

    CLEAR(fmt);

    //  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    //  fmt.fmt.pix.width = 640;
    //  fmt.fmt.pix.height = 480;
    //  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    //  fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;

    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = image_width;
    fmt.fmt.pix.height = image_height;
    fmt.fmt.pix.pixelformat = _pixelformat;
    fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;

    if (-1 == xioctl(_fd, VIDIOC_S_FMT, &fmt))
    {
        //reset_device();
        errno_exit("VIDIOC_S_FMT");
    }

    /* Note VIDIOC_S_FMT may change width and height. */

    /* Buggy driver paranoia. */
    min = fmt.fmt.pix.width * 2;
    if (fmt.fmt.pix.bytesperline < min)
    {
        fmt.fmt.pix.bytesperline = min;
    }
    min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
    if (fmt.fmt.pix.sizeimage < min)
    {
        fmt.fmt.pix.sizeimage = min;
    }

    image_width = fmt.fmt.pix.width;
    image_height = fmt.fmt.pix.height;

    struct v4l2_streamparm stream_params;
    memset(&stream_params, 0, sizeof(stream_params));
    stream_params.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(_fd, VIDIOC_G_PARM, &stream_params) < 0)
    {
        errno_exit("Couldn't query v4l fps!");
    }

    ROS_DEBUG("Capability flag: 0x%x", stream_params.parm.capture.capability);

    stream_params.parm.capture.timeperframe.numerator = 1;
    stream_params.parm.capture.timeperframe.denominator = framerate;
    if (xioctl(_fd, VIDIOC_S_PARM, &stream_params) < 0)
    {
        ROS_WARN("Couldn't set camera framerate");
    }
    else
    {
        ROS_DEBUG("Set framerate to be %i", framerate);
    }

    switch (_io_type)
    {
        case IO_METHOD_READ:
            init_read(fmt.fmt.pix.sizeimage);
            break;

        case IO_METHOD_MMAP:
            init_mmap();
            break;

        case IO_METHOD_USERPTR:
            init_userp(fmt.fmt.pix.sizeimage);
            break;
    }
}

void UsbCam::close_device(void)
{
    if (-1 == close(_fd))
    {
        errno_exit("close");
    }

    _fd = -1;
}

void UsbCam::open_device(void)
{
    struct stat st;

    if (-1 == stat(_camera_dev.c_str(), &st))
    {
        ROS_ERROR_STREAM("Cannot identify '" << _camera_dev << "': " << errno << ", " 
                << strerror(errno) << ":" << _error_code);
        exit(EXIT_FAILURE);
    }

    if (!S_ISCHR(st.st_mode))
    {
        ROS_ERROR_STREAM(_camera_dev << " is no device" << ":" << _error_code);
        exit(EXIT_FAILURE);
    }

    _fd = open(_camera_dev.c_str(), O_RDWR/* required */ | O_NONBLOCK, 0);
    if (-1 == _fd)
    {
        ROS_ERROR_STREAM("Cannot open '" << _camera_dev << "': " << errno << ", "
                << strerror(errno) << ":" << _error_code);
        exit(EXIT_FAILURE);
    }
}

void UsbCam::start(const std::string& dev, io_method io_method,
		   pixel_format pixel_format, int image_width, int image_height,
           int framerate)
{
    _camera_dev = dev;

    _io_type = io_method;
    _monochrome = false;
    if (pixel_format == PIXEL_FORMAT_YUYV)
    {
        _pixelformat = V4L2_PIX_FMT_YUYV;
    }
    else if (pixel_format == PIXEL_FORMAT_UYVY)
    {
        _pixelformat = V4L2_PIX_FMT_UYVY;
    }
    else if (pixel_format == PIXEL_FORMAT_MJPEG)
    {
        _pixelformat = V4L2_PIX_FMT_MJPEG;
        init_mjpeg_decoder(image_width, image_height);
    }
    else if (pixel_format == PIXEL_FORMAT_YUVMONO10)
    {
        //actually format V4L2_PIX_FMT_Y16 (10-bit mono expresed as 16-bit pixels), but we need to use the advertised type (yuyv)
        _pixelformat = V4L2_PIX_FMT_YUYV;
        _monochrome = true;
    }
    else if (pixel_format == PIXEL_FORMAT_RGB24)
    {
        _pixelformat = V4L2_PIX_FMT_RGB24;
    }
    else
    {
        ROS_ERROR("Unknown pixel format.");
        exit(EXIT_FAILURE);
    }

    open_device();
    init_device(image_width, image_height, framerate);
    start_capturing();

    // instead of malloc with smart pointer 
    _image = boost::make_shared<CameraImage>();

    _image->width = image_width;
    _image->height = image_height;
    _image->bytes_per_pixel = 2;      //corrected 11/10/15 (BYTES not BITS per pixel)

    _image->image_size = _image->width * _image->height * _image->bytes_per_pixel;
    _image->is_new = 0;
    _image->image = new char[_image->image_size]();
    if (!_image->image)
    {
        ROS_ERROR("Outof memory!");
        exit(EXIT_FAILURE);
    }
}

void UsbCam::shutdown(void)
{
    stop_capturing();
    uninit_device();
    close_device();

    if (_avcodec_context)
    {
        avcodec_close(_avcodec_context);
        av_free(_avcodec_context);
        _avcodec_context = NULL;
    }

    if (_avframe_camera) {
        av_free(_avframe_camera);
        _avframe_camera = NULL;
    }

    if (_avframe_rgb) {
        av_free(_avframe_rgb);
        _avframe_rgb = NULL;
    }

    if (_image) 
    {
        if (_image->image) 
        {
            delete [] _image->image;
            _image->image = NULL;
        }
    }
}

bool UsbCam::grab_image(sensor_msgs::Image* msg, int timeout)
{
    // grab the image
    bool get_new_image = grab_image(timeout);
    if (!get_new_image) {
        return false;
    }
    // stamp the image
    msg->header.stamp.sec = _image->tv_sec;
    msg->header.stamp.nsec = 1000 * _image->tv_usec;
    // fill the info
    if (_monochrome)
    {
        fillImage(*msg, "mono8", _image->height, _image->width, _image->width,
                _image->image);
    }
    else
    {
        //fillImage(*msg, "rgb8", _image->height, _image->width, 3 * _image->width,
        //        _image->image);
        msg->encoding = "yuyv";
        msg->height = _image->height;
        msg->width = _image->width;
        msg->step = 2 * _image->width;
        size_t len = _image->width * _image->height * 2;
        msg->data.resize(len);
        memcpy(&msg->data[0], _image->image, len);
        msg->is_bigendian = 0;
    }
    return true;
}

bool UsbCam::grab_image(int timeout)
{
    fd_set fds;
    struct timeval tv;
    int r = 0;

    FD_ZERO(&fds);
    FD_SET(_fd, &fds);

    /* Timeout. */
    tv.tv_sec = timeout / 1000;
    tv.tv_usec = 0;

    r = select(_fd + 1, &fds, NULL, NULL, &tv);

    if (-1 == r)
    {
        if (EINTR == errno)
        {
            ROS_ERROR("select error EINTR.");
            return false;
        }

        errno_exit("select");
    }

    if (0 == r)
    {
        ROS_WARN_STREAM("camera is offline:" << _camera_dev);
        //reset usb when camera timeout
        //reset_device();
        exit(EXIT_FAILURE);
    }

    int get_new_image = read_frame();
    if (!get_new_image)
    {
        ROS_ERROR("read frame error.");
        return false;
    }

    _image->is_new = 1;
    return true;
}

// enables/disables auto focus
void UsbCam::set_auto_focus(int value)
{
    struct v4l2_queryctrl queryctrl;
    struct v4l2_ext_control control;

    memset(&queryctrl, 0, sizeof(queryctrl));
    queryctrl.id = V4L2_CID_FOCUS_AUTO;

    if (-1 == xioctl(_fd, VIDIOC_QUERYCTRL, &queryctrl))
    {
        if (errno != EINVAL)
        {
            ROS_ERROR("VIDIOC_QUERYCTRL");
            return;
        }
        else
        {
            ROS_INFO("V4L2_CID_FOCUS_AUTO is not supported");
            return;
        }
    }
    else if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)
    {
        ROS_INFO("V4L2_CID_FOCUS_AUTO is not supported");
        return;
    }
    else
    {
        memset(&control, 0, sizeof(control));
        control.id = V4L2_CID_FOCUS_AUTO;
        control.value = value;

        if (-1 == xioctl(_fd, VIDIOC_S_CTRL, &control))
        {
            ROS_ERROR("VIDIOC_S_CTRL");
            return;
        }
    }
}

void UsbCam::set_error_code(const int& value)
{
    _error_code = value;
}
/**
 * Set video device parameter via call to v4l-utils.
 *
 * @param param The name of the parameter to set
 * @param param The value to assign
*/
void UsbCam::set_v4l_parameter(const std::string& param, int value)
{
    set_v4l_parameter(param, boost::lexical_cast<std::string>(value));
}
/**
* Set video device parameter via call to v4l-utils.
*
* @param param The name of the parameter to set
* @param param The value to assign
*/
void UsbCam::set_v4l_parameter(const std::string& param, const std::string& value)
{
    // build the command
    std::stringstream ss;
    ss << "v4l2-ctl --device=" << _camera_dev << " -c " << param << "=" << value << " 2>&1";
    std::string cmd = ss.str();

    // capture the output
    std::string output;
    int buffer_size = 256;
    char buffer[buffer_size];
    FILE *stream = popen(cmd.c_str(), "r");
    if (stream)
    {
        while (!feof(stream))
        {
            if (fgets(buffer, buffer_size, stream) != NULL)
            {
                output.append(buffer);
            }
        }

        pclose(stream);
        // any output should be an error
        if (output.length() > 0)
        {
            ROS_WARN("%s", output.c_str());
        }
    }
    else 
    {
        ROS_WARN("usb_cam_node could not run '%s'", cmd.c_str());
    }
}

UsbCam::io_method UsbCam::io_method_from_string(const std::string& str)
{
    if (str == "mmap")
    {
        return IO_METHOD_MMAP;
    }
    else if (str == "read")
    {
        return IO_METHOD_READ;
    }
    else if (str == "userptr")
    {
        return IO_METHOD_USERPTR;
    }
    else
    {
        return IO_METHOD_UNKNOWN;
    }
}

UsbCam::pixel_format UsbCam::pixel_format_from_string(const std::string& str)
{
    if (str == "yuyv")
    {
        return PIXEL_FORMAT_YUYV;
    }
    else if (str == "uyvy")
    {
        return PIXEL_FORMAT_UYVY;
    }
    else if (str == "mjpeg")
    {
        return PIXEL_FORMAT_MJPEG;
    }
    else if (str == "yuvmono10")
    {
        return PIXEL_FORMAT_YUVMONO10;
    }
    else if (str == "rgb24")
    {
        return PIXEL_FORMAT_RGB24;
    }
    else
    {
        return PIXEL_FORMAT_UNKNOWN;
    }
}

}
