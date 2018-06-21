/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include <stdio.h>
#include <unistd.h>

#include <jpeglib.h>
#include <vector>

#include "src/pandora_camera.h"
#include "src/pandora_client.h"

namespace apollo {
namespace drivers {
namespace hesai {

cv::Size HesaiLidarSDK_IMAGE_SIZE(IMAGE_WIDTH, IMAGE_HEIGHT);

static int CameraClientCallback(void *handle, int cmd, void *param,
                                void *userp) {
  PandoraPic *pic = static_cast<PandoraPic *>(param);
  PandoraCamera *pSDK = static_cast<PandoraCamera *>(userp);
  pSDK->pushPicture(pic);
  return 0;
}

PandoraCamera::PandoraCamera(
    std::string device_ip, const uint16_t pandoraCameraPort,
    boost::function<void(boost::shared_ptr<cv::Mat> matp, double timestamp,
                         int picid, bool distortion)>
        camera_callback,
    boost::function<void(bool connected)> connectionChanged, int tz) {
  ip_ = device_ip;
  sem_init(&pic_sem_, 0, 0);
  pthread_mutex_init(&pic_lock_, NULL);
  process_pic_thread_ = NULL;
  continue_process_pic_ = false;
  need_remap_ = false;
  camera_port_ = pandoraCameraPort;
  pandora_client_ = NULL;
  camera_callback_ = camera_callback;
  connection_changed_ = connectionChanged;
  tz_second_ = tz * 3600;
}

PandoraCamera::~PandoraCamera() {
  Stop();
  sem_destroy(&pic_sem_);
  pthread_mutex_destroy(&pic_lock_);
}

int PandoraCamera::Start() {
  continue_process_pic_ = true;
  pandora_client_ =
      PandoraClientNew(ip_.c_str(), camera_port_, CameraClientCallback,
                       static_cast<void *>(this));
  if (!pandora_client_) {
    continue_process_pic_ = false;
    return -1;
  }
  process_pic_thread_ =
      new boost::thread(boost::bind(&PandoraCamera::processPic, this));
  if (!process_pic_thread_) {
    continue_process_pic_ = false;
    PandoraClientDestroy(pandora_client_);
    return -1;
  }
  return 0;
}

void PandoraCamera::Stop() {
  continue_process_pic_ = false;
  if (pandora_client_) {
    PandoraClientDestroy(pandora_client_);
  }
  pandora_client_ = NULL;

  if (process_pic_thread_) {
    process_pic_thread_->join();
    delete process_pic_thread_;
  }
  process_pic_thread_ = NULL;
}

void PandoraCamera::pushPicture(PandoraPic *pic) {
  pthread_mutex_lock(&pic_lock_);
  pic_list_.push_back(pic);
  pthread_mutex_unlock(&pic_lock_);
  sem_post(&pic_sem_);
}

void PandoraCamera::processPic() {
  while (continue_process_pic_) {
    struct timespec ts;
    if (clock_gettime(CLOCK_REALTIME, &ts) == -1) {
      printf("processPic, get time error\n");
    }

    ts.tv_sec += 1;
    if (sem_timedwait(&pic_sem_, &ts) == -1) {
      continue;
    }

    pthread_mutex_lock(&pic_lock_);
    PandoraPic *pic = pic_list_.front();
    pic_list_.pop_front();
    pthread_mutex_unlock(&pic_lock_);

    if (pic == NULL) {
      printf("pic is NULL\n");
      return;
    }

    struct tm t;
    t.tm_sec = pic->header.UTC_Time.UTC_Second;
    t.tm_min = pic->header.UTC_Time.UTC_Minute;
    t.tm_hour = pic->header.UTC_Time.UTC_Hour;
    t.tm_mday = pic->header.UTC_Time.UTC_Day;
    t.tm_mon = pic->header.UTC_Time.UTC_Month - 1;
    t.tm_year = pic->header.UTC_Time.UTC_Year + 2000 - 1900;
    t.tm_isdst = 0;
    double timestamp = mktime(&t) + pic->header.timestamp / 1000000.0;
    boost::shared_ptr<cv::Mat> cvMatPic(new cv::Mat());
    switch (pic->header.pic_id) {
      case 0:
        yuv422ToCvmat(cvMatPic, pic->yuv, pic->header.width, pic->header.height,
                      8);
        if (need_remap_)
          remap(cvMatPic->clone(), *cvMatPic, mapx_[pic->header.pic_id],
                mapy_[pic->header.pic_id], CV_INTER_LINEAR);
        break;
      case 1:
      case 2:
      case 3:
      case 4:
        uint8_t *bmp;
        uint32_t bmpSize;
        decompressJpeg(static_cast<uint8_t *>(pic->yuv), pic->header.len, &bmp,
                       &bmpSize);

        *cvMatPic = cv::Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3, bmp).clone();
        if (need_remap_)
          remap(cvMatPic->clone(), *cvMatPic, mapx_[pic->header.pic_id],
                mapy_[pic->header.pic_id], CV_INTER_LINEAR);

        free(bmp);
        break;

      default:
        free(pic->yuv);
        free(pic);
        printf("wrong pic id\n");
        return;
    }
    if (camera_callback_) {
      // Add 2 seconds to correct the timestamp, Reason? No reason.
      // liuxingwei@hesaitech.com
      camera_callback_(cvMatPic, timestamp + 2 + tz_second_, pic->header.pic_id,
                       need_remap_);
    }
    free(pic->yuv);
    pic->yuv = NULL;
    free(pic);
    pic = NULL;
  }
}

bool PandoraCamera::loadIntrinsics(const std::vector<cv::Mat> cameras_k,
                                   const std::vector<cv::Mat> cameras_d) {
  for (int i = 0; i < CAMERA_NUM; i++) {
    cv::Mat mapx = cv::Mat(HesaiLidarSDK_IMAGE_SIZE, CV_32FC1);
    cv::Mat mapy = cv::Mat(HesaiLidarSDK_IMAGE_SIZE, CV_32FC1);
    cv::Mat R = cv::Mat::eye(3, 3, CV_32F);
    initUndistortRectifyMap(cameras_k[i], cameras_d[i], R, cameras_k[i],
                            HesaiLidarSDK_IMAGE_SIZE, CV_32FC1, mapx, mapy);
    mapx_.push_back(mapx);
    mapy_.push_back(mapy);
  }
  need_remap_ = true;
  return true;
}

void PandoraCamera::yuvToRgb(const int iY, const int iU, const int iV, int *iR,
                             int *iG, int *iB) {
  assert(&iR != NULL && &iG != NULL && &iB != NULL);

  *iR = iY + 1.13983 * (iV - 128);
  *iG = iY - 0.39465 * (iU - 128) - 0.58060 * (iV - 128);
  *iB = iY + 2.03211 * (iU - 128);

  *iR = *iR > 255 ? 255 : *iR;
  *iR = *iR < 0 ? 0 : *iR;

  *iG = *iG > 255 ? 255 : *iG;
  *iG = *iG < 0 ? 0 : *iG;

  *iB = *iB > 255 ? 255 : *iB;
  *iB = *iB < 0 ? 0 : *iB;
}

void PandoraCamera::yuv422ToRgb24(const unsigned char *uyvy422,
                                  unsigned char *rgb24, const int width,
                                  const int height) {
  int iR, iG, iB;
  int iY0, iY1, iU, iV;
  int i = 0;
  int j = 0;
  for (i = 0; i < width * height * 2; i += 4) {
    iU = uyvy422[i + 0];
    iY0 = uyvy422[i + 1];
    iV = uyvy422[i + 2];
    iY1 = uyvy422[i + 3];

    yuvToRgb(iY0, iU, iV, &iR, &iG, &iB);
    rgb24[j++] = iB;
    rgb24[j++] = iG;
    rgb24[j++] = iR;
    yuvToRgb(iY1, iU, iV, &iR, &iG, &iB);
    rgb24[j++] = iB;
    rgb24[j++] = iG;
    rgb24[j++] = iR;
  }
}

void PandoraCamera::yuv422ToCvmat(boost::shared_ptr<cv::Mat> dst,
                                  const void *pYUV422, const int nWidth,
                                  const int nHeight, const int bitDepth) {
  if (!pYUV422) {
    return;
  }
  unsigned char *rgb24_buffer = new unsigned char[nWidth * nHeight * 3];
  yuv422ToRgb24((unsigned char *)pYUV422, rgb24_buffer, nWidth, nHeight);
  *dst = cv::Mat(nHeight, nWidth, CV_8UC3, rgb24_buffer).clone();
  delete[] rgb24_buffer;
}

static void my_output_message(j_common_ptr ptr) { return; }

static void print_mem(unsigned char *mem, unsigned int size) {
  int i = 0;
  for (i = 0; i < size; i++) {
    printf("%02x ", mem[i]);
  }
  printf("\n");
}

int PandoraCamera::decompressJpeg(uint8_t *jpgBuffer, const uint32_t jpgSize,
                                  uint8_t **bmp, uint32_t *bmpSize) {
  struct jpeg_decompress_struct cinfo;
  struct jpeg_error_mgr jerr;
  uint8_t *bmpBuffer;
  int rowStride, width, height, pixelSize;

  cinfo.err = jpeg_std_error(&jerr);
  cinfo.err->output_message = my_output_message;
  cinfo.err->error_exit = my_output_message;

  jpeg_create_decompress(&cinfo);
  jpeg_mem_src(&cinfo, jpgBuffer, jpgSize);

  int rc = jpeg_read_header(&cinfo, TRUE);
  if (rc != 1) {
    return -1;
  }

  jpeg_start_decompress(&cinfo);

  width = cinfo.output_width;
  height = cinfo.output_height;
  pixelSize = cinfo.output_components;
  *bmpSize = width * height * pixelSize;
  bmpBuffer = (unsigned char *)malloc(*bmpSize);
  rowStride = width * pixelSize;

  while (cinfo.output_scanline < cinfo.output_height) {
    unsigned char *buffer_array[1];
    buffer_array[0] = bmpBuffer + (cinfo.output_scanline) * rowStride;

    jpeg_read_scanlines(&cinfo, buffer_array, 1);
  }
  *bmp = bmpBuffer;
  jpeg_finish_decompress(&cinfo);
  jpeg_destroy_decompress(&cinfo);
  return 0;
}

}  // namespace hesai
}  // namespace drivers
}  // namespace apollo
