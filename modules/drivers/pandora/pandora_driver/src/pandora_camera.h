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

#ifndef SRC_PANDORA_CAMERA_H_
#define SRC_PANDORA_CAMERA_H_

#include <boost/function.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pthread.h>
#include <semaphore.h>

#include <list>
#include <string>
#include <vector>

#include "src/pandora_client.h"

namespace apollo {
namespace drivers {
namespace hesai {

#define CAMERA_NUM 5
#define IMAGE_WIDTH 1280
#define IMAGE_HEIGHT 720

class PandoraCamera {
 public:
  /**
   * @brief Constructor
   * @param device_ip  				The ip of the device
   *        lidar_port 				The port number of lidar data
   *        gps_port   				The port number of gps data
   *        pcl_callback      The callback of PCL data structure
   *        gps_callback      The callback of GPS structure
   *        type       				The device type
   */
  PandoraCamera(
      std::string device_ip, const uint16_t pandoraCameraPort,
      boost::function<void(boost::shared_ptr<cv::Mat> matp, double timestamp,
                           int picid, bool distortion)>
          camera_callback,
      boost::function<void(bool connected)> connection_changed, int tz);
  ~PandoraCamera();

  /**
   * @brief load the correction file
   * @param angle The start angle
   */

  bool loadIntrinsics(const std::vector<cv::Mat> cameras_k,
                      const std::vector<cv::Mat> cameras_d);

  int Start();
  void Stop();
  void pushPicture(PandoraPic *pic);

 private:
  void processPic();

  int decompressJpeg(uint8_t *jpgBuffer, const uint32_t jpgSize, uint8_t **bmp,
                     uint32_t *bmpSize);
  void yuv422ToCvmat(boost::shared_ptr<cv::Mat> dst, const void *pYUV422,
                     const int nWidth, const int nHeight, const int bitDepth);
  void yuv422ToRgb24(const uint8_t *uyvy422, uint8_t *rgb24, const int width,
                     const int height);
  void yuvToRgb(const int iY, const int iU, const int iV, int *iR, int *iG,
                int *iB);

  pthread_mutex_t pic_lock_;
  sem_t pic_sem_;
  boost::thread *process_pic_thread_;
  bool continue_process_pic_;
  bool need_remap_;
  std::string ip_;
  uint16_t camera_port_;
  void *pandora_client_;
  std::list<PandoraPic *> pic_list_;
  std::vector<cv::Mat> mapx_;
  std::vector<cv::Mat> mapy_;
  boost::function<void(boost::shared_ptr<cv::Mat> matp, double timestamp,
                       int pic_id, bool distortion)>
      camera_callback_;
  boost::function<void(bool connected)> connection_changed_;

  int tz_second_;
};

}  // namespace hesai
}  // namespace drivers
}  // namespace apollo

#endif  // SRC_PANDORA_CAMERA_H_
