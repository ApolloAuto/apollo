/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

/**
 * @file
 */

#ifndef MODULES_DREAMVIEW_BACKEND_HANDLERS_IMAGE_HANDLER_H_
#define MODULES_DREAMVIEW_BACKEND_HANDLERS_IMAGE_HANDLER_H_

#include <condition_variable>
#include <mutex>
#include <string>
#include <vector>

#include "cv_bridge/cv_bridge.h"

#include "CivetServer.h"

/**
 * @namespace apollo::dreamview
 * @brief apollo::dreamview
 */
namespace apollo {
namespace dreamview {

/**
 * @class ImageHandler
 *
 * @brief The ImageHandler, built on top of CivetHandler, converts the received
 * ROS image message to a image stream, wrapped by MJPEG Streaming Protocol.
 */
class ImageHandler : public CivetHandler {
 public:
  // The scale used to resize images sent to frontend
  static constexpr double kImageScale = 0.2;

  ImageHandler();

  bool handleGet(CivetServer *server, struct mg_connection *conn);

 private:
  template <typename SensorMsgsImage>
  void OnImage(const SensorMsgsImage &image);

  std::vector<uchar> send_buffer_;

  // mutex lock and condition variable to protect the received image
  std::mutex mutex_;
  std::condition_variable cvar_;
};

}  // namespace dreamview
}  // namespace apollo

#endif /* MODULES_DREAMVIEW_BACKEND_HANDLERS_IMAGE_HANDLER_H_ */
