/* Copyright 2017 The Apollo Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/

#include "modules/dreamview/backend/handlers/image.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/common/time/time.h"

#include "opencv2/opencv.hpp"

namespace apollo {
namespace dreamview {

using apollo::common::adapter::AdapterManager;

ImageHandler::ImageHandler() {
  AdapterManager::AddCompressedImageCallback(&ImageHandler::OnImage, this);
}

bool ImageHandler::handleGet(CivetServer *server, struct mg_connection *conn) {
  if (send_buffer_.empty()) {
    return true;
  }

  mg_printf(conn,
            "HTTP/1.1 200 OK\r\n"
            "Connection: close\r\n"
            "Max-Age: 0\r\n"
            "Expires: 0\r\n"
            "Cache-Control: no-cache, no-store, must-revalidate, private\r\n"
            "Pragma: no-cache\r\n"
            "Content-Type: multipart/x-mixed-replace; "
            "boundary=--BoundaryString\r\n"
            "\r\n");

  while (true) {
    std::vector<uchar> to_send;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      to_send = send_buffer_;
    }
    // Sends the image data
    mg_printf(conn,
              "--BoundaryString\r\n"
              "Content-type: image/jpeg\r\n"
              "Content-Length: %zu\r\n"
              "\r\n",
              to_send.size());
    if (mg_write(conn, &to_send[0], to_send.size()) <= 0) {
      return false;
    }
    mg_printf(conn, "\r\n\r\n");

    std::unique_lock<std::mutex> lock(mutex_);
    cvar_.wait(lock);
  }
  return true;
}

void ImageHandler::OnImage(const sensor_msgs::CompressedImage &image) {
  try {
    std::unique_lock<std::mutex> lock(mutex_);
    auto current_image = cv_bridge::toCvCopy(image);
    cv::imencode(".jpg", current_image->image, send_buffer_,
                 std::vector<int>() /* params */);
    cvar_.notify_all();
  } catch (cv_bridge::Exception &e) {
    AERROR << "Error when converting ROS image to CV image: " << e.what();
    return;
  }
}

}  // namespace dreamview
}  // namespace apollo
