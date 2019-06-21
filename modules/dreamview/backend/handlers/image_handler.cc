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

#include "modules/dreamview/backend/handlers/image_handler.h"

#include "cyber/common/log.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/configs/config_gflags.h"

#include "opencv2/opencv.hpp"

namespace apollo {
namespace dreamview {

using apollo::drivers::CompressedImage;
using apollo::drivers::Image;

constexpr double ImageHandler::kImageScale;

template <>
void ImageHandler::OnImage(const std::shared_ptr<Image> &image) {
  if (requests_ == 0) {
    return;
  }

  cv::Mat mat(image->height(), image->width(), CV_8UC3,
              const_cast<char *>(image->data().data()), image->step());
  cv::cvtColor(mat, mat, cv::COLOR_RGB2BGR);
  cv::resize(
      mat, mat,
      cv::Size(static_cast<int>(image->width() * ImageHandler::kImageScale),
               static_cast<int>(image->height() * ImageHandler::kImageScale)),
      0, 0, CV_INTER_LINEAR);

  std::unique_lock<std::mutex> lock(mutex_);
  cv::imencode(".jpg", mat, send_buffer_, std::vector<int>() /* params */);
  cvar_.notify_all();
}

template <>
void ImageHandler::OnImage(
    const std::shared_ptr<CompressedImage> &compressed_image) {
  if (requests_ == 0 ||
      compressed_image->format() == "h265" /* skip video format */) {
    return;
  }

  std::vector<uint8_t> compressed_raw_data(compressed_image->data().begin(),
                                           compressed_image->data().end());
  cv::Mat mat_image = cv::imdecode(compressed_raw_data, CV_LOAD_IMAGE_COLOR);

  std::unique_lock<std::mutex> lock(mutex_);
  cv::imencode(".jpg", mat_image, send_buffer_,
               std::vector<int>() /* params */);
  cvar_.notify_all();
}

void ImageHandler::OnImageFront(const std::shared_ptr<Image> &image) {
  if (FLAGS_use_navigation_mode) {
    // Navigation mode
    OnImage(image);
  }
}

void ImageHandler::OnImageShort(const std::shared_ptr<CompressedImage> &image) {
  if (!FLAGS_use_navigation_mode) {
    // Regular mode
    OnImage(image);
  }
}

ImageHandler::ImageHandler()
    : requests_(0), node_(cyber::CreateNode("image_handler")) {
  node_->CreateReader<Image>(
      FLAGS_image_front_topic,
      [this](const std::shared_ptr<Image> &image) { OnImageFront(image); });

  node_->CreateReader<CompressedImage>(
      FLAGS_image_short_topic,
      [this](const std::shared_ptr<CompressedImage> &image) {
        OnImageShort(image);
      });
}

bool ImageHandler::handleGet(CivetServer *server, struct mg_connection *conn) {
  requests_++;

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

  std::vector<uchar> to_send;
  while (true) {
    {
      std::unique_lock<std::mutex> lock(mutex_);
      to_send = send_buffer_;
    }

    if (!to_send.empty()) {
      // Sends the image data
      mg_printf(conn,
                "--BoundaryString\r\n"
                "Content-type: image/jpeg\r\n"
                "Content-Length: %zu\r\n"
                "\r\n",
                to_send.size());

      if (mg_write(conn, &to_send[0], to_send.size()) <= 0) {
        requests_--;
        return false;
      }
      mg_printf(conn, "\r\n\r\n");
    }

    std::unique_lock<std::mutex> lock(mutex_);
    cvar_.wait(lock);
  }

  requests_--;
  return true;
}

}  // namespace dreamview
}  // namespace apollo
