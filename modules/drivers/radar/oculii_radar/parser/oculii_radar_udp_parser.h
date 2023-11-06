/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#ifndef OCULII_RADAR_PARSER_H_
#define OCULII_RADAR_PARSER_H_

#include <atomic>
#include <future>
#include <utility>
#include <memory>

#include "cyber/base/bounded_queue.h"
#include "modules/common_msgs/sensor_msgs/oculii_radar.pb.h"
#include "modules/drivers/radar/oculii_radar/common/type_def.h"

namespace apollo {
namespace drivers {
namespace radar {

using apollo::cyber::base::BoundedQueue;
using apollo::drivers::OculiiTrackTarget;

class OculiiRadarUdpParser {
 public:
  /**
   * @brief constructor
   */
  OculiiRadarUdpParser();

  /**
   * @brief destructor
   */
  ~OculiiRadarUdpParser();

  /**
   * @brief initialize radar parser instance
   * @return bool initialization status
   */
  bool Init(uint16_t port);

  /**
   * @brief compute coordinate values based on alpha, beta, doppler
   *        and range from point
   * @param oculii_output sensor output
   * @return int computation status
   */
  int Parse(apollo::drivers::OculiiPointCloud& oculii_output);

 private:
  int socket_;
  int64_t frame_number_;
  std::future<void> async_result_;
  std::atomic<bool> running_ = {false};
  std::shared_ptr<uint8_t> raw_buffer_;
  std::shared_ptr<BoundedQueue<
    std::pair<int, std::shared_ptr<uint8_t>>>> queue_;

  void AsyncUdpInput();
  int ParseHeader(OculiiHeader& header, void* buffer);
  int ParseDection(OculiiDetection& dection, void* buffer);
  int ParseTrack(OculiiTrack& track, void* buffer);
  int ParseFooter(OculiiFooter& footer, void* buffer);
  bool IsMultiCastMode(float hr, float hd, float ha, float he,
                      float fr, float fd, float fa, float fe);
};

}  // namespace radar
}  // namespace drivers
}  // namespace apollo

#endif  // OCULII_RADAR_PARSER_H_
