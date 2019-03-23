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
#include "modules/drivers/video/input.h"
#include "modules/drivers/video/driver.h"

namespace apollo {
namespace drivers {
namespace video {


using apollo::drivers::video::config::CameraH265Config ;
using apollo::drivers::CompressedImage;

CameraDriver::CameraDriver(CameraH265Config& h265_cfg) 
{
  _config = h265_cfg;
}

bool CameraDriver::poll(
    std::shared_ptr<CompressedImage>& h265)
{
  return poll_by_frame(h265);
}

bool CameraDriver::poll_by_frame(
    std::shared_ptr<CompressedImage>& h265Pb)
{
  while (true) {
    int rc = _input->get_frame_packet(h265Pb);
    if (rc == 0) {
      //const hw_h265_frame_packet *hwPkt = 
      //  (const hw_h265_frame_packet *)h265Pkt->data().c_str();
      //const uint8_t *data = (uint8_t *)hwPkt + sizeof(hw_h265_frame_packet);
      h265Pb->set_frame_id(_config.frame_id());

      uint64_t camera_timestamp = h265Pb->mutable_header()->camera_timestamp();
      uint64_t current_time = cyber::Time().Now().ToNanosecond();
      AINFO << "get frame from port " << _config.udp_port() << 
        "  size = " << h265Pb->data().size() << " ts: camera/host " << 
        camera_timestamp << "/" << current_time << " diff: " <<
        (double)(current_time - camera_timestamp) * 1e-6;

      break;
    } else {
      return false;
    }
  }
  return true;
}

void CameraDriver::Init()
{
  _input.reset(new SocketInput());
  _input->Init(_config.udp_port());
}

} //camera 
} //hwdrivers
} //camera



