/***************************************************************************
Copyright 2018 The Apollo Authors. All Rights Reserved                     /
                                                                            /
Licensed under the Apache License, Version 2.0 (the "License");             /
you may not use this file except in compliance with the License.            /
You may obtain a copy of the License at                                     /
                                                                            /
    http://www.apache.org/licenses/LICENSE-2.0                              /
                                                                            /
Unless required by applicable law or agreed to in writing, software         /
distributed under the License is distributed on an "AS IS" BASIS,           /
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.    /
See the License for the specific language governing permissions and         /
limitations under the License.                                              /
****************************************************************************/

#include <lslidar_decoder/lslidar_decoder_nodelet.h>

namespace apollo {
namespace drivers {
namespace lslidar_decoder {

void LslidarDecoderNodelet::onInit() {
  decoder.reset(new LslidarDecoder(
        getNodeHandle(), getPrivateNodeHandle()));
  if(!decoder->initialize()) {
    ROS_ERROR("Cannot initialize the lslidar puck decoder...");
    return;
  }
  return;
}

} // end namespace lslidar_decoder
}
}
PLUGINLIB_DECLARE_CLASS(lslidar_decoder, LslidarNodelet,
    apollo::drivers::lslidar_decoder::LslidarDecoderNodelet,
    nodelet::Nodelet);
