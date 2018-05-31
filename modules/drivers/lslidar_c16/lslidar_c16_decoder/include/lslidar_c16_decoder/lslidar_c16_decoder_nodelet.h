/***************************************************************************
Copyright 2018 The Leishen Authors. All Rights Reserved                     /
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

#ifndef LSLIDAR_C16_DECODER_NODELET_H
#define LSLIDAR_C16_DECODER_NODELET_H

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <lslidar_c16_decoder/lslidar_c16_decoder.h>

namespace apollo {
namespace drivers {
namespace lslidar_c16_decoder {
class LslidarC16DecoderNodelet: public nodelet::Nodelet {
public:

  LslidarC16DecoderNodelet() {}
  ~LslidarC16DecoderNodelet() {}

private:

  virtual void onInit();
  LslidarC16DecoderPtr decoder;
};

} // end namespace lslidar_n301_decoder
}
}

#endif
