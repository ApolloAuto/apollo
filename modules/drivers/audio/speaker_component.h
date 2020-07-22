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

#pragma

#include "modules/drivers/audio/proto/speaker_config.pb.h"
#include "modules/drivers/audio/respeaker.h"
#include "cyber/cyber.h"

namespace apollo {
namespace drivers {
namespace respeaker {

using apollo::cyber::Component;
using apollo::cyber::Reader;
using apollo::cyber::Writer;
using apollo::drivers::audio::proto::AudioData;  // check if it is package
                                                 // name(proto) / lib name:
                                                 // respeaker_config
using apollo::drivers::audio::proto::ChannelData;
using apollo::drivers::audio::proto::SpeakerConfig;

class SpeakerComponent : public Component<> {
 public:
  bool Init() override;
  ~SpeakerComponent();

 private:
  void run();

  // Configuration
  int sample_rate_, n_channels_, sample_width_, record_seconds_, chunk_;

  // class data
  std::shared_ptr<Writer<AudioData>> writer;
  std::unique_ptr<Respeaker> speaker_device_;
  std::shared_ptr<SpeakerConfig> speaker_config_;
  std::shared_ptr<AudioData> audio_data_;
  std::vector<std::unique_ptr<ChannelData>>* channels_;
  char *buffer_;
  int n_chunks_, chunk_size_, buffer_size_;
};

CYBER_REGISTER_COMPONENT(SpeakerComponent)
}  // namespace respeaker
}  // namespace drivers
}  // namespace apollo
