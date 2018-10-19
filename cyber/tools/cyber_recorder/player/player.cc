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

#include "cyber/tools/cyber_recorder/player/player.h"

#include <iostream>

#include "cyber/init.h"

namespace apollo {
namespace cyber {
namespace record {

const uint64_t Player::kSleepIntervalMiliSec = 100;

Player::Player(const PlayParam& play_param)
    : is_initialized_(false),
      is_stopped_(true),
      consumer_(nullptr),
      producer_(nullptr),
      term_ctrl_(nullptr),
      task_buffer_(nullptr) {
  term_ctrl_.reset(new TerminalController());
  task_buffer_ = std::make_shared<PlayTaskBuffer>();
  consumer_.reset(new PlayTaskConsumer(task_buffer_, play_param.play_rate));
  producer_.reset(new PlayTaskProducer(task_buffer_, play_param));
}

Player::~Player() { Stop(); }

bool Player::Init() {
  if (is_initialized_.exchange(true)) {
    AERROR << "player has been initialized.";
    return false;
  }

  if (producer_->Init()) {
    return true;
  }

  is_initialized_.exchange(false);
  return false;
}

bool Player::Start() {
  if (!is_initialized_.load()) {
    AERROR << "please call Init firstly.";
    return false;
  }

  if (!is_stopped_.exchange(false)) {
    AERROR << "player has been stopped.";
    return false;
  }

  term_ctrl_->SetUp();
  producer_->Start();
  std::cout << "\nPlease wait for loading...\n"
            << "Hit Ctrl+C to stop, or Space to pause.\n"
            << std::endl;

  auto& play_param = producer_->play_param();
  if (play_param.delay_time_s != 0) {
    std::this_thread::sleep_for(std::chrono::seconds(play_param.delay_time_s));
  }

  consumer_->Start(play_param.begin_time_ns);

  std::ios::fmtflags before(std::cout.flags());
  std::cout << std::fixed;
  bool is_paused = false;
  const double total_progress_time_s =
      (double)(play_param.end_time_ns - play_param.begin_time_ns) / 1e9 +
      (double)play_param.start_time_s;

  while (!is_stopped_.load() && apollo::cyber::OK()) {
    auto character = term_ctrl_->ReadChar();
    switch (character) {
      case ' ':
        is_paused = !is_paused;
        break;

      default:
        break;
    }

    if (is_paused) {
      consumer_->Pause();
      std::cout << "\r[PAUSED ] Record Time: ";
    } else {
      consumer_->Continue();
      std::cout << "\r[RUNNING] Record Time: ";
    }

    double last_played_msg_real_time_s =
        (double)consumer_->last_played_msg_real_time_ns() / 1e9;

    double progress_time_s = (double)producer_->play_param().start_time_s;
    if (consumer_->last_played_msg_real_time_ns() > 0) {
      progress_time_s += (double)(consumer_->last_played_msg_real_time_ns() -
                                  consumer_->base_msg_play_time_ns() +
                                  consumer_->base_msg_real_time_ns() -
                                  producer_->play_param().begin_time_ns) /
                         1e9;
    }

    std::cout << std::setprecision(3) << last_played_msg_real_time_s
              << "    Progress: " << progress_time_s << " / "
              << total_progress_time_s;
    std::cout.flush();

    if (producer_->is_stopped() && task_buffer_->Empty()) {
      consumer_->Stop();
      break;
    }
    std::this_thread::sleep_for(
        std::chrono::milliseconds(kSleepIntervalMiliSec));
  }

  std::cout << "\nplay finished." << std::endl;
  std::cout.flags(before);
  return true;
}

bool Player::Stop() {
  if (is_stopped_.exchange(true)) {
    return false;
  }

  term_ctrl_->TearDown();
  producer_->Stop();
  consumer_->Stop();

  return true;
}

}  // namespace record
}  // namespace cyber
}  // namespace apollo
