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

#include <termios.h>

#include "cyber/init.h"

namespace apollo {
namespace cyber {
namespace record {

const uint64_t Player::kSleepIntervalMiliSec = 100;

Player::Player(const PlayParam& play_param, const NodePtr& node,
               const bool preload_fill_buffer_mode)
    : is_initialized_(false),
      is_stopped_(true),
      consumer_(nullptr),
      producer_(nullptr),
      task_buffer_(nullptr) {
  task_buffer_ = std::make_shared<PlayTaskBuffer>();
  consumer_.reset(new PlayTaskConsumer(task_buffer_, play_param.play_rate));
  producer_.reset(new PlayTaskProducer(task_buffer_, play_param, node,
                                       preload_fill_buffer_mode));
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

  is_initialized_.store(false);
  return false;
}

static char Getch() {
  char buf = 0;
  struct termios old = {0};
  fflush(stdout);
  if (tcgetattr(0, &old) < 0) {
    perror("tcsetattr()");
  }
  old.c_lflag &= ~ICANON;
  old.c_lflag &= ~ECHO;
  old.c_cc[VMIN] = 0;
  old.c_cc[VTIME] = 1;
  if (tcsetattr(0, TCSANOW, &old) < 0) {
    perror("tcsetattr ICANON");
  }
  if (read(0, &buf, 1) < 0) {
    perror("read()");
  }
  old.c_lflag |= ICANON;
  old.c_lflag |= ECHO;
  if (tcsetattr(0, TCSADRAIN, &old) < 0) {
    perror("tcsetattr ~ICANON");
  }
  return buf;
}

bool Player::ThreadFunc_Play_Nohup() {
  if (!is_initialized_.load()) {
    AERROR << "please call Init firstly.";
    return false;
  }

  if (!is_stopped_.exchange(false)) {
    AERROR << "player has been stopped.";
    return false;
  }
  auto& play_param = producer_->play_param();
  producer_->Start();
  consumer_->Start(play_param.begin_time_ns);
  const double total_progress_time_s =
      static_cast<double>(play_param.end_time_ns - play_param.begin_time_ns) /
          1e9 +
      static_cast<double>(play_param.start_time_s);
  while (!is_stopped_.load() && apollo::cyber::OK()) {
    if (is_paused_) {
      consumer_->Pause();
    } else {
      consumer_->Continue();
    }

    double progress_time_s =
        static_cast<double>(producer_->play_param().start_time_s);
    if (consumer_->last_played_msg_real_time_ns() > 0) {
      progress_time_s +=
          static_cast<double>(consumer_->last_played_msg_real_time_ns() -
                              consumer_->base_msg_play_time_ns() +
                              consumer_->base_msg_real_time_ns() -
                              producer_->play_param().begin_time_ns) /
          1e9;
    }

    producer_->WriteRecordProgress(progress_time_s, total_progress_time_s);

    if (producer_->is_stopped() && task_buffer_->Empty()) {
      consumer_->Stop();
      break;
    }
    std::this_thread::sleep_for(
        std::chrono::milliseconds(kSleepIntervalMiliSec));
  }
  return true;
}

void Player::HandleNohupThreadStatus() { is_paused_ = !is_paused_; }

void Player::ThreadFunc_Term() {
  while (!is_stopped_.load()) {
    char ch = Getch();
    switch (ch) {
      case 's':
        is_playonce_ = true;
        break;
      case ' ':
        is_paused_ = !is_paused_;
        break;
      default:
        break;
    }
  }
}

void Player::NohupPlayRecord() {
  nohup_play_th_.reset(new std::thread(&Player::ThreadFunc_Play_Nohup, this));
}

bool Player::PreloadPlayRecord(const double& progress_s, bool paused_status) {
  if (!producer_->is_initialized()) {
    return false;
  }
  if (is_preloaded_.load()) {
    producer_->Reset(progress_s);
    is_preloaded_.store(false);
  }
  if (progress_s == 0) {
    // When the progress is 0, it is completely reloaded. At this
    // time, the is_paused_ of the player should change to the initial
    // state to avoid being disturbed by the last state.On the contrary,
    // reset the progress needs to preserve the past state to ensure the
    // unity of the state before and after.
    is_paused_.exchange(false);
  } else {
    is_paused_.exchange(paused_status);
  }
  producer_->FillPlayTaskBuffer();
  is_preloaded_.store(true);
  return true;
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

  auto& play_param = producer_->play_param();
  std::cout << "\nPlease wait " << play_param.preload_time_s
            << " second(s) for loading...\n"
            << "Hit Ctrl+C to stop, Space to pause, or 's' to step.\n"
            << std::endl;
  producer_->Start();

  auto preload_sec = play_param.preload_time_s;
  while (preload_sec > 0 && !is_stopped_.load() && apollo::cyber::OK()) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    --preload_sec;
  }

  auto delay_sec = play_param.delay_time_s;
  while (delay_sec > 0 && !is_stopped_.load() && apollo::cyber::OK()) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    --delay_sec;
  }

  consumer_->Start(play_param.begin_time_ns);

  std::ios::fmtflags before(std::cout.flags());
  std::cout << std::fixed;
  const double total_progress_time_s =
      static_cast<double>(play_param.end_time_ns - play_param.begin_time_ns) /
          1e9 +
      static_cast<double>(play_param.start_time_s);

  term_thread_.reset(new std::thread(&Player::ThreadFunc_Term, this));
  while (!is_stopped_.load() && apollo::cyber::OK()) {
    if (is_playonce_) {
      consumer_->PlayOnce();
      is_playonce_ = false;
    }
    if (is_paused_) {
      consumer_->Pause();
      std::cout << "\r[PAUSED ] Record Time: ";
    } else {
      consumer_->Continue();
      std::cout << "\r[RUNNING] Record Time: ";
    }

    double last_played_msg_real_time_s =
        static_cast<double>(consumer_->last_played_msg_real_time_ns()) / 1e9;

    double progress_time_s =
        static_cast<double>(producer_->play_param().start_time_s);
    if (consumer_->last_played_msg_real_time_ns() > 0) {
      progress_time_s +=
          static_cast<double>(consumer_->last_played_msg_real_time_ns() -
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
  producer_->Stop();
  consumer_->Stop();
  if (term_thread_ != nullptr && term_thread_->joinable()) {
    term_thread_->join();
    term_thread_ = nullptr;
  }
  return true;
}

bool Player::Reset() {
  if (is_stopped_.exchange(true)) {
    return false;
  }
  // produer may not be stopped under reset progress
  // reset is_stopped_ to true to ensure logical unity
  producer_->set_stopped();
  producer_->Stop();
  consumer_->Stop();
  // clear task buffer for refill it
  task_buffer_->Clear();
  if (nohup_play_th_ != nullptr && nohup_play_th_->joinable()) {
    nohup_play_th_->join();
    nohup_play_th_ = nullptr;
  }
  return true;
}

}  // namespace record
}  // namespace cyber
}  // namespace apollo
