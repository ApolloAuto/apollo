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

#include "cybertron/tools/cyber_recorder/player.h"

namespace apollo {
namespace cybertron {
namespace record {

TimedPlayer::TimedPlayer(std::shared_ptr<Writer<RawMessage>>& writer,
                         const std::shared_ptr<RawMessage>& raw_message,
                         uint64_t real_time, uint64_t play_time)
    : writer_(writer),
      raw_message_(raw_message),
      real_time_(real_time),
      play_time_(play_time) {}

TimedPlayer::~TimedPlayer() {}

void TimedPlayer::Proc() {
  played_message_number_++;
  if (writer_ == nullptr) {
    AERROR << "writer is nullptr";
    return;
  }
  if (!writer_->Write(raw_message_)) {
    AERROR << "write raw message fail, played count: " << played_message_number_
           << ", real time: " << real_time_ << ", play time: " << play_time_;
  }
  ADEBUG << "write raw message succ, played count: " << played_message_number_
         << ", real time: " << real_time_ << ", play time: " << play_time_;
}

uint64_t TimedPlayer::played_message_number_ = 0;

Player::Player(const std::string& file, bool all_channels,
               const std::vector<std::string>& channel_vec,
               bool looped_playback, float rate, uint64_t begin_time,
               uint64_t end_time, uint64_t start_seconds,
               uint64_t delay_seconds)
    : file_(file),
      all_channels_(all_channels),
      channel_vec_(channel_vec),
      looped_playback_(looped_playback),
      rate_(rate),
      begin_time_(begin_time),
      end_time_(end_time),
      start_seconds_(start_seconds),
      delay_seconds_(delay_seconds) {
  //  setup terminal
  const int fd = fileno(stdin);
  termios flags;
  tcgetattr(fd, &orig_flags_);
  flags = orig_flags_;
  flags.c_lflag &= ~ICANON;  // set raw (unset canonical modes)
  flags.c_cc[VMIN] = 0;   // min 1 char for blocking, 0 chars for non-blocking
  flags.c_cc[VTIME] = 0;  // block if waiting for char
  tcsetattr(fd, TCSANOW, &flags);
  FD_ZERO(&stdin_fdset_);
  FD_SET(fd, &stdin_fdset_);
  maxfd_ = fd + 1;
}

Player::~Player() {
  //  join loader thread
  if (loader_thread_) {
    loader_thread_->join();
  }

  //  clear writer map
  channel_writer_map_.clear();

  //  restore terminal() {
  const int fd = fileno(stdin);
  tcsetattr(fd, TCSANOW, &orig_flags_);
}

bool Player::Stop() {
  if (is_stopped_) {
    return false;
  }
  is_stopped_ = true;
  return true;
}

bool Player::Init() {
  if (is_initialized_) {
    AERROR << "already initialization";
    return false;
  }
  if (!InitInfileopt()) {
    AERROR << "init infileopt error.";
    return false;
  }
  if (!InitWriters()) {
    AERROR << "init writer error.";
    return false;
  }
  if (delay_seconds_ > 0) {
    sleep(delay_seconds_);
  }
  if (!InitLoader()) {
    AERROR << "init loader error.";
    return false;
  }
  is_initialized_ = true;
  return true;
}

bool Player::InitInfileopt() {
  if (!infileopt_.Open(file_)) {
    AERROR << "open recorder file error. file: " << file_;
    return false;
  }

  // get header
  infileopt_.ReadHeader();
  header_ = infileopt_.GetHeader();
  std::cout << "file: " << file_ << ", chunk_number: " << header_.chunk_number()
            << ", begin_time: " << header_.begin_time()
            << ", end_time: " << header_.end_time()
            << ", message_number: " << header_.message_number() << std::endl;

  // get index
  infileopt_.ReadIndex();
  index_ = infileopt_.GetIndex();
  for (int i = 0; i < index_.indexes_size(); i++) {
    SingleIndex* single_index = index_.mutable_indexes(i);
    if (single_index->type() == SectionType::SECTION_CHANNEL) {
      if (!single_index->has_channel_cache()) {
        AERROR << "single channel index do not have channel_cache field.";
        continue;
      }
      ChannelCache* channel_cache = single_index->mutable_channel_cache();

      ProtobufFactory::Instance()->RegisterMessage(channel_cache->proto_desc());

      channel_message_type_map_[channel_cache->name()] =
          channel_cache->message_type();

      if (all_channels_ ||
          std::find(channel_vec_.begin(), channel_vec_.end(),
                    channel_cache->name()) != channel_vec_.end()) {
        total_message_number_ +=
            single_index->mutable_channel_cache()->message_number();
      }
    }
  }
  ADEBUG << "total_message_number_: " << total_message_number_;
  return true;
}

bool Player::InitWriters() {
  std::string node_name = "cyber_recorder_play_" + std::to_string(getpid());
  node_ = ::apollo::cybertron::CreateNode(node_name);
  if (node_ == nullptr) {
    AERROR << "node init error.";
    return false;
  }

  for (auto& pair : channel_message_type_map_) {
    std::string channel_name = pair.first;
    std::string message_type = pair.second;

    if (all_channels_ ||
        std::find(channel_vec_.begin(), channel_vec_.end(), channel_name) !=
            channel_vec_.end()) {
      RoleAttributes role_attributes;
      role_attributes.set_channel_name(channel_name);
      role_attributes.set_message_type(message_type);
      auto writer = node_->CreateWriter<RawMessage>(role_attributes);
      if (writer == nullptr) {
        AERROR << "create wirter failed. channel name: " << channel_name
               << ", message type: " << message_type;
        return false;
      }
      channel_writer_map_[channel_name] = writer;
      ADEBUG << "create writer succeed. channel name: " << channel_name
             << ", message type:" << message_type;
    }
  }
  return true;
}

bool Player::InitLoader() {
  if (start_seconds_ > 0) {
    begin_time_ += start_seconds_ * 1e6;
  }
  if (begin_time_ < header_.begin_time()) {
    begin_time_ = header_.begin_time();
  }
  if (end_time_ > header_.end_time()) {
    end_time_ = header_.end_time();
  }
  if (begin_time_ >= end_time_) {
    AERROR << "begin time are equal or larger than end time"
           << ", begin_time_=" << begin_time_ << ", end_time_=" << end_time_;
    return false;
  }
  auto f = std::bind(&Player::LoadChunk, this, std::placeholders::_1,
                     std::placeholders::_2);
  loader_thread_ = std::make_shared<std::thread>(f, begin_time_, end_time_);
  if (loader_thread_ == nullptr) {
    AERROR << "init loader thread error.";
    return false;
  }
  return true;
}

bool Player::LoadChunk(uint64_t from_time, uint64_t to_time) {
  ADEBUG << "start to load chunk";

  uint64_t loop_time = end_time_ - begin_time_;

  uint64_t average_period = 0;
  if (total_message_number_ > 0) {
    average_period = loop_time / total_message_number_;
  }

  double average_hz = total_message_number_ / (loop_time * 1e-9);

  uint32_t preload_queue_size = (uint32_t)average_hz * preload_sec_;
  if (preload_queue_size < min_queue_size_) {
    preload_queue_size = min_queue_size_;
  }

  ADEBUG << "begin_time_: " << begin_time_ << ", end_time_: " << end_time_
         << ", total_message_number_: " << total_message_number_
         << ", average_period: " << average_period
         << ", from_time: " << from_time << ", to_time: " << to_time
         << ", average_hz: " << average_hz
         << ", preload_queue_size: " << preload_queue_size;

  uint32_t loop_num = 0;
  while (::apollo::cybertron::OK()) {
    ADEBUG << "new loop started";
    uint32_t chunk_idx = 0;
    infileopt_.ReadHeader();
    while (::apollo::cybertron::OK()) {
      if (chunk_idx == header_.chunk_number()) {
        ADEBUG << "played all chunk";
        break;
      }
      while (::apollo::cybertron::OK() &&
             task_queue_.size() >= preload_queue_size) {
        ADEBUG << "sleep " << average_period << "ns, for waiting task queue.";
        std::this_thread::sleep_for(std::chrono::nanoseconds(average_period));
      }
      Section section;
      if (!infileopt_.ReadSection(&section)) {
        AERROR << "read section fail";
        return false;
      }
      if (section.type == SectionType::SECTION_INDEX) {
        break;
      }
      switch (section.type) {
        case SectionType::SECTION_CHANNEL: {
          Channel channel;
          if (!infileopt_.ReadSection<Channel>(section.size, &channel)) {
            AERROR << "read message fail.";
            return false;
          }
          break;
        }
        case SectionType::SECTION_CHUNK_HEADER: {
          ChunkHeader chunk_header;
          if (!infileopt_.ReadSection<ChunkHeader>(section.size,
                                                   &chunk_header)) {
            AERROR << "read message fail.";
            return false;
          }
          if (from_time > chunk_header.end_time() ||
              to_time < chunk_header.begin_time()) {
            chunk_idx++;
            continue;
          }
          if (!infileopt_.ReadSection(&section)) {
            AERROR << "read section fail.";
            return false;
          }
          ChunkBody chunk_body;
          if (!infileopt_.ReadSection<ChunkBody>(section.size, &chunk_body)) {
            AERROR << "read message fail.";
            return false;
          }
          ADEBUG << "chunk_idx: " << chunk_idx
                 << ", begin_time: " << chunk_header.begin_time()
                 << ", end_time: " << chunk_header.end_time()
                 << ", message_number: " << chunk_header.message_number();
          for (int idx = 0; idx < chunk_body.messages_size(); ++idx) {
            if (!all_channels_ &&
                std::find(channel_vec_.begin(), channel_vec_.end(),
                          chunk_body.messages(idx).channel_name()) ==
                    channel_vec_.end()) {
              continue;
            }
            if (chunk_body.messages(idx).time() < from_time ||
                chunk_body.messages(idx).time() > to_time) {
              continue;
            }
            if (!GenerateTask(chunk_body.messages(idx), loop_num * loop_time)) {
              AERROR << "generate task failed!";
              return false;
            }
          }
          chunk_idx++;
          break;
        }
        default: {
          AERROR << "section should not be here, section type: "
                 << section.type;
          break;
        }
      }  // end switch for section type
    }    // end while for one loop
    if (looped_playback_) {
      loop_num++;
      ADEBUG << "load chunk to the end. continue new loop. loop_num: "
             << loop_num;
      continue;
    }
    break;
  }  // end while for one file
  ADEBUG << "load chunk finished, task count: " << task_count_;
  load_finished_ = true;
  return true;
}

bool Player::GenerateTask(const SingleMessage& msg, const uint64_t& plus_time) {
  if (channel_writer_map_.find(msg.channel_name()) ==
      channel_writer_map_.end()) {
    AERROR << "channel[" << msg.channel_name() << "] type be null error";
    return false;
  }
  std::shared_ptr<RawMessage> proto(new RawMessage(msg.content()));
  std::shared_ptr<TimedPlayer> task(
      new TimedPlayer(channel_writer_map_[msg.channel_name()], proto,
                      msg.time(), msg.time() + plus_time));
  std::lock_guard<std::mutex> lck(mutex_);
  task_queue_.push(task);
  task_count_++;
  ADEBUG << "push task to queue, task count: " << task_count_
         << ",  plus time: " << plus_time;
  return true;
}

bool Player::Start() {
  if (!is_initialized_) {
    AERROR << "NOT initializated";
    return false;
  }
  if (is_started_) {
    AERROR << "already restarted";
    return false;
  }
  is_started_ = true;
  std::cout << "\nplease wait for loading and playing back record...\n"
            << "Hit Ctrl+C to stop replay, or Space to pause.\n"
            << std::endl;
  std::ios::fmtflags before(std::cout.flags());
  std::cout << std::fixed;

  // sleep 5s for preload message
  auto preload_sec = preload_sec_;
  while (preload_sec && ::apollo::cybertron::OK()) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    --preload_sec;
  }

  uint64_t base_tw_time = 0;                          // in nanosecond
  uint64_t pause_time = 0;                            // in nanosecond
  uint64_t period_time = Time::Now().ToNanosecond();  // in nanosecond

  std::thread progress_thread([this] {
    while (::apollo::cybertron::OK()) {
      this->ReadCharFromStdin();
      if (is_paused_) {
        std::cout << "\r[PAUSED ] Record Time: ";
      } else {
        std::cout << "\r[RUNNING] Record Time: ";
      }
      std::cout << std::setprecision(3) << last_played_time_ / 1e9
                << "    Progress: " << std::setprecision(3)
                << (last_played_time_ - base_time_) / 1e9 << " / "
                << std::setprecision(3) << (end_time_ - begin_time_) / 1e9;
      std::cout.flush();
      if (load_finished_ &&
          TimedPlayer::played_message_number_ >= task_count_ &&
          !looped_playback_) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  });

  std::shared_ptr<TimedPlayer> task = nullptr;
  while (is_started_ && ::apollo::cybertron::OK()) {
    if (!task_queue_.empty()) {
      // pop task
      {
        std::lock_guard<std::mutex> lck(mutex_);
        task = task_queue_.front();
        task_queue_.pop();
      }

      // use the first msg's time as the base time
      if (base_time_ == 0) {
        base_time_ = task->play_time_;
        base_tw_time = Time::Now().ToNanosecond();
        ADEBUG << "base_time_: " << base_time_
               << ", base_tw_time: " << base_tw_time;
      }

      int64_t sleep_interval =
          static_cast<double>(task->play_time_ - base_time_) * rate_ -
          static_cast<double>(Time::Now().ToNanosecond() - base_tw_time -
                              pause_time);
      if (sleep_interval > MIN_SLEEP_INTERVAL) {
        std::this_thread::sleep_for(std::chrono::nanoseconds(sleep_interval));
      }
      task->Proc();
      last_played_time_ = task->real_time_;
      ADEBUG << "task_queue_ size: " << task_queue_.size();
    }
    if (load_finished_ && TimedPlayer::played_message_number_ >= task_count_ &&
        !looped_playback_) {
      break;
    }
    while (is_paused_ && ::apollo::cybertron::OK()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      pause_time += 1e8;
    }
  }
  progress_thread.join();
  std::cout << "\nplay finished. file: " << file_ << std::endl;
  std::cout.flags(before);
  ADEBUG << "play finished. file: " << file_;
  return true;
}

void Player::ReadCharFromStdin() {
  fd_set testfd = stdin_fdset_;
  timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 0;
  if (select(maxfd_, &testfd, NULL, NULL, &tv) <= 0) {
    return;
  }
  auto ch = getc(stdin);
  if (ch == EOF) {
    return;
  }
  switch (ch) {
    case ' ':  // pause
      is_paused_ = !is_paused_;
      break;
    default:
      break;
  }
}

}  // namespace record
}  // namespace cybertron
}  // namespace apollo
