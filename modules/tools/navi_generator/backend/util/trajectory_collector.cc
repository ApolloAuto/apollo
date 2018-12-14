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

#include "modules/tools/navi_generator/backend/util/trajectory_collector.h"

#include <sys/statvfs.h>
#include <time.h>
#include <chrono>
#include <cmath>

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/tools/navi_generator/proto/trajectory_collector.pb.h"

using apollo::common::adapter::AdapterManager;
using ros::Time;
using std::set;
using std::shared_ptr;
using std::string;
using std::vector;

namespace apollo {
namespace navi_generator {
namespace util {

// CollectorMessage
CollectorMessage::CollectorMessage(
    const std::string& topic, topic_tools::ShapeShifter::ConstPtr msg,
    boost::shared_ptr<ros::M_string> connection_header, Time time)
    : topic_(topic),
      msg_(msg),
      connection_header_(connection_header),
      time_(time) {}

// CollectorMsgQueue
CollectorMsgQueue::CollectorMsgQueue(const std::string& filename,
                                     std::queue<CollectorMessage>* queue,
                                     Time time)
    : filename_(filename), queue_(queue), time_(time) {}

// CollectorOptions
CollectorOptions::CollectorOptions()
    : split(false),
      append_date(true),
      compression(rosbag::compression::CompressionType::Uncompressed),
      buffer_size(1048576 * 256),
      chunk_size(1024 * 768),
      limit(0),
      max_mileage(0),
      max_splits(0),
      max_size(0),
      min_space(200 * 1024 * 1024),
      max_duration(-1.0),
      max_speed_limit(40.0),
      min_speed_limit(30.0),
      prefix("data/bag/navi_generator"),
      name(""),
      min_space_str("200M") {}
// TrajectoryCollector
TrajectoryCollector::TrajectoryCollector()
    : num_subscribers_(0),
      queue_size_(0),
      split_count_(0),
      writing_enabled_(false),
      initialized_(false),
      is_running_(false),
      last_max_speed_limit(0.0),
      last_min_speed_limit(0.0) {}

// Initialize trajectory collector
bool TrajectoryCollector::Init(const CollectorOptions& options) {
  options_ = options;
  num_subscribers_ = 0;
  queue_size_ = 0;
  split_count_ = 0;
  writing_enabled_ = true;
  initialized_ = true;

  std::string collector_topic = "/apollo/navi_generator/collector";
  ros::NodeHandle nh;
  collector_publisher_ =
      nh.advertise<apollo::navi_generator::TrajectoryCollectorMsg>(
          collector_topic, 100);
  return true;
}
// Start trajectory collector
bool TrajectoryCollector::Start() {
  if (!initialized_) {
    AWARN << "Trajectory collector not initialized.";
    return false;
  }
  if (is_running_) {
    AWARN << "Trajectory collector already running.";
    return false;
  }
  // Make sure topics are specified
  if (options_.topics.size() == 0) {
    AWARN << "No topics specified.";
    return false;
  }
  last_buffer_warn_ = Time();
  queue_ = new std::queue<CollectorMessage>;
  // Subscribe to each topic
  for (const string& topic : options_.topics) {
    AINFO << "Subscribe " << topic.c_str();
    Subscribe(topic);
  }

  ros::Time::waitForValid();
  start_time_ = ros::Time::now();
  // Create a thread for writing to the file
  collect_thread_ =
      std::thread(std::bind(&TrajectoryCollector::CollectData, this));
  // Set a timer to triger publish info periodly
  const double duration = 1.0;
  timer_ = AdapterManager::CreateTimer(ros::Duration(duration),
                                       &TrajectoryCollector::OnTimer, this);
  is_running_ = true;
  return true;
}
// Stop trajectory collector
bool TrajectoryCollector::Stop() {
  if (is_running_) {
    AINFO << "Stopping trajectory collector";
    timer_.stop();
    is_running_ = false;
    if (collect_thread_.joinable()) {
      collect_thread_.join();
    }
    queue_condition_.notify_all();
    if (queue_ != nullptr) {
      delete queue_;
      queue_ = nullptr;
    }
  } else {
    AINFO << "Trajectory collector is not running.";
  }
  AINFO << "Trajectory collector stopped.";
  return true;
}
// Update trajectory collector options
bool TrajectoryCollector::UpdateOptions(const CollectorOptions& options) {
  if (!Stop()) {
    AWARN << "Trajectory collector stop failed.";
    return false;
  }
  if (!Init(options)) {
    AWARN << "Trajectory collector reinitialzied failed.";
    return false;
  }
  if (!Start()) {
    AWARN << "Trajectory collector start failed.";
    return false;
  }
  return true;
}
// Subscribe topics
boost::shared_ptr<ros::Subscriber> TrajectoryCollector::Subscribe(
    const std::string& topic) {
  AINFO << "Subscribing to:" << topic.c_str();
  ros::NodeHandle nh;
  boost::shared_ptr<int> count(boost::make_shared<int>(options_.limit));
  boost::shared_ptr<ros::Subscriber> sub(boost::make_shared<ros::Subscriber>());

  ros::SubscribeOptions ops;
  ops.topic = topic;
  ops.queue_size = 100;
  ops.md5sum = ros::message_traits::md5sum<topic_tools::ShapeShifter>();
  ops.datatype = ros::message_traits::datatype<topic_tools::ShapeShifter>();
  ops.helper = boost::make_shared<ros::SubscriptionCallbackHelperT<
      const ros::MessageEvent<topic_tools::ShapeShifter const>&>>(
      boost::bind(&TrajectoryCollector::EnQueue, this, _1, topic, sub, count));
  ops.transport_hints = ros::TransportHints().tcpNoDelay(true);

  *sub = nh.subscribe(ops);

  currently_recording_.insert(topic);
  num_subscribers_++;
  return sub;
}
// Get local time string
const std::string TrajectoryCollector::GetLocalTimeString() {
  time_t rawtime;
  struct tm* timeinfo;
  char buffer[80] = {0};

  time(&rawtime);
  timeinfo = std::localtime(&rawtime);

  strftime(buffer, 80, "%F-%H-%M-%S", timeinfo);

  return std::string(buffer);
}
// Update file names
void TrajectoryCollector::UpdateFilenames() {
  vector<string> parts;

  std::string prefix = options_.prefix;
  size_t ind = prefix.rfind(".bag");

  if (ind != std::string::npos && ind == prefix.size() - 4) {
    prefix.erase(ind);
  }
  if (prefix.length() > 0) {
    parts.push_back(prefix);
  }
  if (options_.append_date) {
    parts.push_back(GetLocalTimeString());
  }
  if (options_.split) {
    parts.push_back(boost::lexical_cast<string>(split_count_));
  }
  if (parts.size() == 0) {
    throw rosbag::BagException("Bag filename is empty");
  }

  target_filename_ = parts[0];
  for (uint32_t i = 1; i < parts.size(); i++) {
    target_filename_ += string("_") + parts[i];
  }

  target_filename_ += string(".bag");
  write_filename_ = target_filename_ + string(".active");
}
// Start writing data to file
void TrajectoryCollector::StartWriting() {
  bag_.setCompression(options_.compression);
  bag_.setChunkThreshold(options_.chunk_size);

  UpdateFilenames();
  // Open bag file.
  try {
    bag_.open(write_filename_, rosbag::bagmode::Write);
  } catch (rosbag::BagException e) {
    AERROR << "Error writing:" << e.what();
    Stop();
  }
}
// Stop writing file
void TrajectoryCollector::StopWriting() {
  bag_.close();
  rename(write_filename_.c_str(), target_filename_.c_str());
}
// Check logging
bool TrajectoryCollector::CheckNextWarnTime() {
  if (writing_enabled_) {
    return true;
  }

  ros::WallTime now = ros::WallTime::now();
  if (now >= warn_next_) {
    warn_next_ = now + ros::WallDuration().fromSec(5.0);
    AWARN << "Disk is full.";
  }
  return false;
}
// Schedule for check disk
bool TrajectoryCollector::ScheduledCheckDisk() {
  std::lock_guard<std::mutex> lock(check_disk_mutex_);

  if (ros::WallTime::now() < check_disk_next_) {
    return true;
  }
  check_disk_next_ += ros::WallDuration().fromSec(20.0);
  return CheckDisk();
}
// Check free space on disk
bool TrajectoryCollector::CheckDisk() {
  struct statvfs fs_info;
  if ((statvfs(bag_.getFileName().c_str(), &fs_info)) < 0) {
    AWARN << "Failed to check filesystem stats.";
    return true;
  }
  uint64_t free_space = 0;
  free_space = (uint64_t)(fs_info.f_bsize) * (uint64_t)(fs_info.f_bavail);
  if (free_space < options_.min_space) {
    AERROR << "Less than " << options_.min_space_str.c_str()
           << " of space free on disk with " << bag_.getFileName().c_str()
           << " Disabling recording.";
    writing_enabled_ = false;
    return false;
  } else if (free_space < 5 * options_.min_space) {
    AWARN << "Less than 5 x " << options_.min_space_str.c_str()
          << "of space free on disk with " << bag_.getFileName().c_str();
  } else {
    writing_enabled_ = true;
  }
  return true;
}
// Check number of splits
void TrajectoryCollector::CheckNumSplits() {
  if (options_.max_splits > 0) {
    current_files_.push_back(target_filename_);
    if (current_files_.size() > options_.max_splits) {
      int err = unlink(current_files_.front().c_str());
      if (err != 0) {
        AERROR << "Unable to remove " << current_files_.front().c_str()
               << strerror(errno);
      }
      current_files_.pop_front();
    }
  }
}
// Check bag file size
bool TrajectoryCollector::CheckSize() {
  if (options_.max_size > 0) {
    if (bag_.getSize() > options_.max_size) {
      if (options_.split) {
        StopWriting();
        split_count_++;
        CheckNumSplits();
        StartWriting();
      } else {
        StopWriting();
        return true;
      }
    }
  }
  return false;
}
// Check duration
bool TrajectoryCollector::CheckDuration(const ros::Time& t) {
  if (options_.max_duration > ros::Duration(0)) {
    if (t - start_time_ > options_.max_duration) {
      if (options_.split) {
        while (start_time_ + options_.max_duration < t) {
          StopWriting();
          split_count_++;
          CheckNumSplits();
          start_time_ += options_.max_duration;
          StartWriting();
        }
      } else {
        StopWriting();
        return true;
      }
    }
  }
  return false;
}
// Check mileage
bool TrajectoryCollector::CheckMileage() {
  // TODO(zhanghua): mileage check should be add in future.
  return false;
}
// Publish collector message
void TrajectoryCollector::PublishCollector() {
  collector_data_.set_timestamp_sec(ros::Time::now().toSec());
  collector_data_.set_max_speed_limit(options_.max_speed_limit);
  collector_data_.set_min_speed_limit(options_.min_speed_limit);
  collector_publisher_.publish(collector_data_);
}
// Timer callback for publish collector message
void TrajectoryCollector::OnTimer(const ros::TimerEvent&) {
  if ((std::fabs(last_max_speed_limit - options_.max_speed_limit) >
       DBL_EPSILON) ||
      (std::fabs(last_min_speed_limit - options_.min_speed_limit) >
       DBL_EPSILON)) {
    PublishCollector();
    last_max_speed_limit = options_.max_speed_limit;
    last_min_speed_limit = options_.min_speed_limit;
  }
}
// Callback to be invoked to push messages into a queue
void TrajectoryCollector::EnQueue(
    const ros::MessageEvent<topic_tools::ShapeShifter const>& msg_event,
    const std::string& topic, boost::shared_ptr<ros::Subscriber> subscriber,
    boost::shared_ptr<int> count) {
  Time rectime = Time::now();
  CollectorMessage out(topic, msg_event.getMessage(),
                       msg_event.getConnectionHeaderPtr(), rectime);
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);

    queue_->push(out);
    queue_size_ += out.msg_->size();
    // Check to see if buffer has been exceeded
    while (options_.buffer_size > 0 && queue_size_ > options_.buffer_size) {
      CollectorMessage drop = queue_->front();
      queue_->pop();
      queue_size_ -= drop.msg_->size();

      Time now = Time::now();
      if (now > last_buffer_warn_ + ros::Duration(5.0)) {
        AWARN << "Record buffer exceeded.";
        last_buffer_warn_ = now;
      }
    }
  }

  queue_condition_.notify_one();
}

// Thread that actually does writing to file.
void TrajectoryCollector::CollectData() {
  // Open bag file for writing
  StartWriting();
  // Schedule the disk space check
  warn_next_ = ros::WallTime();
  try {
    CheckDisk();
  } catch (rosbag::BagException& ex) {
    AERROR << "TrajectoryCollector check disk failed " << ex.what();
    StopWriting();
    return;
  }
  // Next time check disk
  check_disk_next_ = ros::WallTime::now() + ros::WallDuration().fromSec(20.0);

  // The queue_mutex_ should be locked while checking empty.
  // Except it should only get checked if the node is not ok, and thus
  // it shouldn't be in contention.
  ros::NodeHandle nh;
  while (nh.ok() || !queue_->empty() || is_running_) {
    std::unique_lock<std::mutex> lock(queue_mutex_);
    bool finished = false;
    // User stop, exit this thread.
    if (!is_running_) {
      lock.unlock();
      finished = true;
      break;
    }
    // If Queue is empty,wait for messages incoming.
    while (queue_->empty()) {
      if (!nh.ok()) {
        finished = true;
        break;
      }
      queue_condition_.wait_for(lock, std::chrono::seconds(120));
      // Check Duration
      if (CheckDuration(ros::Time::now())) {
        finished = true;
        break;
      }
    }
    if (finished) {
      lock.unlock();
      break;
    }
    // Pop message to write.
    CollectorMessage out = queue_->front();
    queue_->pop();
    queue_size_ -= out.msg_->size();
    // Unlock mutex
    lock.unlock();
    queue_condition_.notify_one();
    // Check size and duration
    if (CheckSize()) {
      AINFO << "Trajectory collector bag file size has been exceeded.";
      break;
    }
    if (CheckDuration(out.time_)) {
      AINFO << "Trajectory collector duration expired.";
      break;
    }
    // Check disk and write data to bag file
    try {
      if (ScheduledCheckDisk() && CheckNextWarnTime()) {
        bag_.write(out.topic_, out.time_, *out.msg_, out.connection_header_);
      }
    } catch (rosbag::BagException& ex) {
      AERROR << "Trajectory collector write bag file failed " << ex.what();
      break;
    }
  }
  // Stop write file
  StopWriting();
  queue_condition_.notify_one();
}

}  // namespace util
}  // namespace navi_generator
}  // namespace apollo
