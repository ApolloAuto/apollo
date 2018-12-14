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

/**
 * @file
 * @brief This file provides the trajectory colloctor.
 */

#ifndef MODULES_TOOLS_NAVI_GENERATOR_BACKEND_UTIL_TRAJECTORY_COLLECTOR_H_
#define MODULES_TOOLS_NAVI_GENERATOR_BACKEND_UTIL_TRAJECTORY_COLLECTOR_H_

#include <ros/time.h>
#include <rosbag/bag.h>
#include <topic_tools/shape_shifter.h>

#include <condition_variable>
#include <list>
#include <memory>
#include <mutex>
#include <queue>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "modules/tools/navi_generator/proto/trajectory_collector.pb.h"

namespace apollo {
namespace navi_generator {
namespace util {

/**
 * @class CollectorMessage
 * @brief Collector message class.
 */
class CollectorMessage {
 public:
  /**
   * @brief Constructor with the CollectorMessage.
   * @param topic The collector topic name.
   * @param msg The pointer of topic message.
   * @param connection_header The pointer of connection header.
   * @param time Time of received topic.
   */
  CollectorMessage(const std::string& topic,
                   topic_tools::ShapeShifter::ConstPtr msg,
                   boost::shared_ptr<ros::M_string> connection_header,
                   ros::Time time);

  std::string topic_;
  topic_tools::ShapeShifter::ConstPtr msg_;
  boost::shared_ptr<ros::M_string> connection_header_;
  ros::Time time_;
};

class CollectorMsgQueue {
 public:
  /**
   * @brief Constructor with the CollectorMsgQueue.
   * @param filename The file name of bag file.
   * @param queue The pointer of collector message queue.
   * @param time The time of output message.
   */
  CollectorMsgQueue(const std::string& filename,
                    std::queue<CollectorMessage>* queue, ros::Time time);
  std::string filename_;
  std::queue<CollectorMessage>* queue_;
  ros::Time time_;
};

/**
 * @struct CollectorOptions
 * @brief A collector options.
 */
struct CollectorOptions {
  CollectorOptions();
  // Flag of split files.
  bool split;
  // Flag of append date for file name.
  bool append_date;
  // Compression type.
  rosbag::compression::CompressionType compression;
  // Buffer size.
  uint32_t buffer_size;
  // Chunk size.
  uint32_t chunk_size;
  // Limit number of topics.
  uint32_t limit;
  // Max mileage.
  uint32_t max_mileage;
  // Max number of splits.
  uint32_t max_splits;
  // Max size of bag file.
  uint64_t max_size;
  // Min space of disk.
  uint64_t min_space;
  // Max duration.
  ros::Duration max_duration;
  // Max speed limit.
  double max_speed_limit;
  // Min speed limit.
  double min_speed_limit;
  // String of prefix file name.
  std::string prefix;
  // String of file name.
  std::string name;
  // String of min space.
  std::string min_space_str;
  // Vector string for storing topics.
  std::vector<std::string> topics;
};

/**
 * @class TrajectoryCollector
 * @brief A utility class that helps record topic messages into BAG files.
 */
class TrajectoryCollector {
 public:
  TrajectoryCollector();
  ~TrajectoryCollector() = default;
  /**
   * @brief Initialize trajectory collector.
   * @param options of collector
   * @return true is for success
   */
  bool Init(const CollectorOptions& options);
  /**
   * @brief Start collector.
   * @return True is for success.
   */
  bool Start();
  /**
   * @brief Stop collector.
   * @return True is for success.
   */
  bool Stop();
  /**
   * @brief Update collector options.
   * @param options The options of collector.
   * @return True is for success.
   */
  bool UpdateOptions(const CollectorOptions& options);

 private:
  /**
   * @brief Subscribe topics.
   * @param topic The topic to be subscribed.
   * @return The pointer of subscribe handle
   */
  boost::shared_ptr<ros::Subscriber> Subscribe(const std::string& topic);
  /**
   * @brief Update file names.
   */
  void UpdateFilenames();
  /**
   * @brief Start writing data to file.
   */
  void StartWriting();
  /**
   * @brief Stop writing file.
   */
  void StopWriting();
  /**
   * @brief Check next warn time.
   * @return True is for success.
   */
  bool CheckNextWarnTime();
  /**
   * @brief Schedule for check disk.
   * @return True is for success.
   */
  bool ScheduledCheckDisk();
  /**
   * @brief Check disk.
   * @return True is for success
   */
  bool CheckDisk();
  /**
   * @brief Check number of splits.
   */
  void CheckNumSplits();
  /**
   * @brief Check size of file.
   * @return True is for success
   */
  bool CheckSize();
  /**
   * @brief Check duration.
   * @return True is for success
   */
  bool CheckDuration(const ros::Time& t);
  /**
   * @brief Check mileage.
   * @return True is for success.
   */
  bool CheckMileage();
  /**
   * @brief Callback method for subscribed topic message.
   * @param msg_event The ROS message event.
   * @param topic The received topic.
   * @param subscriber The pointer to the ROS subscriber.
   * @param count The count of recording topics.
   */
  void EnQueue(
      const ros::MessageEvent<topic_tools::ShapeShifter const>& msg_event,
      const std::string& topic, boost::shared_ptr<ros::Subscriber> subscriber,
      boost::shared_ptr<int> count);
  /**
   * @brief Collect thread execution method for writing data into bag file.
   */
  void CollectData();
  /**
   * @brief Get local time string.
   */
  const std::string GetLocalTimeString();
  /**
   * @brief Callback timer.
   */
  void OnTimer(const ros::TimerEvent& event);
  /**
   * @brief Publish collector message.
   */
  void PublishCollector();

 private:
  // Collector options.
  CollectorOptions options_;
  // ROS bag instance.
  rosbag::Bag bag_;
  // ROS bag target file name (*.bag)
  std::string target_filename_;
  // ROS bag intermediate file name (attached .active).
  std::string write_filename_;
  // Current files name list.
  std::list<std::string> current_files_;
  // Set of currently recording topics.
  std::set<std::string> currently_recording_;
  // Used for book-keeping of our number of subscribers.
  int num_subscribers_;
  // Conditional variable for queue.
  std::condition_variable queue_condition_;
  // Mutex for queue.
  std::mutex queue_mutex_;
  // Queue for storing.
  std::queue<CollectorMessage>* queue_;
  // Queue size.
  uint64_t queue_size_;
  // Max queue size.
  uint64_t max_queue_size_;
  // Split count.
  uint64_t split_count_;
  // Time for last buffer warning.
  ros::Time last_buffer_warn_;
  // Start collect time.
  ros::Time start_time_;
  // Flag for writing enable.
  bool writing_enabled_;
  // Flag for initialized.
  bool initialized_;
  // Flag for running.
  bool is_running_;
  // Max speed limit.
  double last_max_speed_limit;
  // Min speed limit.
  double last_min_speed_limit;
  // Mutex for check disk.
  std::mutex check_disk_mutex_;
  // Time for next time of check disk.
  ros::WallTime check_disk_next_;
  // Time for next time of warning.
  ros::WallTime warn_next_;
  // Collector thread.
  std::thread collect_thread_;
  // Collector message data
  apollo::navi_generator::TrajectoryCollectorMsg collector_data_;
  // Collector publisher
  ros::Publisher collector_publisher_;
  // Collector timer
  ros::Timer timer_;
};

}  // namespace util
}  // namespace navi_generator
}  // namespace apollo

#endif  // MODULES_TOOLS_NAVI_GENERATOR_BACKEND_UTIL_TRAJECTORY_COLLECTOR_H_
