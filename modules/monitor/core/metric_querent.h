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

#ifndef MODULES_MONITOR_SYSMON_METRIC_QUERENT_H_
#define MODULES_MONITOR_SYSMON_METRIC_QUERENT_H_


#include <chrono>
#include <cstdint>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <map>

#include "modules/common/macro.h"
#include "modules/monitor/sysmon/base/plugin_interface.h"
#include "modules/monitor/sysmon/core/defs.h"

namespace apollo {
namespace monitor {
namespace sysmon {

typedef std::chrono::system_clock::time_point metric_timestamp_t;

class MetricQuerent {
 public:

  class NullQuerentInstException : public std::exception {
   public:
    virtual const char *what() const throw() {
      return "Creating MetricQuerent with a null querent pointer.";
    }
  };

  /// Query result observer function that will be invoked when query result
  /// becomes available for a metric.
  /// An observer function must NOT block, should schedule it to run in
  /// another thread for operations that may take some time to finish.
  /// Callee doesn't take ownership of metric or data pointer.
  typedef std::function<void(const MetricQuerent *metric,
      MetricQuerentInterface::MetricQueryResult,
      MetricDataInterface *data, const metric_timestamp_t &ts,
      const std::string &err)> QueryResultObserver;

  typedef int NidType;

  static constexpr NidType INVALID_NID = -1;

  inline static bool nid_is_valid(NidType id) {
    return id != INVALID_NID;
  }

  explicit MetricQuerent(const std::string &name,
      std::unique_ptr<MetricQuerentInterface> querent, NidType id)
      throw(NullQuerentInstException)
      : num_id_(id), m_name_(name), querent_(querent.release()),
    last_ts_(std::chrono::system_clock::time_point::min()),
    last_result_(MetricQuerentInterface::MetricQueryResult::NOT_AVAILABLE),
    observer_id_(1) {
    if (!querent_.get()) {
      throw NullQuerentInstException();
    }
  }

  virtual ~MetricQuerent() {}

  /// Returns numeric ID of the metric.
  NidType get_nid() const { return num_id_; }

  /// Sets numeric ID of the metric.
  //void set_nid(uint64_t nid) { num_id_ = nid; }

  /// Returns name of the metric.
  const std::string &get_name() const { return m_name_; }

  /// Sets name of the metric.
  //void set_name(const std::string &name) { m_name_ = name; }

  /// Runs the query function for this metric.
  /// @return return value from metric query function.
  /// @see MetricQuerentInterface
  virtual MetricQuerentInterface::MetricQueryResult run_query()
      ACQ_LOCK(result_lock_, observer_lock_);

  /// Gets the data & timestamp of last query for this metric.
  /// Caller takes ownership of pointer stored in data.
  /// Stores timestamp in ts and stores error message (if any) in err.
  MetricQuerentInterface::MetricQueryResult get_last_result(
      MetricDataInterface **data, metric_timestamp_t &ts, std::string *err)
      ACQ_LOCK(result_lock_);

  /// Adds an observer which will be called when query result becomes available,
  /// for which result matches filter.
  /// @return an integer key that can be used to remove the data observer later.
  int add_result_observer(QueryResultObserver observer, unsigned int filter)
      ACQ_LOCK(observer_lock_);

  /// Removes a data observer identified by the given key.
  /// @return # of observer function removed, 0 or 1.
  int rmv_result_observer(int key) ACQ_LOCK(observer_lock_);

  /// Registers query result callback with underlying querent to start receiving
  /// results for async querent.
  bool start_async();

  /// Gets query attributes of the metric querent.
  inline const MetricQuerentInterface::QueryAttr &get_query_attr() const
  { return querent_.get()->get_query_attr(); }

  /// Gets configurations of this metric querent.
  inline const nlohmann::json &get_cfg() const
  { return querent_.get()->get_cfg(); }

 protected:
  // Callback function to be registered with underlying async querent to receive
  // metric result.
  void on_async_result(MetricQuerentInterface *,
      MetricQuerentInterface::MetricQueryResult,
      MetricDataInterface *data, const std::string &err)
      ACQ_LOCK(result_lock_, observer_lock_);

 private:

  void on_query_result(MetricQuerentInterface::MetricQueryResult,
      MetricDataInterface *data, const std::string &err)
      ACQ_LOCK(result_lock_, observer_lock_);

  // Globally unique id (only valid for the lifetime of a class instance).
  NidType num_id_;

  // Globally unique name (may be valid cross sessions).
  std::string m_name_;

  // Underlying querent instance.
  std::unique_ptr<MetricQuerentInterface> querent_;

  // Timestamp of last result quereied/received.
  metric_timestamp_t last_ts_ XLOCK_BY(result_lock_);
  // Last query result.
  MetricQuerentInterface::MetricQueryResult last_result_ XLOCK_BY(result_lock_);
  // Last query error message, if any. Valid only if last result is not OK.
  std::string last_err_ XLOCK_BY(result_lock_);
  // Data from last query.
  std::unique_ptr<MetricDataInterface> last_data_ XLOCK_BY(result_lock_);

  // Internally an id-to-observer map is used to keep track of
  // query result observers.
  int observer_id_ XLOCK_BY(observer_lock_);
  std::map<int, std::pair<unsigned, QueryResultObserver>>
      observers_ XLOCK_BY(observer_lock_);

  std::mutex result_lock_;
  std::mutex observer_lock_;

private:
  DISALLOW_COPY_AND_ASSIGN(MetricQuerent);
};

/// Dummy query result observer that doesn't do anything.
inline void dummy_metric_query_result_observer(const MetricQuerent *,
    std::unique_ptr<MetricDataInterface>, const metric_timestamp_t &,
    const std::string &)
{}

}  // namespace sysmon
}  // namespace monitor
}  // namespace apollo

#endif  // MODULES_MONITOR_SYSMON_METRIC_QUERENT_H_
