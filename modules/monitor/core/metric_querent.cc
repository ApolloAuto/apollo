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

#include "modules/monitor/sysmon/core/metric_querent.h"

#include <algorithm>
#include <chrono>

using std::chrono::system_clock;

namespace apollo {
namespace monitor {
namespace sysmon {

using MetricQueryType = MetricQuerentInterface::MetricQueryType;
using MetricQueryResult = MetricQuerentInterface::MetricQueryResult;

void MetricQuerent::on_query_result(MetricQueryResult result,
    MetricDataInterface *data, const std::string &err)
{
  metric_timestamp_t ts = system_clock::now();

  // @todo: print timestamp
  DBG_ONLY(
    ADEBUG << "got query result for metric " << m_name_ << "(" << num_id_
        << "): " << result;
  )

  {
    std::lock_guard<std::mutex> guard(observer_lock_);
    if (observers_.size() > 0) {
      for(auto& kv : observers_) {
        if (kv.second.first & (unsigned)result) {
          // @todo: print timestamp
          DBG_ONLY(
            ADEBUG << "invoking data observer for metric " << m_name_
                << ", key=" << kv.first;
          )
          kv.second.second(this, result, data, ts, err);
        }
      }
    }
  }

  std::lock_guard<std::mutex> guard(result_lock_);
  last_result_ = result;
  last_ts_ = ts;
  last_data_.reset(data);
  last_err_ = err;
}

void MetricQuerent::on_async_result(MetricQuerentInterface *querent,
    MetricQueryResult result, MetricDataInterface *data,
    const std::string &err)
{
  // We don't take ownership of data, make a copy and save it.
  on_query_result(result, data->mk_copy(), err);
}

MetricQueryResult MetricQuerent::run_query()
{
  MetricDataInterface *data;
  std::string err;

  MetricQueryResult result = querent_->query(&data, &err);
  on_query_result(result, data->mk_copy(), std::move(err));

  return result;
}

bool MetricQuerent::start_async()
{
  const MetricQuerentInterface::QueryAttr &q_attr = querent_->get_query_attr();
  if (q_attr.query_type == MetricQueryType::QT_ASYNC) {
    return querent_->register_result_cb(
        std::bind(&MetricQuerent::on_async_result, this,
                  std::placeholders::_1,
                  std::placeholders::_2,
                  std::placeholders::_3,
                  std::placeholders::_4));
  }

  return false;
}

MetricQueryResult MetricQuerent::get_last_result(
    MetricDataInterface **data, metric_timestamp_t &ts, std::string *err)
{
  std::lock_guard<std::mutex> guard(result_lock_);
  ts = last_ts_;
  if (last_result_ == MetricQueryResult::OK) {
    if (!last_data_.get()) {
      // There must be data if result is OK.
      *err = std::string("internal error: data not available as expected");
      return MetricQueryResult::ERROR;
    }
  } else {  // There may be error message for non-OK result only.
    if (err) { *err = last_err_; }
  }

  if (last_data_.get()) {
    *data = last_data_->mk_copy();
  }
  return last_result_;
}

int MetricQuerent::add_result_observer(
    QueryResultObserver observer, unsigned int filter)
{
  std::lock_guard<std::mutex> guard(observer_lock_);
  // We use a simple incremental int as key (which can be used to remove
  // an observer later).
  int key = observer_id_++;
  observers_[key] =  std::make_pair(filter, observer);

  ADEBUG << "data observer added with key=" << key << " for metric " << m_name_;

  return key;
}

int MetricQuerent::rmv_result_observer(int key)
{
  std::lock_guard<std::mutex> guard(observer_lock_);
  ADEBUG << "to remove data observer of key=" << key
      << " for metric " << m_name_;
  return (int)observers_.erase(key);
}

}  // namespace sysmon
}  // namespace monitor
}  // namespace apollo
