/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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

#pragma once

#ifndef CYBER_STATISTICS_STATISTICS_H_
#define CYBER_STATISTICS_STATISTICS_H_

#include <limits.h>

#include <memory>
#include <mutex>
#include <utility>
#include <string>
#include <unordered_map>

#include "cyber/base/macros.h"
#include "cyber/common/log.h"
#include "cyber/proto/role_attributes.pb.h"
#include "cyber/common/macros.h"
#include "cyber/time/time.h"
#include "third_party/var/bvar/bvar.h"

namespace apollo {
namespace cyber {
namespace statistics {

using LatencyVarPtr = std::shared_ptr<::bvar::LatencyRecorder>;
using StatusVarPtr = std::shared_ptr<::bvar::Status<uint64_t>>;
using AdderVarPtr = std::shared_ptr<::bvar::Adder<int32_t>>;

struct SpanHandler {
  std::string name;
  uint64_t start_time;
  uint64_t end_time;
  std::shared_ptr<::bvar::LatencyRecorder> span_trace_node;
  uint64_t min_ns = 0;
};

static const std::string TIMER_COMPONENT_CHAN_NAME = "_timer_component";    // NOLINT

class Statistics {
 public:
  ~Statistics() {}
  bool RegisterChanVar(const proto::RoleAttributes& role_attr);

  inline bool CreateSpan(std::string name, uint64_t min_ns = 0);
  inline bool StartSpan(std::string name);
  inline bool EndSpan(std::string name);

  inline void DisableChanVar() {
    disable_chan_var_ = true;
  }

  template <typename SampleT>
  std::shared_ptr<::bvar::Adder<SampleT>> CreateAdder(
                        const proto::RoleAttributes& role_attr) {
    std::string expose_name =
      role_attr.node_name() + "-" + role_attr.channel_name();
    return std::make_shared<::bvar::Adder<SampleT>>(expose_name);
  }

  template <typename SampleT>
  bool SamplingProcLatency(
        const proto::RoleAttributes& role_attr, SampleT sample) {
    if (disable_chan_var_) {
      return true;
    }
    auto var_ptr = GetChanProcVar(role_attr);
    if (var_ptr != nullptr) {
      (*var_ptr) << sample;
    } else {
      return false;
    }
    return true;
  }

  template <typename SampleT>
  bool SamplingTranLatency(
        const proto::RoleAttributes& role_attr, SampleT sample) {
    if (disable_chan_var_) {
      return true;
    }
    if (role_attr.channel_name() == TIMER_COMPONENT_CHAN_NAME) {
      return true;
    }
    auto var_ptr = GetChanTranVar(role_attr);
    if (var_ptr != nullptr) {
      (*var_ptr) << sample;
    } else {
      return false;
    }
    return true;
  }

  template <typename SampleT>
  bool SamplingCyberLatency(
          const proto::RoleAttributes& role_attr, SampleT sample) {
    if (disable_chan_var_) {
      return true;
    }
    if (role_attr.channel_name() == TIMER_COMPONENT_CHAN_NAME) {
      return true;
    }
    auto var_ptr = GetChanCyberVar(role_attr);
    if (var_ptr != nullptr) {
      (*var_ptr) << sample;
    } else {
      return false;
    }
    return true;
  }

  bool SetProcStatus(const proto::RoleAttributes& role_attr, uint64_t val) {
    if (disable_chan_var_) {
      return true;
    }
    if (role_attr.channel_name() == TIMER_COMPONENT_CHAN_NAME) {
      return true;
    }
    auto var_ptr = GetProcStatusVar(role_attr);
    if (var_ptr != nullptr) {
      var_ptr->set_value(val);
    } else {
      return false;
    }
    return true;
  }

  bool GetProcStatus(const proto::RoleAttributes& role_attr, uint64_t* val) {
    if (disable_chan_var_) {
      *val = 0;
      return true;
    }
    if (role_attr.channel_name() == TIMER_COMPONENT_CHAN_NAME) {
      return false;
    }
    auto var_ptr = GetProcStatusVar(role_attr);
    if (var_ptr != nullptr) {
      *val = var_ptr->get_value();
    } else {
      return false;
    }
    return true;
  }

  bool SetTotalMsgsStatus(const proto::RoleAttributes& role_attr, int32_t val) {
    if (disable_chan_var_) {
      return true;
    }
    if (role_attr.channel_name() == TIMER_COMPONENT_CHAN_NAME) {
      return true;
    }
    auto var_ptr = GetTotalMsgsStatusVar(role_attr);
    if (var_ptr != nullptr) {
      var_ptr->set_value(val);
    } else {
      return false;
    }
    return true;
  }

  bool AddRecvCount(const proto::RoleAttributes& role_attr, int total_msg_val) {
    if (disable_chan_var_) {
      return true;
    }
    if (role_attr.channel_name() == TIMER_COMPONENT_CHAN_NAME) {
      return true;
    }

    auto var_ptr = GetAdderVar(role_attr);
    if (var_ptr == nullptr) {
      return true;
    }

    if cyber_unlikely(first_recv_) {
      (*var_ptr) << total_msg_val;
      first_recv_ = false;
      return true;
    }

    (*var_ptr) << 1;

    return true;
  }

 private:
  LatencyVarPtr GetChanProcVar(const proto::RoleAttributes& role_attr);

  LatencyVarPtr GetChanTranVar(const proto::RoleAttributes& role_attr);

  LatencyVarPtr GetChanCyberVar(const proto::RoleAttributes& role_attr);

  StatusVarPtr GetProcStatusVar(const proto::RoleAttributes& role_attr);

  AdderVarPtr GetAdderVar(const proto::RoleAttributes& role_attr);

  StatusVarPtr GetTotalMsgsStatusVar(const proto::RoleAttributes& role_attr);

  inline uint64_t GetMicroTimeNow() const noexcept;

  inline const std::string GetProcLatencyKey(
                      const proto::RoleAttributes& role_attr) {
    return role_attr.node_name() + "-" + role_attr.channel_name() + "proc";
  }

  inline const std::string GetTranLatencyKey(
                      const proto::RoleAttributes& role_attr) {
    return role_attr.node_name() + "-" + role_attr.channel_name() + "tran";
  }

  inline const std::string GetCyberLatencyKey(
                      const proto::RoleAttributes& role_attr) {
    return role_attr.node_name() + "-" + role_attr.channel_name() + "cyber";
  }

  inline const std::string GetStartProcessStatusKey(
                      const proto::RoleAttributes& role_attr) {
    return role_attr.node_name() + "-" + \
            role_attr.channel_name() + "process-status";
  }

  inline const std::string GetTotalMsgsStatusKey(
                      const proto::RoleAttributes& role_attr) {
    return role_attr.node_name() + "-" + \
          role_attr.channel_name() + "total-sended-msgs";
  }

  inline const std::string GetTotalRecvStatusKey(
                      const proto::RoleAttributes& role_attr) {
    return role_attr.node_name() + "-" + role_attr.channel_name() + "recv-msgs";
  }

  std::unordered_map<std::string, LatencyVarPtr> latency_map_;
  std::unordered_map<std::string, StatusVarPtr> status_map_;
  std::unordered_map<std::string, AdderVarPtr> adder_map_;

  std::unordered_map<std::string, std::shared_ptr<SpanHandler>> span_handlers_;

  bool first_recv_ = true;
  bool disable_chan_var_ = false;

  DECLARE_SINGLETON(Statistics)
};

inline uint64_t Statistics::GetMicroTimeNow() const noexcept {
  return Time::Now().ToMicrosecond();
}

inline bool Statistics::CreateSpan(std::string name, uint64_t min_ns) {
  if (cyber_unlikely(span_handlers_.find(name) != span_handlers_.end())) {
    AERROR << "span handler " << name << "has been created!";
    return false;
  }
  auto handler = std::make_shared<SpanHandler>();
  handler->name = name;
  handler->span_trace_node = std::move(
          std::make_shared<::bvar::LatencyRecorder>(name, "user", 1200));
  handler->min_ns = min_ns;

  span_handlers_[name] = std::move(handler);
  return true;
}

inline bool Statistics::StartSpan(std::string name) {
  auto it = span_handlers_.find(name);
  if (cyber_unlikely(it == span_handlers_.end())) {
    AERROR << "span handler " << name << "not found!";
    return false;
  }
  it->second->start_time = std::move(GetMicroTimeNow());
  return true;
}

inline bool Statistics::EndSpan(std::string name) {
  auto it = span_handlers_.find(name);
  if (cyber_unlikely(it == span_handlers_.end())) {
    AERROR << "span handler " << name << "not found!";
    return false;
  }
  it->second->end_time = std::move(GetMicroTimeNow());
  auto handler = it->second;
  auto diff = handler->end_time - handler->start_time;
  if (cyber_unlikely(diff < handler->min_ns)) {
    AWARN << "Time span less than preset value: " \
          << diff << " vs " << handler->min_ns;
    return false;
  }
  if (cyber_unlikely(diff > INT32_MAX)) {
    AWARN << "Time span is larger than INT32_MAX: " << diff << ", drop it...";
    return false;
  }
  *(handler->span_trace_node) << diff;
  return true;
}

}  // namespace statistics
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_STATISTICS_STATISTICS_H_
