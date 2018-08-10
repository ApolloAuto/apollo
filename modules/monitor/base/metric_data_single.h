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

#ifndef MODULES_MONITOR_SYSMON_METRIC_DATA_SINGLE_H_
#define MODULES_MONITOR_SYSMON_METRIC_DATA_SINGLE_H_

#include <sstream>
#include <string>

#include "modules/monitor/sysmon/base/plugin_interface.h"

namespace apollo {
namespace monitor {
namespace sysmon {

template<class DataType>
class MetricDataSingle : public MetricDataInterface {
 public:
  explicit MetricDataSingle(const DataType &d) : data_(d) {}

  /// Returns serializer capability.
  unsigned int get_serializer_capability() const override {
    return static_cast<unsigned int>(DataSerDesType::JSON)
        | static_cast<unsigned int>(DataSerDesType::HOST_BLOB);
  }

  /// Creates a copy.
  MetricDataInterface *mk_copy() const override {
    return new MetricDataSingle<DataType>(data_);
  }

  /// Serializes data value to a json string.
  std::string to_json_str() const override {
    std::ostringstream os;
    os << data_;
    return os.str();
  }

  BlobSzReq blob_sz_req_type(DataSerDesType dt) const override {
    if (dt == DataSerDesType::HOST_BLOB) {
      return BlobSzReq::FIXED_INIT;
    } else {
      return BlobSzReq::PROGRESSIVE;
    }
  }

  size_t blob_sz_required(DataSerDesType dt) const override {
    if (dt == DataSerDesType::HOST_BLOB) {
      return sizeof(DataType);
    } else {
      return BLOB_SZ_UNKNOWN;
    }
  }

  DataSerDesResult to_host_blob(ResizableBufferInterface *buf) const override {
    if (buf->reserve(sizeof(DataType))) {
      *(reinterpret_cast<DataType*>(buf->get_buf())) = data_;
      return DataSerDesResult::OK;
    }
    return DataSerDesResult::NO_MEM;
  }

  DataType get() const { return data_; }

  void set(const DataType &d) { data_ = d; }

 protected:
  DataType data_;
};

template<class DataType>
class MetricDataSingleDes : public MetricDataDesInterface {
 public:
  MetricDataSingleDes() {}

  /// Returns deserializer capability.
  unsigned int get_des_capbility() const override {
    return (unsigned int)DataSerDesType::JSON;
  }

  /// Receive data in a json string.
  bool rcv_json_str(const std::string &jstr) override {
    std::stringstream input(jstr);
    input.exceptions(std::stringstream::failbit | std::stringstream::badbit);
    try {
      input >> data_;
      data_json_ = jstr;
      return true;
    } catch (std::stringstream::failure e) {
      data_json_.clear();
      return false;
    }
  }

  /// Prints to a string for on-screen display etc., simple version.
  std::string to_str_simp() const override {
    std::ostringstream os;
    os << data_;
    return os.str();
  }

  /// Prints to a string for on-screen display etc., verbose version.
  std::string to_str_verbose() const override {
    return to_str_simp();
  }

  const nlohmann::json &get_json_str() const override {
    return data_json_;
  }

  const DataType &get() const { return data_; }

 protected:
  DataType data_;
  nlohmann::json data_json_;
};

typedef MetricDataSingleDes<int> SingleIntMetricDataDes;

/// A Presence metric uses integer value non-0 to indicate (something) being
/// present and 0 to indicate missing.
typedef MetricDataSingle<int> PresenceMetricData;

class PresenceMetricDataDes : public MetricDataSingleDes<int> {
 public:
  bool config(const nlohmann::json *cfg) override {
    if (cfg) {
      save_config(cfg);
      const auto val = cfg->find("present");
      if (val != cfg->end() && val->is_string()) {
        present_str_ = *val;
      }
      const auto v2 = cfg->find("missing");
      if (v2 != cfg->end() && v2->is_string()) {
        missing_str_ = *v2;
      }
    }
    return true;
  }

  PresenceMetricDataDes(
      const std::string &pres_str = std::string("present"),
      const std::string &mis_str = std::string("missing")) {
    present_str_ = pres_str;
    missing_str_ = mis_str;
  }

  /// Prints to a string for on-screen display etc., simple version.
  std::string to_str_simp() const override {
    if (data_) {
      return std::string("X");
    } else {
      return std::string("-");
    }
  }

  std::string to_str_verbose() const override {
    if (data_) {
      return present_str_;
    } else {
      return missing_str_;
    }
  }

 protected:
  std::string present_str_;
  std::string missing_str_;
};

/// A temperature metric uses a single integer value to represent temperature.
typedef MetricDataSingle<int> TemperatureMetricData;

// @todo: unicode support
class TemperatureMetricDataDes : public MetricDataSingleDes<int> {
 public:
  enum Unit {
    TEMP_C = 0,
    TEMP_F = 1
  };

  explicit TemperatureMetricDataDes(Unit u = TEMP_C) {
    if (u == TEMP_C) {
      unit_str_ = 'C';
    } else {
      unit_str_ = 'F';
    }
  }

  bool config(const nlohmann::json *cfg) override {
    if (cfg) {
      save_config(cfg);
      const auto val = cfg->find("unit");
      if (val != cfg->end() && val->is_string()) {
        unit_str_ = *val;
      }
    }
    return true;
  }

  /// Prints to a string for on-screen display etc., verbose version.
  std::string to_str_verbose() const override {
    std::ostringstream os;
    os << data_ << unit_str_;
    return os.str();
  }

 protected:
  std::string unit_str_;
};


/// A fan-speed metric uses a single integer value to represent fan rotation
/// speed.
typedef MetricDataSingle<int> FanSpeedMetricData;

class FanSpeedMetricDataDes : public MetricDataSingleDes<int> {
 public:
  /// Prints to a string for on-screen display etc., verbose version.
  std::string to_str_verbose() const override {
    std::ostringstream os;
    os << data_ << "rpm";
    return os.str();
  }
};



/// A status metric uses a single integer value to represent on-off status
/// (e.g., a switch): 0 - off, 1 - on.
typedef MetricDataSingle<int> OnOffStatusMetricData;

class OnOffStatusMetricDataDes : public MetricDataSingleDes<int> {
 public:
  OnOffStatusMetricDataDes(
      const std::string &on_label = std::string("On"),
      const std::string &off_label = std::string("Off")) {
    on_label_ = on_label;
    off_label_ = off_label;
  }

  bool config(const nlohmann::json *cfg) override {
    if (cfg) {
      save_config(cfg);
      const auto val = cfg->find("on_label");
      if (val != cfg->end() && val->is_string()) {
        on_label_ = *val;
      }
      const auto v2 = cfg->find("off_label");
      if (v2 != cfg->end() && v2->is_string()) {
        off_label_ = *v2;
      }
    }
    return true;
  }

  /// Prints to a string for on-screen display etc., simple version.
  std::string to_str_simp() const override {
    return data_ ? std::string("I") : std::string("O");
  }

  /// Prints to a string for on-screen display etc., verbose version.
  std::string to_str_verbose() const override {
    return data_ ? on_label_ : off_label_;
  }

 protected:
  std::string on_label_;
  std::string off_label_;
};

}  // namespace sysmon
}  // namespace monitor
}  // namespace apollo

#endif  // MODULES_MONITOR_SYSMON_METRIC_DATA_SINGLE_H_
