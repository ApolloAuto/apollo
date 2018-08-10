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

#ifndef MODULES_MONITOR_SYSMON_PLUGIN_INTERFACE_H_
#define MODULES_MONITOR_SYSMON_PLUGIN_INTERFACE_H_

#include <functional>
#include <string>

// nlohmann json
#include "third_party/json/json.hpp"

namespace apollo {
namespace monitor {
namespace sysmon {

// @todo: unicode support in name space and id.

constexpr unsigned int PLUGIN_API_VERSION_1 = 1;

/// Memory allocator interface
struct MemAllocInterface {
  /// Allocates memory of the given size.
  /// @returns pointer to allocated memory, nullptr if allocation failed.
  virtual void *alloc(size_t sz) = 0;
  /// Frees a previously allocated memory.
  virtual void free(void *) = 0;
};

/// Buffer that provides an API to get buffer size.
struct SizedBufferInterface {
  /// Returns size of the buffer.
  virtual size_t get_sz() const = 0;
  /// Returns pointer to buffer memory.
  virtual uint8_t *get_buf() = 0;
};

/// Resizable buffer that provides API to reserve capacity.
struct ResizableBufferInterface : public SizedBufferInterface {
  /// Reserves (allocates) buffer memory for the the given size.
  /// @returns pointer to allocated memory, nullptr if allocation failed.
  virtual uint8_t *reserve(size_t sz) = 0;
  /// Reserves buffer memory for the the given size and copy data from old
  /// buffer to the newly allocated buffer.
  /// @param cp_sz # of bytes to copy from old buffer to the new one;
  ///   actual copied size will the min of (old buffer size, new buffer size,
  ///   cp_sz).
  /// @returns pointer to allocated memory, nullptr if allocation failed.
  virtual uint8_t *reserve_and_copy(size_t sz, size_t cp_sz) = 0;
};

/// Types of data format for serialization/deserializations.
enum struct DataSerDesType {
  /// Json (string)
  JSON = 1,
  /// Binary Json
  BSON = 2,
  /// Any metric-specific (cross-platform) blob (string or binary).
  BLOB = 4,
  /// Host memory blob; may be used to improve performance when both querent &
  /// data consumer are running on the same host.
  HOST_BLOB = 8
};

/// Resuld ode for data serialization/deserializations.
enum class DataSerDesResult {
  OK = 0,
  NOT_IMPL = 1,
  DATA_ERR = 2,
  NO_MEM = 3
};


/// Metric data interface.
struct MetricDataInterface {
  /// Blob memory size requirement for seralization operation.
  enum struct BlobSzReq {
    /// Blob size is known and fixed at init time, and thus is always the same.
    FIXED_INIT = 0,
    /// Blob size is known prior to seralization, but may be different
    /// each time.
    KNOWN_PER_OP = 1,
    /// Blob size is unknown, memory may be requested progressively.
    PROGRESSIVE = 2
  };

  static const size_t BLOB_SZ_UNKNOWN = 0;

  virtual ~MetricDataInterface() {}

  /// Returns serializer capability (e.g., JSON | BSON).
  virtual unsigned int get_serializer_capability() const = 0;

  /// Creates a copy.
  virtual MetricDataInterface *mk_copy() const = 0;

  /// Serializes data to a json string.
  virtual std::string to_json_str() const = 0;

  /// Returns type of memory size requirement for serialization operation;
  /// as a hint for application to provide optimized custom memory allocator.
  virtual BlobSzReq blob_sz_req_type(DataSerDesType dt) const = 0;

  /// Returns memory size needed to seralize data to the given type.
  /// May return BLOB_SZ_UNKNOWN for PROGRESSIVE type.
  /// Can be used by application to provide optimized custom memory allocator.
  virtual size_t blob_sz_required(DataSerDesType dt) const {
    return BLOB_SZ_UNKNOWN;
  }

  virtual DataSerDesResult to_json_bin(ResizableBufferInterface *buf) const
  { return DataSerDesResult::NOT_IMPL; }

  virtual DataSerDesResult to_host_blob(ResizableBufferInterface *buf) const
  { return DataSerDesResult::NOT_IMPL; }

  virtual DataSerDesResult to_serdes_blob(ResizableBufferInterface *buf) const
  { return DataSerDesResult::NOT_IMPL; }
};

/// Metric data derserializer interface.
class MetricDataDesInterface {
 public:
  virtual ~MetricDataDesInterface() {}

  virtual bool config(const nlohmann::json *cfg) {
    save_config(cfg);
    return true;
  }

  /// Returns deserializer capability (e.g., JSON | BSON).
  virtual unsigned int get_des_capbility() const = 0;

  /// Receives data in a json string.
  virtual bool rcv_json_str(const std::string &jstr) = 0;

  /// Receive data in json binary.
  virtual DataSerDesResult rcv_json_bin(const SizedBufferInterface &buf)
  { return DataSerDesResult::NOT_IMPL; }

  /// Receive data in host blob.
  virtual DataSerDesResult rcv_host_blob(const SizedBufferInterface &buf)
  { return DataSerDesResult::NOT_IMPL; }

  /// Receive data in cross-platform blob.
  virtual DataSerDesResult rcv_serdes_blob(const SizedBufferInterface &buf)
  { return DataSerDesResult::NOT_IMPL; }

  /// Prints to a string for on-screen display etc., simple version.
  virtual std::string to_str_simp() const = 0;

  /// Prints to a string for on-screen display etc., verbose version.
  virtual std::string to_str_verbose() const = 0;

  /// Gets data in Json representation.
  virtual const nlohmann::json &get_json_str() const = 0;

  /// Gets the config.
  const nlohmann::json &get_cfg() const { return config_; }

 protected:
  void save_config(const nlohmann::json *cfg) {
    if (cfg) { config_ = *cfg; }
  }

  nlohmann::json config_;
};

constexpr unsigned int MICROS_IN_SEC  = 1000000;

/// Interface of a class that does metric queries.
class MetricQuerentInterface {
 public:
  struct MetricQueryType {
    static const unsigned int QT_UNDEFED = 0;
    /// Only needs to query once at the beginning and data never changes.
    static const unsigned int QT_ONCE = 1;
    /// After query result is otained, data never changes; but data may not
    /// be available at the beginning, shall keep trying until then.
    static const unsigned int QT_UNTIL_DONE = 2;
    /// Data keeps changing, shall query periodically to get snapshot.
    static const unsigned int QT_PERIODIC = 3;
    /// Data changes at unpredictable interval, should use async query
    /// (callback).
    static const unsigned int QT_ASYNC = 4;
  };

  struct MetricQuerySpeed {
    static const unsigned int UNDEFED = 0;
    static const unsigned int INSTANT = 1;
    static const unsigned int FAST = 2;
    static const unsigned int MEDIUM = 3;
    static const unsigned int SLOW = 4;
  };

  /// Result code of metric queries; defined as bitmask.
  enum struct MetricQueryResult {
    /// Query was succesful.
    OK = 1,
    /// Not initialized.
    NOT_INIT = 2,
    /// Query result not available.
    NOT_AVAILABLE = 4,
    /// The hardware being queried is not present.
    NOT_PRESENT = 8,
    /// Query request rejected (too often, not supported etc.).
    REJECTED = 16,
    /// Non-fatal error occurred, can retry future queries.
    ERROR = 32,
    /// Fatal error, no future query should be done.
    FATAL = 64
  };

  /// Data pointed to by data is only guaranteed to be valid within the
  /// scope of the callback.
  /// Callee makes a copy of *data if needed.
  typedef std::function<void(MetricQuerentInterface *, MetricQueryResult,
      MetricDataInterface *data, const std::string &err)> QueryResultCallbackFn;

  struct QueryAttr {
    QueryAttr() : query_type(MetricQueryType::QT_UNDEFED),
        may_block(false), query_speed(MetricQuerySpeed::UNDEFED),
        min_query_intv_us(0), dft_query_intv_us(0)
    {}

    QueryAttr(
        unsigned int q_type, bool block, unsigned int speed,
            unsigned int min_intv_us, unsigned int dft_intv_us)
        : query_type(q_type),
        may_block(block), query_speed(speed),
        min_query_intv_us(min_intv_us),
        dft_query_intv_us(dft_intv_us)
    {}

    /// @see MetricQueryType
    unsigned int query_type;

    /// True to indicate query may block (e.g., may go into sleep, or wait on
    /// operations that may sleep); as a hint for scheduling.
    bool may_block;

    /// How fast query will finish; as a hint for scheduling.
    unsigned int query_speed;

    /// Minimum query interval in micro-seconds (for type QT_UNTIL_DONE
    /// & QT_PERIODIC). Can be 0.
    unsigned int min_query_intv_us;

    /// Default (suggested) query interval in micro-seconds (
    /// for type QT_UNTIL_DONE & QT_PERIODIC).
    unsigned int dft_query_intv_us;
  };

  virtual ~MetricQuerentInterface() {}

  /// Initialization function.
  /// @return true if successful.
  /// A metric querent must NOT be used if this function returns false.
  virtual bool init(const nlohmann::json *cfg) {
    save_config(cfg);
    return true;
  }

  /// Runs the query. Stores query result data in m_data and error message
  /// in err if there is any error.
  /// Caller takes ownership of *m_data.
  virtual MetricQueryResult query(
      MetricDataInterface **m_data, std::string *err) = 0;

  /// For metric type of QT_ASYNC.
  /// @return true if callback function is registered; false if
  /// async mode is not supported.
  virtual bool register_result_cb(QueryResultCallbackFn cb_fn) {
    return false;
  }

  /// Closes the querent instance. It must NOT be used to run query again
  /// after this function is called.
  virtual void close() {}

  /// Gets query attributes of the metric querent.
  const QueryAttr &get_query_attr() const { return q_attr_; }

  /// Gets configurations of this metric querent.
  const nlohmann::json &get_cfg() const { return config_; }

 protected:
  void save_config(const nlohmann::json *cfg) {
    if (cfg) { config_ = *cfg; }
  }

  QueryAttr q_attr_;

  nlohmann::json config_;
};

/// Type of function that creates a plugin instance.
/// It takes 2 arguments: plug-in API version (supported by callee), and
/// pointer to a memory location for the new instance (must be big enough).
/// If mem is null, will use default new to create instance.
template <class PluginType>
using PluginInstCreator = std::function<PluginType*(unsigned int api_ver)>;


/// This character must not be used in name space, metric id (name) or
/// metric deserializer plugin names.
constexpr char NAMING_RSVD_CHAR = ':';

// Convenient inline functions for working with names (id).  {

/// Returns if the given name is valid for using as id.
inline bool is_valid_name(const std::string &name) {
  return name.find(NAMING_RSVD_CHAR) == std::string::npos;
}

/// Returns a global id for the given name under namespace nsp.
inline std::string mk_global_id(
    const std::string &nsp, const std::string &name) {
  return nsp + NAMING_RSVD_CHAR + name;
}

//  }

template <class PluginType_>
struct PluginEntry {
  /// Name of the plugin; must pass is_valid_name() check.
  const char *name;
  /// Description about this plugin.
  const char *description;
  /// Function to create an instance of the plugin.
  PluginInstCreator<PluginType_> inst_creator;

  const char *get_name() const { return name; }

  const char *get_description() const { return description; }

  PluginInstCreator<PluginType_> *get_inst_creator() { return &inst_creator; }

  typedef PluginType_ PluginType;
};

typedef PluginEntry<MetricQuerentInterface> MetricQuerentPluginEntry;

typedef PluginEntry<MetricDataDesInterface> MetricDataDesPluginEntry;


#define _SYSMON_TYPE_TO_CREATOR_NAME(TN) mk_##TN

// Convenient macros for creating plugin "creator".  {

// Only current API version is supported by the querent creator.
// Provides a customer implementation (instead of using this one) to support
// backward compabitility when there are incompatible new API versions
// in the future.
#define DCL_METRIC_QUERENT_CREATOR(MQT) \
apollo::monitor::sysmon::MetricQuerentInterface*  \
_SYSMON_TYPE_TO_CREATOR_NAME(MQT)(unsigned int api_ver)  \
{ if (api_ver > PLUGIN_API_VERSION_1) { return nullptr; }  \
  return new MQT(); }

// Only current API version is supported by the data-deserializer creator.
// Provides a customer implementation (instead of using this one) to support
// backward compabitility when there are incompatible new API versions
// in the future.
#define DCL_METRIC_DATADES_CREATOR(DDT) \
apollo::monitor::sysmon::MetricDataDesInterface*  \
_SYSMON_TYPE_TO_CREATOR_NAME(DDT)(unsigned int api_ver)  \
{ if (api_ver > PLUGIN_API_VERSION_1) { return nullptr; }  \
  return new DDT(); }

//  }

// { Symbols exported by a plugin dll.

/// API version; must be exported by all plugin dll.
/// We want to make this explicit to avoid potential mistakes.
#define DCL_SYSMON_PLUGIN_API_VER(version)  \
extern "C" unsigned int sysmon_plugin_api_ver = version;

/// A metric querent plug-in dynamically linked library must provide
/// symbol sysmon_metric_querent_list which points to a list of
/// MetricQuerentPluginEntry.
/// The last entry must have a nullptr for name to indicate it's the end.
#define DCL_METRIC_QUERENT_LIST(list...)  \
extern "C" apollo::monitor::sysmon::MetricQuerentPluginEntry  \
sysmon_metric_querent_list[] = list;

/// A metric data deserializer plug-in dynamically linked library must provide
/// symobl sysmon_metric_datades_list which points to a list of
/// MetricDataDesPluginEntry.
/// The last entry must have a nullptr for name to indicate it's the end.
#define DCL_METRIC_DATADES_LIST(list...)  \
extern "C" apollo::monitor::sysmon::MetricDataDesPluginEntry  \
sysmon_metric_datades_list[] = list;

// }

}  // namespace sysmon
}  // namespace monitor
}  // namespace apollo

#endif  // MODULES_MONITOR_SYSMON_PLUGIN_INTERFACE_H_
