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

#ifndef MODULES_MONITOR_SYSMON_PLUGIN_MANAGER_H_
#define MODULES_MONITOR_SYSMON_PLUGIN_MANAGER_H_

#include <map>
#include <memory>
#include <string>

#include <dlfcn.h>

#include "third_party/json/json.hpp"

#include "modules/common/macro.h"
#include "modules/common/log.h"
#include "modules/monitor/sysmon/base/plugin_interface.h"

namespace apollo {
namespace monitor {
namespace sysmon {

constexpr const char *PLUGIN_API_VER_SYM = "sysmon_plugin_api_ver";

//template<class EntryType, class MakerType>
template <class PluginType>
class PluginManager {
 public:
  static const unsigned int API_VERSION =
      apollo::monitor::sysmon::PLUGIN_API_VERSION_1;

  explicit PluginManager() {}

  virtual ~PluginManager() {
    for (auto el : dl_libs_) {
      if (el.second.dl_hdl) {
        ADEBUG << "Close plugin dll file " << el.first;
        dlclose(el.second.dl_hdl);
        el.second.dl_hdl = nullptr;
      }
    }
  }

  PluginType *create_plugin_inst(
      const std::string &nsp, const std::string &name) {
    std::string id = std::move(mk_global_id(nsp, name));
    return create_plugin_inst(id);
  }

  PluginType *create_plugin_inst(const std::string &id) {
    auto m_ent = m_plugin_makers_.find(id);
    if ( m_ent != m_plugin_makers_.end()) {
      // For now, we only work with one API version.
      // If we add incompatible plugin APIs in the future, add logic here to
      // support backward compatiblility.
      return (*(m_ent->second))(API_VERSION);
    }
    return nullptr;
  }

 protected:

  /// Loads a plugin .so library.
  ///@nsp name space to place all plugin(s) under.
  ///@lib_file library to load.
  bool load_plugin_lib(const std::string &nsp, const std::string &lib_file,
      const char *entry_sym)
  {
    if (!is_valid_name(nsp)) {
      AERROR << "Namespace name " << nsp
          << " is invalid; will NOT load plugin file" << lib_file;
      return false;
    }

    if (dl_libs_.find(lib_file) != dl_libs_.end()) {
      AERROR << "Plugin file " << lib_file << " has been loaded already";
      return false;
    }

    void *dl_hdl = dlopen(lib_file.c_str(), RTLD_LAZY | RTLD_LOCAL);
    if (!dl_hdl) {
      AERROR << "Failed to load plugin dll file " << lib_file
          << "; error: " << dlerror();
      return false;
    }

    unsigned int *api_ver = (unsigned int*)dlsym(dl_hdl, PLUGIN_API_VER_SYM);
    if (!api_ver) {
      AERROR << "Symbol " << PLUGIN_API_VER_SYM << " not found in plugin file "
          << lib_file;
      dlclose(dl_hdl);
      return false;
    }

    if (*api_ver < PLUGIN_API_VERSION_1) {
      // We can't really go lower than that ...
      AERROR << "Plugin API version " << *api_ver << "from plugin file "
          << lib_file << " is not supported";
      dlclose(dl_hdl);
      return false;
    }

    PluginEntry<PluginType> *tx = (PluginEntry<PluginType>*)
        dlsym(dl_hdl, entry_sym);

    if (!tx) {
      AERROR << "Symbol " << entry_sym << " not found in plugin file "
          << lib_file;
      dlclose(dl_hdl);
      return false;
    }

    int cnt = 0;
    for (int i = 0; ; ++i) {
      if (tx[i].get_name()) {
        std::string m_name = std::string(tx[i].get_name());
        if (is_valid_name(m_name)) {
          std::string n_name = mk_global_id(nsp, m_name);
          if (m_plugin_makers_.find(n_name) != m_plugin_makers_.end()) {
            AWARN << "Duplicate metric name " << n_name
                << "; existing plugin maker will be overwritten.";
          }
          ADEBUG << "Added metric " << n_name;
          m_plugin_makers_[std::move(n_name)] = tx[i].get_inst_creator();
          ++cnt;
        } else {
          AERROR << "Plugin name " << m_name
              << " is invalid; plugin will not be enabled.";
        }
      } else {
        break;
      }
    }

    if (cnt <= 0) {
      AWARN << "No metric loaded from plugin dll file " << lib_file;
      dlclose(dl_hdl);
    } else {
      dl_libs_[lib_file] = {nsp, dl_hdl};
      AINFO << "# of metrics loaded from plugin dll file " << lib_file
          << " under namespace " << nsp << ": " << cnt;
    }
    return true;
  }

  struct LibInfo {
    std::string nsp;
    void *dl_hdl;
  };

  std::map<std::string, LibInfo> dl_libs_;
  std::map<std::string, PluginInstCreator<PluginType>*> m_plugin_makers_;

  DISALLOW_COPY_AND_ASSIGN(PluginManager);
};

class MetricQuerentPluginManager
    : public PluginManager<MetricQuerentInterface> {
 public:
  explicit MetricQuerentPluginManager() {}

  /// Loads a plugin .so library.
  ///@nsp name space to place all metric querent(s) under.
  ///@lib_file library to load.
  bool load_plugin(const std::string &nsp, const std::string &lib_file) {
    return load_plugin_lib(nsp, lib_file, "sysmon_metric_querent_list");
  }

 protected:
  DISALLOW_COPY_AND_ASSIGN(MetricQuerentPluginManager);
};

class MetricDataDesPluginManager
    : public PluginManager<MetricDataDesInterface> {
 public:
  explicit MetricDataDesPluginManager() {}

  /// Loads a plugin .so library.
  ///@nsp name space to place all metric querent(s) under.
  ///@lib_file library to load.
  bool load_plugin(const std::string &nsp, const std::string &lib_file) {
    return load_plugin_lib(nsp, lib_file, "sysmon_metric_datades_list");
  }

 protected:
  DISALLOW_COPY_AND_ASSIGN(MetricDataDesPluginManager);
};

}  // namespace sysmon
}  // namespace monitor
}  // namespace apollo

#endif  // MODULES_MONITOR_SYSMON_PLUGIN_MANAGER_H_
