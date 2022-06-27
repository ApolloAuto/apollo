/******************************************************************************
 * Copyright 2021 The Apollo Authors. All Rights Reserved.
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

#include <map>

#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>

#include "nlohmann/json.hpp"

#include "modules/dreamview/proto/account_info.pb.h"
#include "modules/dreamview/proto/configuration_profile_status.pb.h"

#include "cyber/cyber.h"
#include "modules/dreamview/backend/common/curl_request.h"
#include "modules/dreamview/backend/handlers/websocket_handler.h"

/**
 * @namespace apollo::dreamview
 * @brief apollo::dreamview
 */
namespace apollo {
namespace dreamview {

/**
 * @class AccountUpdater
 * @brief WebSockerHandler wraper to communicate to frontend for updating
 * configuration
 */
class Configurator {
 public:
  /**
   * @brief Constructor with websocket handler
   * @param websocket Pointer of websocket handler that has been attached to
   * the server.
   */
  Configurator(WebSocketHandler* websocket);

  ~Configurator();

  /**
   * @breif Start
   */
  void Start();
  void Stop();

 private:
  void RegisterMessageHandlers();
  void UpdateAccountInfo(const std::shared_ptr<AccountInfo>& msg);
  void ReloadConfigProfileStatus();
  const std::string GetCacheDir();
  const std::string GetApolloRootDir();
  const std::string GetSessionFilePath();
  const std::string GetSessionID();
  const std::string GetCookiesFilePath();
  const std::string GetVehicleTypeName(uint32_t vtype);
  const std::string GetInstalledConfigurationVersion(ConfigurationProfile*);
  ConfigurationProfile::Status GetConfigurationStatus(ConfigurationProfile*);
  const std::string GetConfigurationDownloadPath(ConfigurationProfile*);
  const std::string GetConfigurationDownloadPath(VehicleInfo*);
  const std::string GetConfigurationBackupName(ConfigurationProfile*);
  const std::string GetConfigurationBackupPath();
  const std::string GetConfigurationBackupPath(ConfigurationProfile*);
  const std::string GetConfigurationPath();
  const std::string GetConfigurationPath(const std::string name);
  const std::string GetConfigurationPath(ConfigurationProfile* profile);
  const std::string GetConfigurationPath(VehicleInfo* vehicle);
  const std::string GetVehicleSnFilePath();
  void LoadVehicleSn();
  void SaveVehicleSn();
  void BroadcastConfigurationProfileStatus();
  const std::string FetchConfigurationChecksum(ConfigurationProfile*);
  bool FetchConfiguration(ConfigurationProfile*, bool);
  bool FetchConfiguration(VehicleInfo*, bool);
  bool UploadConfiguration(ConfigurationProfile*);
  bool BackupConfiguration(ConfigurationProfile*);
  bool BackupConfiguration(ConfigurationProfile*, const std::string path);
  bool RestoreConfiguration(ConfigurationProfile*);
  bool RestoreConfiguration(ConfigurationProfile*, const std::string path);
  bool InstallConfiguration(ConfigurationProfile* profile);
  bool InstallConfiguration(ConfigurationProfile*, const std::string path);
  bool InstallConfiguration(VehicleInfo* vehicle);
  bool InstallConfiguration(VehicleInfo* vehicle, const std::string path);
  ConfigurationProfile* GetProfile(uint64_t id);
  VehicleInfo* GetVehicle(uint64_t id);
  void UpdateProfileStatus(uint64_t id, ConfigurationProfile::Status status);
  void UpdateProfileStatus(ConfigurationProfile* profile,
                           ConfigurationProfile::Status status);
  void RefreshProfileStatus();

  AccountInfo account_info_;
  ConfigurationProfileStatus config_profile_status_;
  std::unique_ptr<cyber::Node> node_;
  std::shared_ptr<cyber::Reader<AccountInfo>> account_info_reader_;
  WebSocketHandler* websocket_;

  std::map<std::string, std::shared_ptr<CurlRequest>> cached_request_;
  // Mutex to protect concurrent download requests
  // NOTE: Use boost until we upgrade to std version with rwlock support.
  boost::shared_mutex mutex_;
  boost::shared_mutex fetch_mutex_;
  boost::shared_mutex fetch_mutex_dkit_;
  boost::shared_mutex fetch_mutex_dkit_standard_;
  boost::shared_mutex fetch_mutex_dkit_advance_ne_s_;
  boost::shared_mutex fetch_mutex_dkit_advance_sne_r_;
};

}  // namespace dreamview
}  // namespace apollo
