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

#include "modules/dreamview/backend/configurator/configurator.h"

#include <unistd.h>

#include <fstream>
#include <iostream>
#include <map>
#include <thread>

#include <boost/beast/core/detail/base64.hpp>
#include <curl/curl.h>

#include "modules/common/util/json_util.h"
#include "modules/dreamview/backend/account/account_gflags.h"
#include "modules/dreamview/backend/common/curl_request.h"

namespace apollo {
namespace dreamview {

using apollo::common::util::JsonUtil;
using Json = nlohmann::json;

namespace {

static const uint32_t VEHICLE_TYPE_DKIT = 1;
static const uint32_t VEHICLE_TYPE_DKIT_STANDARD = 2;
static const uint32_t VEHICLE_TYPE_DKIT_ADVANCE_NE_S = 3;
static const uint32_t VEHICLE_TYPE_DKIT_ADVANCE_SNE_R = 4;
static const uint32_t VEHICLE_TYPE_DKIT_LITE_S = 5;
static const uint32_t VEHICLE_TYPE_DKIT_STANDARD_S = 6;
static const uint32_t VEHICLE_TYPE_DKIT_CHALLENGE = 7;

static const std::map<uint32_t, std::string> VTYPE_NAME_MAPPING = {
    {VEHICLE_TYPE_DKIT, "dev_kit"},
    {VEHICLE_TYPE_DKIT_STANDARD, "dev_kit_standard"},
    {VEHICLE_TYPE_DKIT_ADVANCE_NE_S, "dev_kit_advanced_ne-s"},
    {VEHICLE_TYPE_DKIT_ADVANCE_SNE_R, "dev_kit_advanced_sne-r"},
    {VEHICLE_TYPE_DKIT_LITE_S, "dev_kit_lite_s"},
    {VEHICLE_TYPE_DKIT_STANDARD_S, "dev_kit_standard_s"},
    {VEHICLE_TYPE_DKIT_CHALLENGE, "dev_kit_challenge"},
};

} // namespace

using Json = nlohmann::json;
using WLock = boost::unique_lock<boost::shared_mutex>;

Configurator::Configurator(WebSocketHandler *websocket)
    : node_(cyber::CreateNode("configuration")), websocket_(websocket) {
  ReloadConfigProfileStatus();
  RegisterMessageHandlers();
}

Configurator::~Configurator() { Stop(); }

const std::string Configurator::GetCacheDir() {
  const std::string cache_dir =
      cyber::common::GetEnv("APOLLO_CACHE_DIR", "/apollo/.cache");
  return cache_dir;
}

const std::string Configurator::GetApolloRootDir() {
  const std::string root_dir =
      cyber::common::GetEnv("APOLLO_ROOT_DIR", "/apollo");
  return root_dir;
}

const std::string Configurator::GetSessionFilePath() {
  const std::string cache_dir = GetCacheDir();
  const std::string filepath = cache_dir + "/session";
  return filepath;
}

const std::string Configurator::GetSessionID() {
  const std::string filepath = GetSessionFilePath();
  std::ifstream fin(filepath);
  std::string sid;
  if (fin.is_open()) {
    fin >> sid;
    fin.close();
  }
  return sid;
}

const std::string Configurator::GetCookiesFilePath() {
  const std::string cache_dir = GetCacheDir();
  const std::string filepath = cache_dir + "/cookies";
  return filepath;
}

const std::string Configurator::GetVehicleTypeName(uint32_t vtype) {
  auto iter = VTYPE_NAME_MAPPING.find(vtype);
  if (iter == VTYPE_NAME_MAPPING.end()) {
    return "";
  }
  return iter->second;
}

const std::string
Configurator::GetInstalledConfigurationVersion(ConfigurationProfile *profile) {
  // TODO: Use `id` to identify profile
  std::string conf_path = GetConfigurationPath(profile);
  std::string version_file = conf_path + "/version";
  std::ifstream fin(version_file);
  std::string version;
  if (fin.is_open()) {
    fin >> version;
    fin.close();
  }
  return version;
}

ConfigurationProfile::Status
Configurator::GetConfigurationStatus(ConfigurationProfile *profile) {
  std::string conf_path = GetConfigurationPath(profile);
  if (cyber::common::DirectoryExists(conf_path)) {
    return ConfigurationProfile::Status::ConfigurationProfile_Status_CLEAN;
  } else {
    return ConfigurationProfile::Status::ConfigurationProfile_Status_UNKNOWN;
  }
}

const std::string
Configurator::GetConfigurationDownloadPath(ConfigurationProfile *profile) {
  const std::string cache_dir = GetCacheDir();
  const std::string profile_key = std::to_string(profile->vehicle_id()) + "." +
                                  std::to_string(profile->id());
  const std::string fname = "calibration.data." + profile_key + ".tar.gz";
  const std::string fpath = cache_dir + "/" + fname;
  return fpath;
}

const std::string
Configurator::GetConfigurationDownloadPath(VehicleInfo *vehicle) {
  const std::string cache_dir = GetCacheDir();
  const std::string profile_key = std::to_string(vehicle->id()) + ".default";
  const std::string fname = "calibration.data." + profile_key + ".tar.gz";
  const std::string fpath = cache_dir + "/" + fname;
  return fpath;
}

const std::string
Configurator::GetConfigurationBackupName(ConfigurationProfile *profile) {
  const std::string profile_key = std::to_string(profile->vehicle_id()) + "." +
                                  std::to_string(profile->id());
  return "calibration.data." + profile_key + ".tar.gz";
}

const std::string Configurator::GetConfigurationBackupPath() {
  const std::string cache_dir = GetCacheDir();
  // TODO: get from environment or configuration file
  return cache_dir + "/backup";
}

const std::string
Configurator::GetConfigurationBackupPath(ConfigurationProfile *profile) {
  const std::string fname = GetConfigurationBackupName(profile);
  return GetConfigurationBackupPath() + "/" + fname;
}

const std::string Configurator::GetConfigurationPath() {
  const std::string root_dir = GetApolloRootDir();
  const std::string conf_path = root_dir + "/modules/calibration/data";
  return conf_path;
}

const std::string Configurator::GetConfigurationPath(const std::string name) {
  return GetConfigurationPath() + "/" + name;
}

const std::string
Configurator::GetConfigurationPath(ConfigurationProfile *profile) {
  return GetConfigurationPath() + "/" + profile->name();
}

const std::string Configurator::GetConfigurationPath(VehicleInfo *vehicle) {
  return GetConfigurationPath() + "/" + GetVehicleTypeName(vehicle->vtype());
}

const std::string Configurator::GetVehicleSnFilePath() {
  const std::string cache_dir = GetCacheDir();
  const std::string filepath = cache_dir + "/vehiclesn";
  return filepath;
}

void Configurator::LoadVehicleSn() {
  // TODO: detect by canbus
  const std::string filepath = GetVehicleSnFilePath();
  std::ifstream fin(filepath);
  std::string vin;
  if (fin.is_open()) {
    fin >> vin;
    config_profile_status_.set_vehicle_in(vin);
    fin.close();
  }
}

void Configurator::SaveVehicleSn() {
  const std::string filepath = GetVehicleSnFilePath();
  cyber::common::EnsureDirectory(GetCacheDir());
  std::ofstream fout(filepath);
  if (fout.is_open()) {
    fout << config_profile_status_.vehicle_in();
    fout.close();
  }
}

void Configurator::BroadcastConfigurationProfileStatus() {
  websocket_->BroadcastData(
      JsonUtil::ProtoToTypedJson("ConfigurationProfileStatus",
                                 config_profile_status_)
          .dump());
}

const std::string
Configurator::FetchConfigurationChecksum(ConfigurationProfile *profile) {
  // TODO: Implement, depend on studio backend api
  return "";
}

bool Configurator::FetchConfiguration(ConfigurationProfile *profile,
                                      bool is_original) {
  std::string url = FLAGS_dreamview_account_server_api_entry +
                    "/user/api/vehicle/profile/tarball";
  cyber::common::EnsureDirectory(GetCacheDir());
  auto req = std::shared_ptr<CurlRequest>(new CurlRequest(url));
  std::map<std::string, std::string> queries;
  queries["vehicleId"] = std::to_string(profile->vehicle_id());
  queries["profileId"] = std::to_string(profile->id());
  queries["isOriginal"] = std::to_string(is_original);
  req->AddParams(queries);
  std::map<std::string, std::string> headers;
  headers["Accept"] = "*/*";
  headers["Host"] = "studio.apollo.auto";
  headers["User-Agent"] = "Apollo Dreamview";
  req->AddHeaders(headers);

  req->SetCookieFile(GetCookiesFilePath());
  req->SetCookieJar(GetCookiesFilePath());

  const std::string sid(GetSessionID());
  char bduss_buffer[512];
  std::pair<std::size_t, std::size_t> wrc =
      boost::beast::detail::base64::decode(bduss_buffer, sid.c_str(),
                                           sid.size());
  const std::string bduss(bduss_buffer, wrc.first);
  req->SetCookie("BDUSS", bduss, "/", ".apollo.auto", 31536000);

  req->SetSslVerifyHost(FLAGS_dreamview_curl_ssl_verify_host);
  req->SetSslVerifyPeer(FLAGS_dreamview_curl_ssl_verify_peer);
  req->SetVerbose(FLAGS_dreamview_curl_verbose);

  bool result = true;
  // Caching request for cancelation from another thread
  const std::string cache_key = std::to_string(profile->vehicle_id()) + "." +
                                std::to_string(profile->id());
  cached_request_[cache_key] = req;
  if (req->perform()) {
    // TODO: Verify content
    cyber::common::EnsureDirectory(GetCacheDir());
    std::ofstream fout(GetConfigurationDownloadPath(profile));
    if (fout.is_open()) {
      fout << req->Response();
      fout.close();
    } else {
      AERROR << "Fail to open file: " << GetConfigurationDownloadPath(profile);
      result = false;
    }
  } else {
    AERROR << "Failed to fetch configuration for profile: " << profile->id();
    result = false;
  }
  cached_request_.erase(cache_key);
  return result;
}

bool Configurator::FetchConfiguration(VehicleInfo *vehicle, bool is_original) {
  std::string url = FLAGS_dreamview_account_server_api_entry +
                    "/user/api/vehicle/profile/tarball";
  cyber::common::EnsureDirectory(GetCacheDir());
  auto req = std::shared_ptr<CurlRequest>(new CurlRequest(url));
  std::map<std::string, std::string> queries;
  queries["vehicleId"] = std::to_string(vehicle->id());
  queries["isOriginal"] = std::to_string(is_original);
  req->AddParams(queries);

  std::map<std::string, std::string> headers;
  headers["Accept"] = "*/*";
  headers["Host"] = "studio.apollo.auto";
  headers["User-Agent"] = "Apollo Dreamview";
  req->AddHeaders(headers);

  req->SetCookieFile(GetCookiesFilePath());
  req->SetCookieJar(GetCookiesFilePath());

  const std::string sid(GetSessionID());
  char bduss_buffer[512];
  std::pair<std::size_t, std::size_t> wrc =
      boost::beast::detail::base64::decode(bduss_buffer, sid.c_str(),
                                           sid.size());
  const std::string bduss(bduss_buffer, wrc.first);
  req->SetCookie("BDUSS", bduss, "/", ".apollo.auto", 31536000);

  req->SetSslVerifyHost(FLAGS_dreamview_curl_ssl_verify_host);
  req->SetSslVerifyPeer(FLAGS_dreamview_curl_ssl_verify_peer);
  req->SetVerbose(FLAGS_dreamview_curl_verbose);

  bool result = true;
  // Caching request for cancelation from another thread
  const std::string cache_key = std::to_string(vehicle->id()) + ".default";
  // At present, vtype is same to id, use as key
  cached_request_[cache_key] = req;
  if (req->perform()) {
    // TODO: Verify content
    cyber::common::EnsureDirectory(GetCacheDir());
    std::ofstream fout(GetConfigurationDownloadPath(vehicle));
    if (fout.is_open()) {
      fout << req->Response();
      fout.close();
    } else {
      AERROR << "Fail to open file: " << GetConfigurationDownloadPath(vehicle);
      result = false;
    }
  } else {
    AERROR << "Failed to fetch configuration for vehicle: " << vehicle->id();
    result = false;
  }
  cached_request_.erase(cache_key);
  return result;
}

bool Configurator::UploadConfiguration(ConfigurationProfile *profile) {
  // create profile tarball by backup action
  // TODO: create by gzip library or upgrade structured profile
  if (!BackupConfiguration(profile)) {
    // backup failed,
    return false;
  }
  std::string url = FLAGS_dreamview_account_server_api_entry +
                    "/user/api/vehicle/profile/tarball";
  cyber::common::EnsureDirectory(GetCacheDir());
  auto req = std::shared_ptr<CurlRequest>(new CurlRequest(url));
  std::map<std::string, std::string> queries;
  queries["vehicleId"] = std::to_string(profile->vehicle_id());
  queries["profileId"] = std::to_string(profile->id());
  req->AddParams(queries);

  std::map<std::string, std::string> headers;
  headers["Accept"] = "*/*";
  headers["Host"] = "studio.apollo.auto";
  headers["User-Agent"] = "Apollo Dreamview";
  req->AddHeaders(headers);

  req->SetCookieFile(GetCookiesFilePath());
  req->SetCookieJar(GetCookiesFilePath());

  const std::string sid(GetSessionID());
  char bduss_buffer[512];
  std::pair<std::size_t, std::size_t> wrc =
      boost::beast::detail::base64::decode(bduss_buffer, sid.c_str(),
                                           sid.size());
  const std::string bduss(bduss_buffer, wrc.first);
  req->SetCookie("BDUSS", bduss, "/", ".apollo.auto", 31536000);

  req->SetSslVerifyHost(FLAGS_dreamview_curl_ssl_verify_host);
  req->SetSslVerifyPeer(FLAGS_dreamview_curl_ssl_verify_peer);
  req->SetVerbose(FLAGS_dreamview_curl_verbose);

  req->AddFile(GetConfigurationBackupName(profile),
               GetConfigurationBackupPath(profile));

  bool result = true;
  const std::string cache_key = std::to_string(profile->vehicle_id()) + "." +
                                std::to_string(profile->id());
  cached_request_[cache_key] = req;
  if (req->perform()) {
    AINFO << "upload vehicle profile:" << profile->id() << " success";
  } else {
    AERROR << "failed to upload vehicle profile:" << profile->id();
    result = false;
  }
  cached_request_.erase(cache_key);
  return result;
}

bool Configurator::BackupConfiguration(ConfigurationProfile *profile) {
  const std::string backup_filepath = GetConfigurationBackupPath(profile);
  const std::string backup_dirpath = GetConfigurationBackupPath();
  cyber::common::EnsureDirectory(backup_dirpath);
  return BackupConfiguration(profile, backup_filepath);
}

bool Configurator::BackupConfiguration(ConfigurationProfile *profile,
                                       const std::string backup_path) {
  const std::string conf_path = GetConfigurationPath(profile);
  if (cyber::common::DirectoryExists(conf_path)) {
    std::string cmd;
    cmd.append("tar -zcf \"").append(backup_path).append("\"");
    cmd.append(" -C \"").append(conf_path).append("\"");
    cmd.append(" .");
    const int ret = std::system(cmd.c_str());
    if (ret == 0) {
      AINFO << "Backup " << conf_path << " to " << backup_path << " success";
      return true;
    } else {
      AERROR << "Backup " << conf_path << " to " << backup_path
             << " failed, command returns " << ret;
      return false;
    }
  } else {
    AWARN << conf_path << " not exist, ignore backup";
    return true;
  }
}

bool Configurator::RestoreConfiguration(ConfigurationProfile *profile) {
  const std::string backup_path = GetConfigurationBackupPath(profile);
  return RestoreConfiguration(profile, backup_path);
}

bool Configurator::RestoreConfiguration(ConfigurationProfile *profile,
                                        const std::string backup_path) {
  if (cyber::common::PathExists(backup_path)) {
    return InstallConfiguration(profile, backup_path);
  } else {
    AERROR << backup_path << " not exist, nothing to do";
    return false;
  }
}

bool Configurator::InstallConfiguration(ConfigurationProfile *profile) {
  return InstallConfiguration(profile, GetConfigurationDownloadPath(profile));
}

bool Configurator::InstallConfiguration(ConfigurationProfile *profile,
                                        const std::string src_path) {
  std::string conf_path = GetConfigurationPath(profile);
  cyber::common::EnsureDirectory(conf_path);
  std::string cmd;
  cmd.append("tar -zxf \"").append(src_path).append("\"");
  cmd.append(" -C \"").append(conf_path).append("\"");
  const int ret = std::system(cmd.c_str());
  if (ret == 0) {
    AINFO << "Install " << conf_path << " from " << src_path << " success";
    return true;
  } else {
    AERROR << "Install " << conf_path << " from " << src_path
           << " failed, command returns " << ret;
    return false;
  }
}

bool Configurator::InstallConfiguration(VehicleInfo *vehicle) {
  return InstallConfiguration(vehicle, GetConfigurationDownloadPath(vehicle));
}

bool Configurator::InstallConfiguration(VehicleInfo *vehicle,
                                        const std::string src_path) {
  std::string conf_path = GetConfigurationPath(vehicle);
  cyber::common::EnsureDirectory(conf_path);
  std::string cmd;
  cmd.append("tar -zxf \"").append(src_path).append("\"");
  cmd.append(" -C \"").append(conf_path).append("\"");
  const int ret = std::system(cmd.c_str());
  if (ret == 0) {
    AINFO << "Install " << conf_path << " from " << src_path << " success";
    return true;
  } else {
    AERROR << "Install " << conf_path << " from " << src_path
           << "failed, command returns " << ret;
    return false;
  }
}

VehicleInfo *Configurator::GetVehicle(uint64_t id) {
  for (int idx = 0; idx < account_info_.vehicles_size(); ++idx) {
    if (id == account_info_.vehicles(idx).id()) {
      return account_info_.mutable_vehicles(idx);
    }
  }
  // not found
  return nullptr;
}

ConfigurationProfile *Configurator::GetProfile(uint64_t id) {
  for (int idx = 0; idx < config_profile_status_.profiles_size(); ++idx) {
    if (id == config_profile_status_.profiles(idx).id()) {
      return config_profile_status_.mutable_profiles(idx);
    }
  }
  // Not found
  return nullptr;
}

void Configurator::UpdateProfileStatus(uint64_t id,
                                       ConfigurationProfile::Status status) {
  auto profile = GetProfile(id);
  if (profile == nullptr) {
    return;
  }
  return UpdateProfileStatus(profile, status);
}

void Configurator::UpdateProfileStatus(ConfigurationProfile *profile,
                                       ConfigurationProfile::Status status) {
  profile->set_status(status);
  AINFO << "Sending status: "
        << JsonUtil::ProtoToTypedJson("ConfigurationProfileStatus",
                                      config_profile_status_)
               .dump();
  BroadcastConfigurationProfileStatus();
}

void Configurator::RefreshProfileStatus() {
  for (int i = 0; i < config_profile_status_.profiles_size(); ++i) {
    auto profile = config_profile_status_.mutable_profiles(i);
    profile->set_status(GetConfigurationStatus(profile));
  }
  AINFO << "Config profile refreshed";
  BroadcastConfigurationProfileStatus();
}

void Configurator::RegisterMessageHandlers() {
  websocket_->RegisterConnectionReadyHandler(
      [this](WebSocketHandler::Connection *conn) {
        websocket_->SendData(
            conn, JsonUtil::ProtoToTypedJson("ConfigurationProfileStatus",
                                             config_profile_status_)
                      .dump());
      });
  websocket_->RegisterMessageHandler(
      "ConfigurationProfileStatus",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        try {
          RefreshProfileStatus();
        } catch (const std::exception &e) {
          AERROR << "ConfigurationProfileStatus process failed: " << e.what();
        }
      });
  websocket_->RegisterMessageHandler(
      "UpdateVehicleSn",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        try {
          std::string vin = json["vin"].get<std::string>();
          config_profile_status_.clear_vehicle_in();
          config_profile_status_.set_vehicle_in(vin);
          SaveVehicleSn();
        } catch (const std::exception &e) {
          AERROR << "update vehicle_in failed: " << e.what();
        }
      });
  websocket_->RegisterMessageHandler(
      "RequestVehicleProfileTarballDownload",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        try {
          // 64 bit number is serialized as string for javascript precision
          uint64_t vehicle_id =
              std::stoull(json["vehicleId"].get<std::string>());
          if (json.find("profileId") == json.end()) {
            // no profile_id, use default profile
            auto vehicle = GetVehicle(vehicle_id);
            if (vehicle == nullptr) {
              AWARN << "vehicle:" << vehicle_id << " not found";
              return;
            }
            std::thread t1([this, vehicle]() {
              if (FetchConfiguration(vehicle, false) &&
                  InstallConfiguration(vehicle)) {
                AINFO << "Install default profile success";
                // TODO: notification
                Json msg;
                msg["type"] = "DefaultVehicleProfileTarballDownloadSuccess";
                websocket_->BroadcastData(msg.dump());
              } else {
                AINFO << " Fetch or install default profile failed";
                // TODO: notification
                Json msg;
                msg["type"] = "DefaultVehicleProfileTarballDownloadFailed";
                websocket_->BroadcastData(msg.dump());
              }
            });
            t1.detach();
          }

          uint64_t profile_id =
              std::stoull(json["profileId"].get<std::string>());
          auto profile = GetProfile(profile_id);
          if (profile == nullptr) {
            // profile not found, do nothing
            AWARN << "profile:" << profile_id << " not found";
            return;
          }
          auto orig_status = profile->status();
          if (orig_status == ConfigurationProfile::Status::
                                 ConfigurationProfile_Status_RESETING ||
              orig_status == ConfigurationProfile::Status::
                                 ConfigurationProfile_Status_SYNCING ||
              orig_status == ConfigurationProfile::Status::
                                 ConfigurationProfile_Status_UPLOADING ||
              orig_status == ConfigurationProfile_Status_DOWNLOADING) {
            // Syning, do nothing
          } else {
            UpdateProfileStatus(profile,
                                ConfigurationProfile::Status::
                                    ConfigurationProfile_Status_DOWNLOADING);
            // backup, fetch and install
            if (BackupConfiguration(profile)) {
              std::thread t1([this, profile]() {
                if (FetchConfiguration(profile, false) &&
                    InstallConfiguration(profile)) {
                  AINFO << "Install success";
                  UpdateProfileStatus(profile, GetConfigurationStatus(profile));
                } else {
                  AINFO << "Fetch or install failed";
                  UpdateProfileStatus(profile, GetConfigurationStatus(profile));
                }
              });
              t1.detach();
            }
          }
        } catch (const std::exception &e) {
          AERROR << "RequestVehicleProfileTarballDownload process failed: "
                 << e.what();
        }
      });
  websocket_->RegisterMessageHandler(
      "CancelVehicleProfileTarballDownload",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        try {
          // 64 bit number is serialized as string for javascript precision
          uint64_t vehicle_id =
              std::stoull(json["vehicleId"].get<std::string>());
          if (json.find("profileId") == json.end()) {
            auto vehicle = GetVehicle(vehicle_id);
            if (vehicle == nullptr) {
              AWARN << "vehicle:" << vehicle_id << " not found";
              return;
            }
            const std::string cache_key =
                std::to_string(vehicle->id()) + ".default";
            auto reqit = cached_request_.find(cache_key);
            if (reqit != cached_request_.end()) {
              reqit->second->Abort();
              AINFO << "configuration download job canceled";
              // TODO: notification
              Json msg;
              msg["type"] = "DefaultVehicleProfileTarballDownloadCanceled";
              websocket_->BroadcastData(msg.dump());
            } else {
              AWARN << "vehicle:" << vehicle_id << " no cached request";
            }
          }
          uint64_t profile_id =
              std::stoull(json["profileId"].get<std::string>());
          auto profile = GetProfile(profile_id);
          if (profile == nullptr) {
            AWARN << "profile:" << profile_id << " not found";
            return;
          }
          auto orig_status = profile->status();
          if (orig_status == ConfigurationProfile::Status::
                                 ConfigurationProfile_Status_DOWNLOADING) {
            // Abort
            AINFO << "canceling configuration download job";
            const std::string cache_key =
                std::to_string(profile->vehicle_id()) + "." +
                std::to_string(profile->id());
            auto reqit = cached_request_.find(cache_key);
            if (reqit != cached_request_.end()) {
              reqit->second->Abort();
              AINFO << "configuration download job canceled";
              auto new_status = GetConfigurationStatus(profile);
              UpdateProfileStatus(profile, new_status);
            } else {
              // Nothing found, ignore
              AWARN << "profile:" << profile_id << " no cached request";
            }
          } else {
            // Not syning, do nothing
            AWARN << "profile:" << profile_id << " not downloading";
          }
        } catch (const std::exception &e) {
          AERROR << "CancelVehicleProfileTarballDownload process failed: "
                 << e.what();
        }
      });
  websocket_->RegisterMessageHandler(
      "RequestVehicleProfileTarballUpload",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        try {
          if (json.find("profileId") == json.end()) {
            AERROR << "no profile id specified, upload operation is not allow "
                      "for default profile";
            return;
          }
          uint64_t profile_id =
              std::stoull(json["profileId"].get<std::string>());
          auto profile = GetProfile(profile_id);
          if (profile == nullptr) {
            AWARN << "profile:" << profile_id << " not found";
            return;
          }
          auto orig_status = profile->status();
          if (orig_status == ConfigurationProfile::Status::
                                 ConfigurationProfile_Status_RESETING ||
              orig_status == ConfigurationProfile::Status::
                                 ConfigurationProfile_Status_SYNCING ||
              orig_status == ConfigurationProfile::Status::
                                 ConfigurationProfile_Status_UPLOADING ||
              orig_status == ConfigurationProfile_Status_DOWNLOADING) {
            // Syncing, do nothing
          } else {
            UpdateProfileStatus(profile,
                                ConfigurationProfile::Status::
                                    ConfigurationProfile_Status_UPLOADING);
            // backup, fetch and install
            std::thread t1([this, profile]() {
              if (UploadConfiguration(profile)) {
                AINFO << "Upload success";
                Json msg;
                msg["type"] = "RequestVehicleProfileTarballUploadSuccess";
                websocket_->BroadcastData(msg.dump());
              } else {
                AINFO << "Upload failed";
                Json msg;
                msg["type"] = "RequestVehicleProfileTarballUploadFailed";
                websocket_->BroadcastData(msg.dump());
                UpdateProfileStatus(profile, GetConfigurationStatus(profile));
              }
            });
            t1.detach();
          }
        } catch (const std::exception &e) {
          AERROR << "RequestVehicleProfileTarballUpload process failed: "
                 << e.what();
        }
      });
  websocket_->RegisterMessageHandler(
      "CancelVehicleProfileTarballUpload",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        try {
          if (json.find("profileId") == json.end()) {
            AERROR << "no profile id specified, upload operation is not allow "
                      "for default profile";
            return;
          }
          uint64_t profile_id =
              std::stoull(json["profileId"].get<std::string>());
          auto profile = GetProfile(profile_id);
          if (profile == nullptr) {
            AWARN << "profile:" << profile_id << " not found";
            return;
          }
          auto orig_status = profile->status();
          if (orig_status == ConfigurationProfile::Status::
                                 ConfigurationProfile_Status_UPLOADING) {
            // Abort
            AINFO << "canceling configuration download job";
            const std::string cache_key =
                std::to_string(profile->vehicle_id()) + "." +
                std::to_string(profile->id());
            auto reqit = cached_request_.find(cache_key);
            if (reqit != cached_request_.end()) {
              reqit->second->Abort();
              AINFO << "configuration download job canceled";
              auto new_status = GetConfigurationStatus(profile);
              UpdateProfileStatus(profile, new_status);
            } else {
              // Nothing found, ignore
              AWARN << "profile:" << profile_id << " no cached request";
            }
          } else {
            // Not uploading, do nothing
            AWARN << "profile:" << profile_id << " not uploading";
          }
        } catch (const std::exception &e) {
          AERROR << "CancelVehicleProfileTarballUpload process failed: "
                 << e.what();
        }
      });
  websocket_->RegisterMessageHandler(
      "RequestVehicleProfileTarballReset",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        try {
          // 64 bit number is serialized as string for javascript precision
          uint64_t vehicle_id =
              std::stoull(json["vehicleId"].get<std::string>());
          if (json.find("profileId") == json.end()) {
            // no profile_id, use default profile
            auto vehicle = GetVehicle(vehicle_id);
            if (vehicle == nullptr) {
              AWARN << "vehicle:" << vehicle_id << " not found";
              return;
            }
            std::thread t1([this, vehicle]() {
              if (FetchConfiguration(vehicle, true) &&
                  InstallConfiguration(vehicle)) {
                AINFO << "Install default profile success";
                // TODO: notification
                Json msg;
                msg["type"] = "DefaultVehicleProfileTarballDownloadSuccess";
                websocket_->BroadcastData(msg.dump());
              } else {
                AINFO << " Fetch or install default profile failed";
                // TODO: notification
                Json msg;
                msg["type"] = "DefaultVehicleProfileTarballDownloadFailed";
                websocket_->BroadcastData(msg.dump());
              }
            });
            t1.detach();
          }

          uint64_t profile_id =
              std::stoull(json["profileId"].get<std::string>());
          auto profile = GetProfile(profile_id);
          if (profile == nullptr) {
            // profile not found, do nothing
            AWARN << "profile:" << profile_id << " not found";
            return;
          }
          auto orig_status = profile->status();
          if (orig_status == ConfigurationProfile::Status::
                                 ConfigurationProfile_Status_RESETING ||
              orig_status == ConfigurationProfile::Status::
                                 ConfigurationProfile_Status_SYNCING ||
              orig_status == ConfigurationProfile::Status::
                                 ConfigurationProfile_Status_UPLOADING ||
              orig_status == ConfigurationProfile_Status_DOWNLOADING) {
            // Syning, do nothing
          } else {
            UpdateProfileStatus(profile,
                                ConfigurationProfile::Status::
                                    ConfigurationProfile_Status_RESETING);
            // backup, fetch and install
            if (BackupConfiguration(profile)) {
              std::thread t1([this, profile]() {
                if (FetchConfiguration(profile, true) &&
                    InstallConfiguration(profile)) {
                  AINFO << "Install success";
                  UpdateProfileStatus(profile, GetConfigurationStatus(profile));
                } else {
                  AINFO << "Fetch or install failed";
                  UpdateProfileStatus(profile, GetConfigurationStatus(profile));
                }
              });
              t1.detach();
            }
          }
        } catch (const std::exception &e) {
          AERROR << "RequestVehicleProfileReset process failed: " << e.what();
        }
      });
  websocket_->RegisterMessageHandler(
      "CancelVehicleProfileTarballReset",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        try {
          // 64 bit number is serialized as string for javascript precision
          uint64_t vehicle_id =
              std::stoull(json["vehicleId"].get<std::string>());
          if (json.find("profileId") == json.end()) {
            auto vehicle = GetVehicle(vehicle_id);
            if (vehicle == nullptr) {
              AWARN << "vehicle:" << vehicle_id << " not found";
              return;
            }
            const std::string cache_key =
                std::to_string(vehicle->id()) + ".default";
            auto reqit = cached_request_.find(cache_key);
            if (reqit != cached_request_.end()) {
              reqit->second->Abort();
              AINFO << "configuration download job canceled";
              // TODO: notification
              Json msg;
              msg["type"] = "DefaultVehicleProfileTarballDownloadCanceled";
              websocket_->BroadcastData(msg.dump());
            } else {
              AWARN << "vehicle:" << vehicle_id << " no cached request";
            }
          }
          uint64_t profile_id =
              std::stoull(json["profileId"].get<std::string>());
          auto profile = GetProfile(profile_id);
          if (profile == nullptr) {
            AWARN << "profile:" << profile_id << " not found";
            return;
          }
          auto orig_status = profile->status();
          if (orig_status == ConfigurationProfile::Status::
                                 ConfigurationProfile_Status_RESETING) {
            // Abort
            AINFO << "canceling configuration download job";
            const std::string cache_key =
                std::to_string(profile->vehicle_id()) + "." +
                std::to_string(profile->id());
            auto reqit = cached_request_.find(cache_key);
            if (reqit != cached_request_.end()) {
              reqit->second->Abort();
              AINFO << "configuration download job canceled";
              auto new_status = GetConfigurationStatus(profile);
              UpdateProfileStatus(profile, new_status);
            } else {
              // Nothing found, ignore
              AWARN << "profile:" << profile_id << " no cached request";
            }
          } else {
            // Not reseting, do nothing
            AWARN << "profile:" << profile_id << " not reseting";
          }
        } catch (const std::exception &e) {
          AERROR << "CancelVehicleProfileReset process failed: " << e.what();
        }
      });
}

void Configurator::UpdateAccountInfo(const std::shared_ptr<AccountInfo> &msg) {
  // Fill account info struct
  account_info_.clear_id();
  account_info_.set_id(msg->id());
  account_info_.clear_username();
  account_info_.set_username(msg->username());
  account_info_.clear_vehicles();
  for (int i = 0; i < msg->vehicles_size(); ++i) {
    auto &item = msg->vehicles(i);
    auto vehicle = account_info_.add_vehicles();
    vehicle->set_id(item.id());
    vehicle->set_vtype(item.vtype());
    vehicle->set_vin(item.vin());
  }
  account_info_.clear_profiles();
  for (int i = 0; i < msg->profiles_size(); ++i) {
    auto &item = msg->profiles(i);
    auto profile = account_info_.add_profiles();
    profile->set_id(item.id());
    profile->set_name(item.name());
    profile->set_vehicle_id(item.vehicle_id());
    profile->set_vehicle_type(item.vehicle_type());
    profile->set_vehicle_in(item.vehicle_in());
    if (item.has_revision_id()) {
      profile->set_revision_id(item.revision_id());
    }
    if (item.has_revision_digest()) {
      profile->set_revision_digest(item.revision_digest());
    }
  }
}

void Configurator::ReloadConfigProfileStatus() {
  LoadVehicleSn();
  config_profile_status_.clear_profiles();
  for (int i = 0; i < account_info_.profiles_size(); ++i) {
    auto &item = account_info_.profiles(i);
    auto profile = config_profile_status_.add_profiles();
    profile->set_id(item.id());
    profile->set_name(item.name());
    profile->set_vehicle_id(item.vehicle_id());
    profile->set_vehicle_type(item.vehicle_type());
    profile->set_vehicle_in(item.vehicle_in());
    if (item.has_vehicle_id()) {
      profile->set_revision_id(item.revision_id());
    }
    if (item.has_revision_digest()) {
      profile->set_revision_digest(item.revision_digest());
    }
    profile->set_is_local(false);
    profile->set_is_synced(false);
    profile->set_status(GetConfigurationStatus(profile));
  }
}

void Configurator::Start() {
  LoadVehicleSn();
  account_info_reader_ = node_->CreateReader<AccountInfo>(
      FLAGS_dreamview_account_info_topic,
      [this](const std::shared_ptr<AccountInfo> &msg) {
        UpdateAccountInfo(msg);
        ReloadConfigProfileStatus();
        BroadcastConfigurationProfileStatus();
      });
}

void Configurator::Stop() {}

} // namespace dreamview
} // namespace apollo
