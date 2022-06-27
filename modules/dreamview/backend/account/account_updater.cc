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

#include "modules/dreamview/backend/account/account_updater.h"

#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>

#include <boost/beast/core/detail/base64.hpp>
#include <curl/curl.h>

#include "cyber/common/environment.h"
#include "cyber/common/file.h"
#include "modules/common/util/json_util.h"
#include "modules/dreamview/backend/account/account_gflags.h"
#include "modules/dreamview/backend/common/curl_request.h"

namespace apollo {
namespace dreamview {

using apollo::common::util::JsonUtil;
using Json = nlohmann::json;

AccountUpdater::AccountUpdater(WebSocketHandler *websocket,
                               LoginCallbackHandler *login_callback_handler)
    : node_(cyber::CreateNode("account")), websocket_(websocket),
      login_callback_handler_(login_callback_handler) {
  RegisterMessageHandlers();
}

AccountUpdater::~AccountUpdater() { Stop(); }

std::string AccountUpdater::GetCacheDir() {
  const std::string cache_dir =
      cyber::common::GetEnv("APOLLO_CACHE_DIR", "/apollo/.cache");
  return cache_dir;
}

std::string AccountUpdater::GetSessionFilePath() {
  const std::string cache_dir = GetCacheDir();
  const std::string filepath = cache_dir + "/session";
  return filepath;
}

std::string AccountUpdater::GetCookiesFilePath() {
  const std::string cache_dir = GetCacheDir();
  const std::string filepath = cache_dir + "/cookies";
  return filepath;
}

void AccountUpdater::LoadSession() {
  const std::string filepath = GetSessionFilePath();
  std::ifstream fin(filepath);
  if (fin.is_open()) {
    fin >> sid_;
    fin.close();
  }
}

void AccountUpdater::SaveSession() {
  const std::string filepath = GetSessionFilePath();
  cyber::common::EnsureDirectory(GetCacheDir());
  std::ofstream fout(filepath);
  if (fout.is_open()) {
    fout << sid_;
    fout.close();
  }
}

void AccountUpdater::DeleteCookies() {
  const std::string filepath = GetCookiesFilePath();
  cyber::common::EnsureDirectory(GetCacheDir());
  std::ofstream fout(filepath);
  if (fout.is_open()) {
    fout << "";
    fout.close();
  }
}

void AccountUpdater::FetchAccountInfo(const std::string sid) {
  std::string url =
      FLAGS_dreamview_account_server_api_entry + "/user/api/account";
  AINFO << "Fetching account info from " << url;
  cyber::common::EnsureDirectory(GetCacheDir());
  CurlRequest req(url);
  std::map<std::string, std::string> headers;
  headers["Accept"] = "*/*";
  headers["Host"] = "studio.apollo.auto";
  headers["User-Agent"] = "Apollo Dreamview";
  req.AddHeaders(headers);
  req.SetCookieFile(GetCookiesFilePath());
  req.SetCookieJar(GetCookiesFilePath());

  char bduss_buffer[512];
  std::pair<std::size_t, std::size_t> wrc =
      boost::beast::detail::base64::decode(bduss_buffer, sid.c_str(),
                                           sid.size());
  const std::string bduss(bduss_buffer, wrc.first);
  req.SetEncodedCookie("BDUSS", bduss, "/", ".apollo.auto", 31536000);
  req.SetSslVerifyHost(FLAGS_dreamview_curl_ssl_verify_host);
  req.SetSslVerifyPeer(FLAGS_dreamview_curl_ssl_verify_peer);
  req.SetVerbose(FLAGS_dreamview_curl_verbose);
  if (!req.perform()) {
    AERROR << "Failed to fetch account info";
  } else {
    try {
      Json json = Json::parse(req.Response());
      // Fill account info struct
      account_info_.clear_id();
      account_info_.set_id(json["data"]["id"].get<uint64_t>());
      account_info_.clear_username();
      account_info_.set_username(json["data"]["username"].get<std::string>());
      account_info_.clear_vehicles();
      for (auto &item :
           json["data"]["vehicles"].get<std::vector<Json::object_t>>()) {
        auto vehicle = account_info_.add_vehicles();
        vehicle->clear_id();
        vehicle->set_id(item["id"].get<uint64_t>());
        vehicle->clear_vtype();
        vehicle->set_vtype(item["vtype"].get<uint64_t>());
        vehicle->clear_vin();
        vehicle->set_vin(item["vin"].get<std::string>());
      }
      account_info_.clear_profiles();
      if (json["data"].find("profiles") != json["data"].end()) {
        for (auto &item :
             json["data"]["profiles"].get<std::vector<Json::object_t>>()) {
          auto profile = account_info_.add_profiles();
          profile->clear_id();
          profile->set_id(item["id"].get<uint64_t>());
          profile->clear_name();
          profile->set_name(item["name"].get<std::string>());
          profile->clear_vehicle_id();
          profile->set_vehicle_id(item["vehicle_id"].get<uint64_t>());
          profile->set_vehicle_type(item["vtype"].get<uint32_t>());
          profile->set_vehicle_in(item["vin"].get<std::string>());
          if (item.find("revision_id") != item.end()) {
            profile->set_revision_id(item["revision_id"].get<uint64_t>());
          }
          if (item.find("revision_digest") != item.end()) {
            profile->set_revision_digest(
                item["revision_digest"].get<std::string>());
          }
        }
      }
      AINFO << "Fetch account info success, user: " << account_info_.username();
      // Publich account info
      account_info_writer_->Write(account_info_);
    } catch (const std::exception &e) {
      AERROR << "Failed to parse account info result: " << e.what();
    }
  }
}

void AccountUpdater::UpdateInfoBySid(const std::string sid) {
  FetchAccountInfo(sid);
  // If sid valid, Save it;
  sid_ = sid;
  SaveSession();
}

void AccountUpdater::RegisterMessageHandlers() {
  login_callback_handler_->RegisterLoginSuccessHandler(
      [this](const std::string sid) {
        UpdateInfoBySid(sid);
        // If login success, broadcast to all clients
        websocket_->BroadcastData(
            JsonUtil::ProtoToTypedJson("AccountInfo", account_info_).dump());
      });
  // Send account info to the new client.
  websocket_->RegisterConnectionReadyHandler(
      [this](WebSocketHandler::Connection *conn) {
        // Reload account info every time refresh page
        FetchAccountInfo(sid_);
        websocket_->SendData(
            conn,
            JsonUtil::ProtoToTypedJson("AccountInfo", account_info_).dump());
      });
  // Refresh account info
  websocket_->RegisterMessageHandler(
      "AccountInfo",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        FetchAccountInfo(sid_);
        websocket_->BroadcastData(
            JsonUtil::ProtoToTypedJson("AccountInfo", account_info_).dump());
      });
  // Logout, destory session
  websocket_->RegisterMessageHandler(
      "AccountLogout",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        account_info_.clear_id();
        account_info_.clear_username();
        account_info_.clear_vehicles();
        account_info_.clear_profiles();
        sid_ = "";
        SaveSession();
        DeleteCookies();
        websocket_->BroadcastData(
            JsonUtil::ProtoToTypedJson("AccountInfo", account_info_).dump());
      });
}

void AccountUpdater::Start() {
  account_info_writer_ =
      node_->CreateWriter<AccountInfo>(FLAGS_dreamview_account_info_topic);
  LoadSession();
  FetchAccountInfo(sid_);
  websocket_->BroadcastData(
      JsonUtil::ProtoToTypedJson("AccountInfo", account_info_).dump());
}

void AccountUpdater::Stop() {}

} // namespace dreamview
} // namespace apollo
