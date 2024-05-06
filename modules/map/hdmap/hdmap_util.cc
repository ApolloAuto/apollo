/* Copyright 2017 The Apollo Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
=========================================================================*/
#include "modules/map/hdmap/hdmap_util.h"

#include <string>
#include <vector>

#include "absl/strings/str_split.h"
#include "cyber/common/file.h"

namespace apollo {
namespace hdmap {

using apollo::relative_map::MapMsg;

namespace {

// Find the first existing file from a list of candidates: "file_a|file_b|...".
std::string FindFirstExist(const std::string& dir, const std::string& files) {
  const std::vector<std::string> candidates = absl::StrSplit(files, '|');
  for (const auto& filename : candidates) {
    const std::string file_path = absl::StrCat(FLAGS_map_dir, "/", filename);
    if (cyber::common::PathExists(file_path)) {
      return file_path;
    }
  }
  AERROR << "No existing file found in " << dir << "/" << files
         << ". Fallback to first candidate as default result.";
  ACHECK(!candidates.empty()) << "Please specify at least one map.";
  return absl::StrCat(FLAGS_map_dir, "/", candidates[0]);
}

}  // namespace

std::string BaseMapFile() {
  if (FLAGS_use_navigation_mode) {
    AWARN << "base_map file is not used when FLAGS_use_navigation_mode is true";
  }
  return FLAGS_test_base_map_filename.empty()
             ? FindFirstExist(FLAGS_map_dir, FLAGS_base_map_filename)
             : FindFirstExist(FLAGS_map_dir, FLAGS_test_base_map_filename);
}

std::string SimMapFile() {
  if (FLAGS_use_navigation_mode) {
    AWARN << "sim_map file is not used when FLAGS_use_navigation_mode is true";
  }
  return FindFirstExist(FLAGS_map_dir, FLAGS_sim_map_filename);
}

std::string RoutingMapFile() {
  if (FLAGS_use_navigation_mode) {
    AWARN << "routing_map file is not used when FLAGS_use_navigation_mode is "
             "true";
  }
  return FindFirstExist(FLAGS_map_dir, FLAGS_routing_map_filename);
}

std::unique_ptr<HDMap> CreateMap(const std::string& map_file_path) {
  std::unique_ptr<HDMap> hdmap(new HDMap());
  if (hdmap->LoadMapFromFile(map_file_path) != 0) {
    AERROR << "Failed to load HDMap " << map_file_path;
    return nullptr;
  }
  AINFO << "Load HDMap success: " << map_file_path;
  return hdmap;
}

std::unique_ptr<HDMap> CreateMap(const MapMsg& map_msg) {
  std::unique_ptr<HDMap> hdmap(new HDMap());
  if (hdmap->LoadMapFromProto(map_msg.hdmap()) != 0) {
    AERROR << "Failed to load RelativeMap: "
           << map_msg.header().ShortDebugString();
    return nullptr;
  }
  return hdmap;
}

std::unique_ptr<HDMap> HDMapUtil::base_map_ = nullptr;
uint64_t HDMapUtil::base_map_seq_ = 0;
std::mutex HDMapUtil::base_map_mutex_;

std::unique_ptr<HDMap> HDMapUtil::sim_map_ = nullptr;
std::mutex HDMapUtil::sim_map_mutex_;

const HDMap* HDMapUtil::BaseMapPtr(const MapMsg& map_msg) {
  std::lock_guard<std::mutex> lock(base_map_mutex_);
  if (base_map_ != nullptr &&
      base_map_seq_ == map_msg.header().sequence_num()) {
    // avoid re-create map in the same cycle.
    return base_map_.get();
  } else {
    base_map_ = CreateMap(map_msg);
    base_map_seq_ = map_msg.header().sequence_num();
  }
  return base_map_.get();
}

const HDMap* HDMapUtil::BaseMapPtr() {
  // TODO(all) Those logics should be removed to planning
  /*if (FLAGS_use_navigation_mode) {
    std::lock_guard<std::mutex> lock(base_map_mutex_);
    auto* relative_map = AdapterManager::GetRelativeMap();
    if (!relative_map) {
      AERROR << "RelativeMap adapter is not registered";
      return nullptr;
    }
    if (relative_map->Empty()) {
      AERROR << "RelativeMap is empty";
      return nullptr;
    }
    const auto& latest = relative_map->GetLatestObserved();
    if (base_map_ != nullptr &&
        base_map_seq_ == latest.header().sequence_num()) {
      // avoid re-create map in the same cycle.
      return base_map_.get();
    } else {
      base_map_ = CreateMap(latest);
      base_map_seq_ = latest.header().sequence_num();
    }
  } else*/
  if (base_map_ == nullptr) {
    std::lock_guard<std::mutex> lock(base_map_mutex_);
    if (base_map_ == nullptr) {  // Double check.
      base_map_ = CreateMap(BaseMapFile());
    }
  }
  return base_map_.get();
}

const HDMap& HDMapUtil::BaseMap() { return *CHECK_NOTNULL(BaseMapPtr()); }

const HDMap* HDMapUtil::SimMapPtr() {
  if (FLAGS_use_navigation_mode) {
    return BaseMapPtr();
  } else if (sim_map_ == nullptr) {
    std::lock_guard<std::mutex> lock(sim_map_mutex_);
    if (sim_map_ == nullptr) {  // Double check.
      sim_map_ = CreateMap(SimMapFile());
    }
  }
  return sim_map_.get();
}

const HDMap& HDMapUtil::SimMap() { return *CHECK_NOTNULL(SimMapPtr()); }

bool HDMapUtil::ReloadMaps() {
  {
    std::lock_guard<std::mutex> lock(base_map_mutex_);
    base_map_ = CreateMap(BaseMapFile());
  }
  {
    std::lock_guard<std::mutex> lock(sim_map_mutex_);
    sim_map_ = CreateMap(SimMapFile());
  }
  return base_map_ != nullptr && sim_map_ != nullptr;
}

bool HDMapUtil::ReloadBaseMap() {
  {
    std::lock_guard<std::mutex> lock(base_map_mutex_);
    base_map_ = CreateMap(BaseMapFile());
  }
  return base_map_ != nullptr;
}

}  // namespace hdmap
}  // namespace apollo
