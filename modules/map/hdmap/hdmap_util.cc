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

#include "modules/common/util/file.h"
#include "modules/common/util/string_tokenizer.h"

namespace apollo {
namespace hdmap {

namespace {

// Find the first existing file from a list of candidates: "file_a|file_b|...".
std::string FindFirstExist(const std::string& dir, const std::string& files) {
  const auto candidates =
      apollo::common::util::StringTokenizer::Split(files, "|");
  for (const auto& filename : candidates) {
    const std::string file_path =
        apollo::common::util::StrCat(FLAGS_map_dir, "/", filename);
    if (apollo::common::util::PathExists(file_path)) {
      return file_path;
    }
  }
  AERROR << "No existing file found in " << dir << "/" << files
         << ". "
            "Fallback to first candidate as default result.";
  CHECK(!candidates.empty()) << "Please specify at least one map.";
  return apollo::common::util::StrCat(FLAGS_map_dir, "/", candidates[0]);
}

}  // namespace

std::string BaseMapFile() {
  return FindFirstExist(FLAGS_map_dir, FLAGS_base_map_filename);
}

std::string SimMapFile() {
  return FindFirstExist(FLAGS_map_dir, FLAGS_sim_map_filename);
}

std::string RoutingMapFile() {
  return FindFirstExist(FLAGS_map_dir, FLAGS_routing_map_filename);
}

std::unique_ptr<HDMap> CreateMap(const std::string& map_file_path) {
  std::unique_ptr<HDMap> hdmap(new HDMap());
  if (hdmap->LoadMapFromFile(map_file_path) != 0) {
    AERROR << "Failed to load HDMap " << map_file_path;
    hdmap.reset(nullptr);
  } else {
    AINFO << "Load HDMap succ.: " << map_file_path;
  }
  return hdmap;
}

std::unique_ptr<HDMap> HDMapUtil::base_map_ = nullptr;
std::mutex HDMapUtil::base_map_mutex_;

std::unique_ptr<HDMap> HDMapUtil::sim_map_ = nullptr;
std::mutex HDMapUtil::sim_map_mutex_;

const HDMap* HDMapUtil::BaseMapPtr() {
  if (base_map_ == nullptr) {
    std::lock_guard<std::mutex> lock(base_map_mutex_);
    if (base_map_ == nullptr) {  // Double check.
      base_map_ = CreateMap(BaseMapFile());
    }
  }
  return base_map_.get();
}

const HDMap& HDMapUtil::BaseMap() {
  return *CHECK_NOTNULL(BaseMapPtr());
}

const HDMap* HDMapUtil::SimMapPtr() {
  if (sim_map_ == nullptr) {
    std::lock_guard<std::mutex> lock(sim_map_mutex_);
    if (sim_map_ == nullptr) {  // Double check.
      sim_map_ = CreateMap(SimMapFile());
    }
  }
  return sim_map_.get();
}

const HDMap& HDMapUtil::SimMap() {
  return *CHECK_NOTNULL(SimMapPtr());
}

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

}  // namespace hdmap
}  // namespace apollo
