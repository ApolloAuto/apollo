/* Copyright 2020 The Apollo Authors. All Rights Reserved.

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

#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "modules/common/math/line_segment2d.h"
#include "modules/common/util/util.h"
#include "modules/map/hdmap/hdmap_common.h"

/**
 * @namespace apollo::hdmap::objects
 * @brief apollo::hdmap::objects
 */
namespace apollo {
namespace hdmap {
namespace objects {

inline Id CreateHDMapId(const std::string& string_id) {
  Id id;
  id.set_id(string_id);
  return id;
}

template <typename Item>
class Base {
 public:
  using ItemConstPtr = std::shared_ptr<const Item>;
  using Table = std::unordered_map<std::string, std::shared_ptr<Item>>;

  ItemConstPtr GetById(const Id& id) const noexcept;

  const Table& GetTable() const noexcept { return table_; }

  template <typename Array>
  void LoadTable(const Array& array) {
    table_.reserve(array.size());
    for (const auto& item : array) {
      table_[item.id().id()].reset(new Item(item));
    }
  }

  void Clear();

 protected:
  Table table_;
};

template <typename Item, typename Box>
class SearchableBase : public Base<Item> {
 public:
  using Table = std::unordered_map<std::string, std::shared_ptr<Item>>;
  using BoxTable = std::vector<Box>;
  using Vec2d = apollo::common::math::Vec2d;
  using PointENU = apollo::common::PointENU;
  using KDTree = apollo::common::math::AABoxKDTree2d<Box>;
  using AABoxKDTreeParams = apollo::common::math::AABoxKDTreeParams;
  using Base<Item>::GetById;
  using typename Base<Item>::ItemConstPtr;

  const BoxTable& GetBoxTable() const noexcept { return box_table_; }

  int GetItems(const PointENU& point, double distance,
               std::vector<ItemConstPtr>* items) const;

  int GetItems(const Vec2d& point, double distance,
               std::vector<ItemConstPtr>* items) const;

  void Clear();

 protected:
  using Base<Item>::table_;

  int SearchObjects(const Vec2d& center, const double radius,
                    std::vector<std::string>* const results) const;
  BoxTable box_table_;
  std::unique_ptr<KDTree> kdtree_;
  mutable std::mutex kdtree_mutex_;
};

template <typename Item>
class PolygonSearchableBase
    : public SearchableBase<
          Item, ObjectWithAABox<Item, apollo::common::math::Polygon2d>> {
 public:
  using Parent =
      SearchableBase<Item,
                     ObjectWithAABox<Item, apollo::common::math::Polygon2d>>;
  using typename Parent::AABoxKDTreeParams;
  using typename Parent::ItemConstPtr;

  void BuildKDTree(const AABoxKDTreeParams& params);
  void BuildKDTree(int max_leaf_size, double max_leaf_dimension = 5);

 protected:
  using Parent::box_table_;
  using Parent::kdtree_;
  using Parent::kdtree_mutex_;
  using Parent::table_;
  using typename Parent::KDTree;
};

template <typename Item>
class SegmentSearchableBase
    : public SearchableBase<
          Item, ObjectWithAABox<Item, apollo::common::math::LineSegment2d>> {
 public:
  using Parent = SearchableBase<
      Item, ObjectWithAABox<Item, apollo::common::math::LineSegment2d>>;
  using typename Parent::AABoxKDTreeParams;
  using typename Parent::ItemConstPtr;
  using Box = ObjectWithAABox<Item, apollo::common::math::LineSegment2d>;

  void BuildKDTree(const AABoxKDTreeParams& params);
  void BuildKDTree(int max_leaf_size, double max_leaf_dimension = 5);

 protected:
  using Parent::box_table_;
  using Parent::kdtree_;
  using Parent::kdtree_mutex_;
  using Parent::table_;
  using typename Parent::KDTree;
};

template <typename Base>
class PostProcessingBased : public Base {
 public:
  using Parent = Base;
  using Parent::box_table_;
  using Parent::kdtree_;
  using Parent::kdtree_mutex_;
  using Parent::table_;
  using typename Parent::KDTree;

  void PostProcess(const HDMapImpl& map);
};

template <typename Item>
typename Base<Item>::ItemConstPtr Base<Item>::GetById(
    const Id& id) const noexcept {
  auto it = table_.find(id.id());
  return it != table_.end() ? it->second : nullptr;
}

template <typename Item>
void Base<Item>::Clear() {
  table_.clear();
}

template <typename Item, typename Box>
int SearchableBase<Item, Box>::GetItems(
    const PointENU& point, double distance,
    std::vector<ItemConstPtr>* items) const {
  return GetItems({point.x(), point.y()}, distance, items);
}

template <typename Item, typename Box>
int SearchableBase<Item, Box>::GetItems(
    const Vec2d& point, double distance,
    std::vector<ItemConstPtr>* items) const {
  if (items == nullptr || kdtree_ == nullptr) {
    return -1;
  }

  items->clear();
  std::vector<std::string> ids;
  const int status = this->SearchObjects(point, distance, &ids);
  if (status < 0) {
    return status;
  }

  items->reserve(ids.size());
  for (const auto& id : ids) {
    items->emplace_back(GetById(CreateHDMapId(id)));
  }

  return 0;
}

template <typename Item, typename Box>
int SearchableBase<Item, Box>::SearchObjects(
    const Vec2d& center, const double radius,
    std::vector<std::string>* const results) const {
  if (results == nullptr) {
    return -1;
  }

  UNIQUE_LOCK_MULTITHREAD(kdtree_mutex_);
  auto objects = kdtree_->GetObjects(center, radius);
  std::unordered_set<std::string> result_ids;
  result_ids.reserve(objects.size());
  for (const auto* object_ptr : objects) {
    result_ids.insert(object_ptr->object()->id().id());
  }

  results->assign(result_ids.begin(), result_ids.end());
  return 0;
}

template <typename Item, typename Box>
void SearchableBase<Item, Box>::Clear() {
  UNIQUE_LOCK_MULTITHREAD(kdtree_mutex_);
  Base<Item>::Clear();
  box_table_.clear();
  kdtree_.reset(nullptr);
}

template <typename Item>
void PolygonSearchableBase<Item>::BuildKDTree(const AABoxKDTreeParams& params) {
  UNIQUE_LOCK_MULTITHREAD(kdtree_mutex_);
  box_table_.clear();
  box_table_.reserve(table_.size());
  for (const auto& info_with_id : table_) {
    const auto* info = info_with_id.second.get();
    const auto& polygon = info->polygon();
    box_table_.emplace_back(polygon.AABoundingBox(), info, &polygon, 0);
  }

  kdtree_.reset(new KDTree(box_table_, params));
}

template <typename Item>
void PolygonSearchableBase<Item>::BuildKDTree(int max_leaf_size,
                                              double max_leaf_dimension) {
  AABoxKDTreeParams params{-1, max_leaf_size, max_leaf_dimension};
  BuildKDTree(params);
}

template <typename Item>
void SegmentSearchableBase<Item>::BuildKDTree(const AABoxKDTreeParams& params) {
  UNIQUE_LOCK_MULTITHREAD(kdtree_mutex_);
  box_table_.clear();
  box_table_.reserve(10 * table_.size());
  for (const auto& info_with_id : table_) {
    const auto* info = info_with_id.second.get();
    auto& segments = info->segments();
    for (size_t id = 0; id != segments.size(); ++id) {
      const auto& segment = segments[id];
      box_table_.emplace_back(
          apollo::common::math::AABox2d(segment.start(), segment.end()), info,
          &segment, id);
    }
  }

  kdtree_.reset(new KDTree(box_table_, params));
}

template <typename Item>
void SegmentSearchableBase<Item>::BuildKDTree(int max_leaf_size,
                                              double max_leaf_dimension) {
  AABoxKDTreeParams params{-1, max_leaf_size, max_leaf_dimension};
  BuildKDTree(params);
}

template <typename Base>
void PostProcessingBased<Base>::PostProcess(const HDMapImpl& hd_map) {
  for (auto& entry : table_) {
    entry.second->PostProcess(hd_map);
  }
}

using ClearAreas = PolygonSearchableBase<ClearAreaInfo>;
using Crosswalks = PolygonSearchableBase<CrosswalkInfo>;
using ParkingSpaces = PolygonSearchableBase<ParkingSpaceInfo>;
using PNCJunctions = PolygonSearchableBase<PNCJunctionInfo>;

using Signals = SegmentSearchableBase<SignalInfo>;
using SpeedBumps = SegmentSearchableBase<SpeedBumpInfo>;
using YieldSigns = SegmentSearchableBase<YieldSignInfo>;

using Junctions = PostProcessingBased<PolygonSearchableBase<JunctionInfo>>;

using Overlaps = Base<OverlapInfo>;

class Lanes : public PostProcessingBased<SegmentSearchableBase<LaneInfo>> {
 public:
  using RoadIds = std::unordered_set<std::string>;
  using PointENU = apollo::common::PointENU;

  Lanes(const Signals& signals, const Overlaps& overlaps)
      : signals_(signals), overlaps_(overlaps) {}

  int GetRoadIds(const Vec2d& point, double distance, RoadIds* road_ids) const;

  int GetNearest(const PointENU& point, LaneInfoConstPtr* nearest_lane,
                 double* nearest_s, double* nearest_l) const;

  int GetNearest(const Vec2d& point, LaneInfoConstPtr* nearest_lane,
                 double* nearest_s, double* nearest_l) const;

  int GetItemsWithHeading(const PointENU& point, const double distance,
                          const double central_heading,
                          const double max_heading_difference,
                          std::vector<LaneInfoConstPtr>* lanes) const;

  int GetItemsWithHeading(const Vec2d& point, const double distance,
                          const double central_heading,
                          const double max_heading_difference,
                          std::vector<LaneInfoConstPtr>* lanes) const;

  int GetNearestWithHeading(const Vec2d& point, const double distance,
                            const double central_heading,
                            const double max_heading_difference,
                            LaneInfoConstPtr* nearest_lane, double* nearest_s,
                            double* nearest_l) const;

  int GetForwardNearestSignalsOnLane(
      const apollo::common::PointENU& point, const double distance,
      std::vector<SignalInfoConstPtr>* signals) const;

 protected:
  const Signals& signals_;
  const Overlaps& overlaps_;
};

class StopSigns
    : public PostProcessingBased<SegmentSearchableBase<StopSignInfo>> {
 public:
  StopSigns(const Lanes& lanes, const Junctions& junctions)
      : lanes_(lanes), junctions_(junctions) {}

  int GetAssociatedSigns(const Id& id,
                         std::vector<StopSignInfoConstPtr>* stop_signs) const;

  int GetAssociatedLanes(const Id& id,
                         std::vector<LaneInfoConstPtr>* lanes) const;

 protected:
  const Lanes& lanes_;
  const Junctions& junctions_;
};

class Roads : public Base<RoadInfo> {
 public:
  using Vec2d = apollo::common::math::Vec2d;
  using PointENU = apollo::common::PointENU;
  using Ids = Lanes::RoadIds;

  Roads(Lanes* lanes, const Junctions& junctions, const Overlaps& overlaps,
        const ParkingSpaces& parking_spaces);

  int GetItems(const PointENU& point, double distance,
               std::vector<RoadInfoConstPtr>* roads) const;

  int GetItems(const Vec2d& point, double distance,
               std::vector<RoadInfoConstPtr>* roads) const;

  int GetRoadBoundaries(const PointENU& point, double radius,
                        std::vector<RoadROIBoundaryPtr>* road_boundaries,
                        std::vector<JunctionBoundaryPtr>* junctions) const;

  int GetRoadBoundaries(const PointENU& point, double radius,
                        std::vector<RoadRoiPtr>* road_boundaries,
                        std::vector<JunctionInfoConstPtr>* junctions) const;

  int GetRoi(const apollo::common::PointENU& point, double radius,
             std::vector<RoadRoiPtr>* roads_roi,
             std::vector<PolygonRoiPtr>* polygons_roi);

  void PostProcess();

 protected:
  static void GetBoundaries(std::vector<RoadRoiPtr>* roads_roi,
                            const RoadInfoConstPtr& road_ptr);

  Lanes* lanes_;
  const Junctions& junctions_;
  const Overlaps& overlaps_;
  const ParkingSpaces& parking_spaces_;
};

class RSUs : public Base<RSUInfo> {
 public:
  RSUs(const Lanes& lanes, const Junctions& junctions, const Overlaps& overlaps)
      : lanes_(lanes), junctions_(junctions), overlaps_(overlaps) {}

  int GetForwardNearestItems(const apollo::common::PointENU& point,
                             double distance, double central_heading,
                             double max_heading_difference,
                             std::vector<RSUInfoConstPtr>* rsus) const;

 protected:
  const Lanes& lanes_;
  const Junctions& junctions_;
  const Overlaps& overlaps_;
};

}  // namespace objects
}  // namespace hdmap
}  // namespace apollo
