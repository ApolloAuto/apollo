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

#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "modules/map/proto/map_clear_area.pb.h"
#include "modules/map/proto/map_crosswalk.pb.h"
#include "modules/map/proto/map_id.pb.h"
#include "modules/map/proto/map_junction.pb.h"
#include "modules/map/proto/map_lane.pb.h"
#include "modules/map/proto/map_overlap.pb.h"
#include "modules/map/proto/map_parking_space.pb.h"
#include "modules/map/proto/map_pnc_junction.pb.h"
#include "modules/map/proto/map_road.pb.h"
#include "modules/map/proto/map_rsu.pb.h"
#include "modules/map/proto/map_signal.pb.h"
#include "modules/map/proto/map_speed_bump.pb.h"
#include "modules/map/proto/map_stop_sign.pb.h"
#include "modules/map/proto/map_yield_sign.pb.h"

#include "modules/common/math/aabox2d.h"
#include "modules/common/math/aaboxkdtree2d.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/math/polygon2d.h"
#include "modules/common/math/vec2d.h"

/**
 * @namespace apollo::hdmap
 * @brief apollo::hdmap
 */
namespace apollo {
namespace hdmap {

template <class Object, class GeoObject>
class ObjectWithAABox {
 public:
  ObjectWithAABox(const apollo::common::math::AABox2d &aabox,
                  const Object *object, const GeoObject *geo_object,
                  const int id)
      : aabox_(aabox), object_(object), geo_object_(geo_object), id_(id) {}
  ~ObjectWithAABox() {}
  const apollo::common::math::AABox2d &aabox() const { return aabox_; }
  double DistanceTo(const apollo::common::math::Vec2d &point) const {
    return geo_object_->DistanceTo(point);
  }
  double DistanceSquareTo(const apollo::common::math::Vec2d &point) const {
    return geo_object_->DistanceSquareTo(point);
  }
  const Object *object() const { return object_; }
  const GeoObject *geo_object() const { return geo_object_; }
  int id() const { return id_; }

 private:
  apollo::common::math::AABox2d aabox_;
  const Object *object_;
  const GeoObject *geo_object_;
  int id_;
};

struct LineBoundary {
  std::vector<apollo::common::PointENU> line_points;
};
struct PolygonBoundary {
  std::vector<apollo::common::PointENU> polygon_points;
};

enum class PolygonType {
  JUNCTION_POLYGON = 0,
  PARKINGSPACE_POLYGON = 1,
  ROAD_HOLE_POLYGON = 2,
};

struct RoiAttribute {
  PolygonType type;
  Id id;
};

struct PolygonRoi {
  apollo::common::math::Polygon2d polygon;
  RoiAttribute attribute;
};

struct RoadRoi {
  Id id;
  LineBoundary left_boundary;
  LineBoundary right_boundary;
  std::vector<PolygonBoundary> holes_boundary;
};

template <typename ProtoBufferObject>
class BaseInfo {
 public:
  using InnerObject = ProtoBufferObject;

  explicit BaseInfo(const InnerObject &inner_object)
      : inner_object_(inner_object) {}

  const Id &id() const noexcept { return inner_object_.id(); }
  const InnerObject &inner_object() const noexcept { return inner_object_; }

 protected:
  const InnerObject &inner_object_;
};

apollo::common::math::Polygon2d ConvertToPolygon2d(const Polygon &polygon);

template <typename InnerObject>
class PolygonBasedInfo : public BaseInfo<InnerObject> {
 public:
  using Parent = BaseInfo<InnerObject>;

  explicit PolygonBasedInfo(const InnerObject &inner_object)
      : Parent(inner_object),
        polygon_(ConvertToPolygon2d(inner_object_.polygon())) {
    CHECK_GT(polygon_.num_points(), 2);
  }

  const apollo::common::math::Polygon2d &polygon() const { return polygon_; }

 protected:
  using Parent::inner_object_;
  apollo::common::math::Polygon2d polygon_;
};

void SegmentsFromCurve(
    const Curve &curve,
    std::vector<apollo::common::math::LineSegment2d> *segments);

template <typename InnerObject>
class SegmentBasedInfo : public BaseInfo<InnerObject> {
 public:
  using Parent = BaseInfo<InnerObject>;
  using SegmentVector = std::vector<apollo::common::math::LineSegment2d>;

  explicit SegmentBasedInfo(const InnerObject &inner_object)
      : Parent(inner_object) {
    for (const auto &stop_line : inner_object_.stop_line()) {
      SegmentsFromCurve(stop_line, &segments_);
    }

    ACHECK(!segments_.empty());
  }

  explicit SegmentBasedInfo(const InnerObject &inner_object, bool)
      : Parent(inner_object) {
    for (const auto &stop_line : inner_object_.position()) {
      SegmentsFromCurve(stop_line, &segments_);
    }

    ACHECK(!segments_.empty());
  }

  const SegmentVector &segments() const { return segments_; }

 protected:
  using Parent::inner_object_;
  SegmentVector segments_;
};

template <typename ItemInfo>
class OverlapBased : public ItemInfo {
 public:
  explicit OverlapBased(const typename ItemInfo::InnerObject &inner_object)
      : ItemInfo(inner_object) {
    overlap_ids_.reserve(inner_object_.overlap_id().size());
    for (const auto &overlap_id : inner_object_.overlap_id()) {
      overlap_ids_.emplace_back(overlap_id);
    }
  }

 protected:
  using ItemInfo::inner_object_;
  std::vector<Id> overlap_ids_;
};

using ClearAreaInfo = PolygonBasedInfo<ClearArea>;
using CrosswalkInfo = PolygonBasedInfo<Crosswalk>;
using ParkingSpaceInfo = PolygonBasedInfo<ParkingSpace>;
using PNCJunctionInfo = OverlapBased<PolygonBasedInfo<PNCJunction>>;
using RSUInfo = BaseInfo<RSU>;
using YieldSignInfo = SegmentBasedInfo<YieldSign>;

class LaneInfo;
class JunctionInfo;
class SignalInfo;
class StopSignInfo;
class SpeedBumpInfo;
class OverlapInfo;
class RoadInfo;
class HDMapImpl;

using LaneSegmentBox =
    ObjectWithAABox<LaneInfo, apollo::common::math::LineSegment2d>;
using LaneSegmentKDTree = apollo::common::math::AABoxKDTree2d<LaneSegmentBox>;
using OverlapInfoConstPtr = std::shared_ptr<const OverlapInfo>;
using LaneInfoConstPtr = std::shared_ptr<const LaneInfo>;
using JunctionInfoConstPtr = std::shared_ptr<const JunctionInfo>;
using SignalInfoConstPtr = std::shared_ptr<const SignalInfo>;
using CrosswalkInfoConstPtr = std::shared_ptr<const CrosswalkInfo>;
using StopSignInfoConstPtr = std::shared_ptr<const StopSignInfo>;
using YieldSignInfoConstPtr = std::shared_ptr<const YieldSignInfo>;
using ClearAreaInfoConstPtr = std::shared_ptr<const ClearAreaInfo>;
using SpeedBumpInfoConstPtr = std::shared_ptr<const SpeedBumpInfo>;
using RoadInfoConstPtr = std::shared_ptr<const RoadInfo>;
using ParkingSpaceInfoConstPtr = std::shared_ptr<const ParkingSpaceInfo>;
using RoadROIBoundaryPtr = std::shared_ptr<RoadROIBoundary>;
using PolygonRoiPtr = std::shared_ptr<PolygonRoi>;
using RoadRoiPtr = std::shared_ptr<RoadRoi>;
using PNCJunctionInfoConstPtr = std::shared_ptr<const PNCJunctionInfo>;
using RSUInfoConstPtr = std::shared_ptr<const RSUInfo>;

struct JunctionBoundary {
  JunctionInfoConstPtr junction_info;
};

using JunctionBoundaryPtr = std::shared_ptr<JunctionBoundary>;

class LaneInfo : public BaseInfo<Lane> {
 public:
  explicit LaneInfo(const Lane &lane);

  const Id &road_id() const { return road_id_; }
  void set_road_id(const Id &road_id) { road_id_ = road_id; }

  const Id &section_id() const { return section_id_; }
  void set_section_id(const Id &section_id) { section_id_ = section_id; }

  const std::vector<apollo::common::math::Vec2d> &points() const {
    return points_;
  }
  const std::vector<apollo::common::math::Vec2d> &unit_directions() const {
    return unit_directions_;
  }
  double Heading(const double s) const;
  double Curvature(const double s) const;
  const std::vector<double> &headings() const { return headings_; }
  const std::vector<apollo::common::math::LineSegment2d> &segments() const {
    return segments_;
  }
  const std::vector<double> &accumulate_s() const { return accumulated_s_; }
  const std::vector<OverlapInfoConstPtr> &overlaps() const { return overlaps_; }
  const std::vector<OverlapInfoConstPtr> &cross_lanes() const {
    return cross_lanes_;
  }
  const std::vector<OverlapInfoConstPtr> &signals() const { return signals_; }
  const std::vector<OverlapInfoConstPtr> &yield_signs() const {
    return yield_signs_;
  }
  const std::vector<OverlapInfoConstPtr> &stop_signs() const {
    return stop_signs_;
  }
  const std::vector<OverlapInfoConstPtr> &crosswalks() const {
    return crosswalks_;
  }
  const std::vector<OverlapInfoConstPtr> &junctions() const {
    return junctions_;
  }
  const std::vector<OverlapInfoConstPtr> &clear_areas() const {
    return clear_areas_;
  }
  const std::vector<OverlapInfoConstPtr> &speed_bumps() const {
    return speed_bumps_;
  }
  const std::vector<OverlapInfoConstPtr> &parking_spaces() const {
    return parking_spaces_;
  }
  const std::vector<OverlapInfoConstPtr> &pnc_junctions() const {
    return pnc_junctions_;
  }
  double total_length() const { return total_length_; }
  using SampledWidth = std::pair<double, double>;
  const std::vector<SampledWidth> &sampled_left_width() const {
    return sampled_left_width_;
  }
  const std::vector<SampledWidth> &sampled_right_width() const {
    return sampled_right_width_;
  }
  void GetWidth(const double s, double *left_width, double *right_width) const;
  double GetWidth(const double s) const;
  double GetEffectiveWidth(const double s) const;

  const std::vector<SampledWidth> &sampled_left_road_width() const {
    return sampled_left_road_width_;
  }
  const std::vector<SampledWidth> &sampled_right_road_width() const {
    return sampled_right_road_width_;
  }
  void GetRoadWidth(const double s, double *left_width,
                    double *right_width) const;
  double GetRoadWidth(const double s) const;

  bool IsOnLane(const apollo::common::math::Vec2d &point) const;
  bool IsOnLane(const apollo::common::math::Box2d &box) const;

  apollo::common::PointENU GetSmoothPoint(double s) const;
  double DistanceTo(const apollo::common::math::Vec2d &point) const;
  double DistanceTo(const apollo::common::math::Vec2d &point,
                    apollo::common::math::Vec2d *map_point, double *s_offset,
                    int *s_offset_index) const;
  apollo::common::PointENU GetNearestPoint(
      const apollo::common::math::Vec2d &point, double *distance) const;
  bool GetProjection(const apollo::common::math::Vec2d &point,
                     double *accumulate_s, double *lateral) const;

  void PostProcess(const HDMapImpl &map_instance);

 private:
  friend class HDMapImpl;
  friend class RoadInfo;
  void Init();
  void UpdateOverlaps(const HDMapImpl &map_instance);
  double GetWidthFromSample(const std::vector<LaneInfo::SampledWidth> &samples,
                            const double s) const;
  void CreateKDTree();

 private:
  std::vector<apollo::common::math::Vec2d> points_;
  std::vector<apollo::common::math::Vec2d> unit_directions_;
  std::vector<double> headings_;
  std::vector<apollo::common::math::LineSegment2d> segments_;
  std::vector<double> accumulated_s_;
  std::vector<std::string> overlap_ids_;
  std::vector<OverlapInfoConstPtr> overlaps_;
  std::vector<OverlapInfoConstPtr> cross_lanes_;
  std::vector<OverlapInfoConstPtr> signals_;
  std::vector<OverlapInfoConstPtr> yield_signs_;
  std::vector<OverlapInfoConstPtr> stop_signs_;
  std::vector<OverlapInfoConstPtr> crosswalks_;
  std::vector<OverlapInfoConstPtr> junctions_;
  std::vector<OverlapInfoConstPtr> clear_areas_;
  std::vector<OverlapInfoConstPtr> speed_bumps_;
  std::vector<OverlapInfoConstPtr> parking_spaces_;
  std::vector<OverlapInfoConstPtr> pnc_junctions_;
  std::vector<SampledWidth> sampled_left_width_;
  std::vector<SampledWidth> sampled_right_width_;
  std::vector<SampledWidth> sampled_left_road_width_;
  std::vector<SampledWidth> sampled_right_road_width_;
  std::vector<LaneSegmentBox> segment_box_list_;
  std::unique_ptr<LaneSegmentKDTree> lane_segment_kdtree_;
  Id road_id_;
  Id section_id_;
  double total_length_ = 0.0;
};

class JunctionInfo : public OverlapBased<PolygonBasedInfo<Junction>> {
 public:
  using Parent = OverlapBased<PolygonBasedInfo<Junction>>;
  using Parent::Parent;

  const std::vector<Id> &OverlapStopSignIds() const {
    return overlap_stop_sign_ids_;
  }

  void PostProcess(const HDMapImpl &map_instance);

 private:
  friend class HDMapImpl;
  void UpdateOverlaps(const HDMapImpl &map_instance);

  std::vector<Id> overlap_stop_sign_ids_;
};

class SignalInfo : public SegmentBasedInfo<Signal> {
 public:
  explicit SignalInfo(const Signal &signal);

 private:
  void Init();
};

class StopSignInfo : public OverlapBased<SegmentBasedInfo<StopSign>> {
 public:
  using Parent = OverlapBased<SegmentBasedInfo<StopSign>>;
  using Parent::Parent;

  const std::vector<Id> &OverlapLaneIds() const { return overlap_lane_ids_; }
  const std::vector<Id> &OverlapJunctionIds() const {
    return overlap_junction_ids_;
  }

  void PostProcess(const HDMapImpl &map_instance);

 private:
  friend class HDMapImpl;
  void UpdateOverlaps(const HDMapImpl &map_instance);

  std::vector<Id> overlap_lane_ids_;
  std::vector<Id> overlap_junction_ids_;
};

class SpeedBumpInfo : public SegmentBasedInfo<SpeedBump> {
 public:
  explicit SpeedBumpInfo(const SpeedBump &inner_object)
      : SegmentBasedInfo<SpeedBump>(inner_object, false) {}
};

class OverlapInfo : public BaseInfo<Overlap> {
 public:
  explicit OverlapInfo(const Overlap &overlap);

  const ObjectOverlapInfo *GetObjectOverlapInfo(const Id &id) const;
};

class RoadInfo : public BaseInfo<Road> {
 public:
  explicit RoadInfo(const Road &road);
  const std::vector<RoadSection> &sections() const { return sections_; }

  const Id &junction_id() const { return inner_object_.junction_id(); }
  bool has_junction_id() const { return inner_object_.has_junction_id(); }

  const std::vector<RoadBoundary> &GetBoundaries() const;

  apollo::hdmap::Road_Type type() const { return inner_object_.type(); }

 private:
  std::vector<RoadSection> sections_;
  std::vector<RoadBoundary> road_boundaries_;
};

}  // namespace hdmap
}  // namespace apollo
