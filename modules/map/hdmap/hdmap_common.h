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

#ifndef MODULES_MAP_HDMAP_HDMAP_COMMON_H
#define MODULES_MAP_HDMAP_HDMAP_COMMON_H

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "modules/common/math/aabox2d.h"
#include "modules/common/math/aaboxkdtree2d.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/math/polygon2d.h"
#include "modules/common/math/vec2d.h"
#include "modules/map/proto/map_crosswalk.pb.h"
#include "modules/map/proto/map_id.pb.h"
#include "modules/map/proto/map_junction.pb.h"
#include "modules/map/proto/map_lane.pb.h"
#include "modules/map/proto/map_overlap.pb.h"
#include "modules/map/proto/map_road.pb.h"
#include "modules/map/proto/map_signal.pb.h"
#include "modules/map/proto/map_stop_sign.pb.h"
#include "modules/map/proto/map_yield_sign.pb.h"

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
      : _aabox(aabox), _object(object), _geo_object(geo_object), _id(id) {}
  ~ObjectWithAABox() {}
  const apollo::common::math::AABox2d &aabox() const { return _aabox; }
  double DistanceTo(const apollo::common::math::Vec2d &point) const {
    return _geo_object->DistanceTo(point);
  }
  double DistanceSquareTo(const apollo::common::math::Vec2d &point) const {
    return _geo_object->DistanceSquareTo(point);
  }
  const Object *object() const { return _object; }
  const GeoObject *geo_object() const { return _geo_object; }
  int id() const { return _id; }

 private:
  apollo::common::math::AABox2d _aabox;
  const Object *_object;
  const GeoObject *_geo_object;
  int _id;
};

class LaneInfo;
class JunctionInfo;
class CrosswalkInfo;
class SignalInfo;
class StopSignInfo;
class YieldSignInfo;
class OverlapInfo;
class ClearAreaInfo;

class HDMapImpl;

using LaneSegmentBox =
    ObjectWithAABox<LaneInfo, apollo::common::math::LineSegment2d>;
using LaneSegmentKDTree = apollo::common::math::AABoxKDTree2d<LaneSegmentBox>;

typedef std::shared_ptr<const OverlapInfo> OverlapInfoConstPtr;
class LaneInfo {
 public:
  explicit LaneInfo(const Lane &lane);

  const Id &id() const { return _lane.id(); }
  const Id &road_id() const { return _road_id; }
  const Id &section_id() const { return _section_id; }
  const Lane &lane() const { return _lane; }
  const std::vector<apollo::common::math::Vec2d> &points() const {
    return _points;
  }
  const std::vector<apollo::common::math::Vec2d> &unit_directions() const {
    return _unit_directions;
  }
  double heading(const double s) const;
  const std::vector<double> &headings() const { return _headings; }
  const std::vector<apollo::common::math::LineSegment2d> &segments() const {
    return _segments;
  }
  const std::vector<double> &accumulate_s() const { return _accumulated_s; }
  const std::vector<OverlapInfoConstPtr> &overlaps() const { return _overlaps; }
  const std::vector<OverlapInfoConstPtr> &cross_lanes() const {
    return _cross_lanes;
  }
  const std::vector<OverlapInfoConstPtr> &signals() const { return _signals; }
  const std::vector<OverlapInfoConstPtr> &yield_signs() const {
    return _yield_signs;
  }
  const std::vector<OverlapInfoConstPtr> &stop_signs() const {
    return _stop_signs;
  }
  const std::vector<OverlapInfoConstPtr> &crosswalks() const {
    return _crosswalks;
  }
  const std::vector<OverlapInfoConstPtr> &junctions() const {
    return _junctions;
  }
  double total_length() const { return _total_length; }
  using SampledWidth = std::pair<double, double>;
  const std::vector<SampledWidth> &sampled_left_width() const {
    return _sampled_left_width;
  }
  const std::vector<SampledWidth> &sampled_right_width() const {
    return _sampled_right_width;
  }
  void get_width(const double s, double *left_width, double *right_width) const;
  double get_width(const double s) const;
  double get_effective_width(const double s) const;

  bool is_on_lane(const apollo::common::math::Vec2d &point) const;
  bool is_on_lane(const apollo::common::math::Box2d &box) const;

  apollo::common::PointENU get_smooth_point(double s) const;
  double distance_to(const apollo::common::math::Vec2d &point) const;
  double distance_to(const apollo::common::math::Vec2d &point,
                     apollo::common::math::Vec2d *map_point, double *s_offset,
                     int *s_offset_index) const;
  apollo::common::PointENU get_nearest_point(
      const apollo::common::math::Vec2d &point, double *distance) const;
  bool get_projection(const apollo::common::math::Vec2d &point,
                      double *accumulate_s, double *lateral) const;

 private:
  friend class HDMapImpl;
  friend class RoadInfo;
  void init();
  void post_process(const HDMapImpl &map_instance);
  void update_overlaps(const HDMapImpl &map_instance);
  double get_width_from_sample(
      const std::vector<LaneInfo::SampledWidth> &samples, const double s) const;
  void create_kdtree();
  void set_road_id(const Id &road_id) { _road_id = road_id; }
  void set_section_id(const Id &section_id) { _section_id = section_id; }

 private:
  const Lane &_lane;
  std::vector<apollo::common::math::Vec2d> _points;
  std::vector<apollo::common::math::Vec2d> _unit_directions;
  std::vector<double> _headings;
  std::vector<apollo::common::math::LineSegment2d> _segments;
  std::vector<double> _accumulated_s;
  std::vector<std::string> _overlap_ids;
  std::vector<OverlapInfoConstPtr> _overlaps;
  std::vector<OverlapInfoConstPtr> _cross_lanes;
  std::vector<OverlapInfoConstPtr> _signals;
  std::vector<OverlapInfoConstPtr> _yield_signs;
  std::vector<OverlapInfoConstPtr> _stop_signs;
  std::vector<OverlapInfoConstPtr> _crosswalks;
  std::vector<OverlapInfoConstPtr> _parking_spaces;
  std::vector<OverlapInfoConstPtr> _junctions;
  double _total_length = 0.0;
  std::vector<SampledWidth> _sampled_left_width;
  std::vector<SampledWidth> _sampled_right_width;

  std::vector<LaneSegmentBox> _segment_box_list;
  std::unique_ptr<LaneSegmentKDTree> _lane_segment_kdtree;

  Id _road_id;
  Id _section_id;
};

class JunctionInfo {
 public:
  explicit JunctionInfo(const Junction &junction);

  const Id &id() const { return _junction.id(); }
  const Junction &junction() const { return _junction; }
  const apollo::common::math::Polygon2d &polygon() const { return _polygon; }
  const apollo::common::math::AABox2d &mbr() const { return _mbr; }

 private:
  void init();

 private:
  const Junction &_junction;
  apollo::common::math::Polygon2d _polygon;
  apollo::common::math::AABox2d _mbr;
};
using JunctionPolygonBox =
    ObjectWithAABox<JunctionInfo, apollo::common::math::Polygon2d>;
using JunctionPolygonKDTree =
    apollo::common::math::AABoxKDTree2d<JunctionPolygonBox>;

class SignalInfo {
 public:
  explicit SignalInfo(const Signal &signal);

  const Id &id() const { return _signal.id(); }
  const Signal &signal() const { return _signal; }
  const std::vector<apollo::common::math::LineSegment2d> &segments() const {
    return _segments;
  }

 private:
  void init();

 private:
  const Signal &_signal;
  std::vector<apollo::common::math::LineSegment2d> _segments;
};
using SignalSegmentBox =
    ObjectWithAABox<SignalInfo, apollo::common::math::LineSegment2d>;
using SignalSegmentKDTree =
    apollo::common::math::AABoxKDTree2d<SignalSegmentBox>;

class CrosswalkInfo {
 public:
  explicit CrosswalkInfo(const Crosswalk &crosswalk);

  const Id &id() const { return _crosswalk.id(); }
  const Crosswalk &crosswalk() const { return _crosswalk; }
  const apollo::common::math::Polygon2d &polygon() const { return _polygon; }

 private:
  void init();

 private:
  const Crosswalk &_crosswalk;
  apollo::common::math::Polygon2d _polygon;
};
using CrosswalkPolygonBox =
    ObjectWithAABox<CrosswalkInfo, apollo::common::math::Polygon2d>;
using CrosswalkPolygonKDTree =
    apollo::common::math::AABoxKDTree2d<CrosswalkPolygonBox>;

class StopSignInfo {
 public:
  explicit StopSignInfo(const StopSign &stop_sign);

  const Id &id() const { return _stop_sign.id(); }
  const StopSign &stop_sign() const { return _stop_sign; }
  const std::vector<apollo::common::math::LineSegment2d> &segments() const {
    return _segments;
  }

 private:
  void init();

 private:
  const StopSign &_stop_sign;
  std::vector<apollo::common::math::LineSegment2d> _segments;
};
using StopSignSegmentBox =
    ObjectWithAABox<StopSignInfo, apollo::common::math::LineSegment2d>;
using StopSignSegmentKDTree =
    apollo::common::math::AABoxKDTree2d<StopSignSegmentBox>;

class YieldSignInfo {
 public:
  explicit YieldSignInfo(const YieldSign &yield_sign);

  const Id &id() const { return _yield_sign.id(); }
  const YieldSign &yield_sign() const { return _yield_sign; }
  const std::vector<apollo::common::math::LineSegment2d> &segments() const {
    return _segments;
  }

 private:
  void init();

 private:
  const YieldSign &_yield_sign;
  std::vector<apollo::common::math::LineSegment2d> _segments;
};
using YieldSignSegmentBox =
    ObjectWithAABox<YieldSignInfo, apollo::common::math::LineSegment2d>;
using YieldSignSegmentKDTree =
    apollo::common::math::AABoxKDTree2d<YieldSignSegmentBox>;

class OverlapInfo {
 public:
  explicit OverlapInfo(const Overlap &overlap);

  const Id &id() const { return _overlap.id(); }
  const Overlap &overlap() const { return _overlap; }
  const ObjectOverlapInfo *get_object_overlap_info(const Id &id) const;

 private:
  const Overlap &_overlap;
};

class RoadInfo {
 public:
  explicit RoadInfo(const Road &road);
  const Id &id() const { return _road.id(); }
  const Road &road() const { return _road; }
  const std::vector<RoadSection> &sections() const { return _sections; }

  const Id &junction_id() const { return _road.junction_id(); }
  bool has_junction_id() const { return _road.has_junction_id(); }

  const std::vector<RoadBoundary> &get_boundaries() const;

 private:
  Road _road;
  std::vector<RoadSection> _sections;
  std::vector<RoadBoundary> _road_boundaries;
};

typedef std::shared_ptr<const LaneInfo> LaneInfoConstPtr;
typedef std::shared_ptr<const JunctionInfo> JunctionInfoConstPtr;
typedef std::shared_ptr<const SignalInfo> SignalInfoConstPtr;
typedef std::shared_ptr<const CrosswalkInfo> CrosswalkInfoConstPtr;
typedef std::shared_ptr<const StopSignInfo> StopSignInfoConstPtr;
typedef std::shared_ptr<const YieldSignInfo> YieldSignInfoConstPtr;
typedef std::shared_ptr<const RoadInfo> RoadInfoConstPtr;

struct RoadROIBoundary {
  Id id;
  std::vector<RoadBoundary> road_boundaries;
};

using RoadROIBoundaryPtr = std::shared_ptr<RoadROIBoundary>;

struct JunctionBoundary {
  JunctionInfoConstPtr junction_info;
};

using JunctionBoundaryPtr = std::shared_ptr<JunctionBoundary>;

}  // namespace hdmap
}  // namespace apollo

#endif  // MODULES_MAP_HDMAP_HDMAP_COMMON_H
