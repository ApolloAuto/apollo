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

#ifndef MODULES_MAP_MAP_LOADER_INC_HDMAP_IMPL_H
#define MODULES_MAP_MAP_LOADER_INC_HDMAP_IMPL_H

#include <memory>
#include <vector>
#include <string>
#include <unordered_map>

#include "modules/common/math/aabox2d.h"
#include "modules/common/math/aaboxkdtree2d.h"
#include "modules/common/math/polygon2d.h"
#include "modules/common/math/line_segment2d.h"
#include "modules/common/math/vec2d_utils.h"
#include "modules/map/proto/map.pb.h"
#include "modules/map/proto/map_geometry.pb.h"
#include "modules/map/proto/map_lane.pb.h"
#include "modules/map/proto/map_junction.pb.h"
#include "modules/map/proto/map_signal.pb.h"
#include "modules/map/proto/map_crosswalk.pb.h"
#include "modules/map/proto/map_stop_sign.pb.h"
#include "modules/map/proto/map_yield_sign.pb.h"
#include "modules/map/proto/map_overlap.pb.h"

#include "modules/map/hdmap/hdmap_common.h"

namespace apollo {
namespace hdmap {

class HDMapImpl {
 public:
    using LaneTable =
        std::unordered_map<std::string, std::shared_ptr<LaneInfo>>;
    using JunctionTable =
        std::unordered_map<std::string, std::shared_ptr<JunctionInfo>>;
    using SignalTable =
        std::unordered_map<std::string, std::shared_ptr<SignalInfo>>;
    using CrosswalkTable =
        std::unordered_map<std::string, std::shared_ptr<CrosswalkInfo>>;
    using StopSignTable =
        std::unordered_map<std::string, std::shared_ptr<StopSignInfo>>;
    using YieldSignTable =
        std::unordered_map<std::string, std::shared_ptr<YieldSignInfo>>;
    using OverlapTable =
        std::unordered_map<std::string, std::shared_ptr<OverlapInfo>>;

 public:
    int load_map_from_file(const std::string& map_filename);

    LaneInfoConstPtr get_lane_by_id(const Id& id) const;
    JunctionInfoConstPtr get_junction_by_id(const Id& id) const;
    SignalInfoConstPtr get_signal_by_id(const Id& id) const;
    CrosswalkInfoConstPtr get_crosswalk_by_id(const Id& id) const;
    StopSignInfoConstPtr get_stop_sign_by_id(const Id& id) const;
    YieldSignInfoConstPtr get_yield_sign_by_id(const Id& id) const;
    OverlapInfoConstPtr get_overlap_by_id(const Id& id) const;

    int get_lanes(const apollo::hdmap::Point& point,
                double distance,
                std::vector<LaneInfoConstPtr>* lanes) const;
    int get_junctions(const apollo::hdmap::Point& point,
                    double distance,
                    std::vector<JunctionInfoConstPtr>* junctions) const;
    int get_crosswalks(const apollo::hdmap::Point& point,
                    double distance,
                    std::vector<CrosswalkInfoConstPtr>* crosswalks) const;
    int get_signals(const apollo::hdmap::Point& point,
                    double distance,
                    std::vector<SignalInfoConstPtr>* signals) const;
    int get_stop_signs(const apollo::hdmap::Point& point,
                    double distance,
                    std::vector<StopSignInfoConstPtr>* stop_signs) const;
    int get_yield_signs(const apollo::hdmap::Point& point,
                    double distance,
                    std::vector<YieldSignInfoConstPtr>* yield_signs) const;

    int get_nearest_lane(const apollo::hdmap::Point& point,
                    LaneInfoConstPtr* nearest_lane,
                    double* nearest_s,
                    double* nearest_l);

 private:
    int get_lanes(const apollo::common::Vec2D& point,
                double distance,
                std::vector<LaneInfoConstPtr>* lanes) const;
    int get_junctions(const apollo::common::Vec2D& point,
                double distance,
                std::vector<JunctionInfoConstPtr>* junctions) const;
    int get_crosswalks(const apollo::common::Vec2D& point,
                    double distance,
                    std::vector<CrosswalkInfoConstPtr>* crosswalks) const;
    int get_signals(const apollo::common::Vec2D& point,
                    double distance,
                    std::vector<SignalInfoConstPtr>* signals) const;
    int get_stop_signs(const apollo::common::Vec2D& point,
                    double distance,
                    std::vector<StopSignInfoConstPtr>* stop_signs) const;
    int get_yield_signs(const apollo::common::Vec2D& point,
                    double distance,
                    std::vector<YieldSignInfoConstPtr>* yield_signs) const;
    int get_nearest_lane(const apollo::common::Vec2D &point,
                    LaneInfoConstPtr* nearest_lane,
                    double *nearest_s,
                    double *nearest_l) const;

    template<class Table, class BoxTable, class KDTree>
    static void build_segment_kdtree(const Table& table,
            const apollo::common::math::AABoxKDTreeParams& params,
            BoxTable* const box_table,
            std::unique_ptr<KDTree>* const kdtree);

    template<class Table, class BoxTable, class KDTree>
    static void build_polygon_kdtree(const Table& table,
                const apollo::common::math::AABoxKDTreeParams& params,
                BoxTable* const box_table,
                std::unique_ptr<KDTree>* const kdtree);

    void build_lane_segment_kdtree();
    void build_junction_polygon_kdtree();
    void build_crosswalk_polygon_kdtree();
    void build_signal_segment_kdtree();
    void build_stop_sign_segment_kdtree();
    void build_yield_sign_segment_kdtree();

    template<class KDTree>
    static int search_objects(const apollo::common::Vec2D& center,
                    const double radius,
                    const KDTree& kdtree,
                    std::vector<std::string>* const results);

    void clear();

 private:
    apollo::hdmap::Map _map;

    LaneTable           _lane_table;
    JunctionTable       _junction_table;
    CrosswalkTable      _crosswalk_table;
    SignalTable         _signal_table;
    StopSignTable       _stop_sign_table;
    YieldSignTable      _yield_sign_table;
    OverlapTable        _overlap_table;

    std::vector<LaneSegmentBox> _lane_segment_boxes;
    std::unique_ptr<LaneSegmentKDTree> _lane_segment_kdtree;

    std::vector<JunctionPolygonBox> _junction_polygon_boxes;
    std::unique_ptr<JunctionPolygonKDTree> _junction_polygon_kdtree;

    std::vector<CrosswalkPolygonBox> _crosswalk_polygon_boxes;
    std::unique_ptr<CrosswalkPolygonKDTree> _crosswalk_polygon_kdtree;

    std::vector<SignalSegmentBox> _signal_segment_boxes;
    std::unique_ptr<SignalSegmentKDTree> _signal_segment_kdtree;

    std::vector<StopSignSegmentBox> _stop_sign_segment_boxes;
    std::unique_ptr<StopSignSegmentKDTree> _stop_sign_segment_kdtree;

    std::vector<YieldSignSegmentBox> _yield_sign_segment_boxes;
    std::unique_ptr<YieldSignSegmentKDTree> _yield_sign_segment_kdtree;
};

}  // namespace hdmap
}  // namespace apollo

#endif  // MODULES_MAP_MAP_LOADER_INC_HDMAP_IMPL_H
