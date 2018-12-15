/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/localization/lmd/predictor/perception/pc_map.h"

#include <limits>
#include <utility>

#include "modules/common/log.h"
#include "modules/common/math/vec2d.h"

namespace apollo {
namespace localization {

using apollo::common::Point3D;
using apollo::common::PointENU;
using apollo::common::Status;
using apollo::common::math::RotateAxis;
using apollo::common::math::Vec2d;
using apollo::perception::LaneMarker;
using apollo::perception::LaneMarkers;

namespace {
constexpr double kMapResolution = 0.02;
constexpr char kMapSizeMaxLevel = 30;
}  // namespace

PCMap::PCMap(LMProvider* provider) {
  // initialize root node
  nodes_.resize(1);
  NodeRef(0).level = kMapSizeMaxLevel;
  NodeRef(0).cx = 0;
  NodeRef(0).cy = 0;

  // load all of data from provider
  if (provider != nullptr) {
    provider_ = provider;
    auto pack_size = provider_->GetLaneMarkerPackSize();
    for (decltype(pack_size) pack_index = 0; pack_index < pack_size;
         ++pack_index) {
      auto lane_marker_size = provider_->GetLaneMarkerSize(pack_index);
      for (decltype(lane_marker_size) lane_index = 0;
           lane_index < lane_marker_size; ++lane_index) {
        auto lane_marker =
            provider_->GetLaneMarker(std::make_pair(pack_index, lane_index));
        if (lane_marker != nullptr) {
          LoadLaneMarker(*lane_marker);
        }
      }
    }
  }
}

Status PCMap::UpdateRange(const PointENU& position, double radius) {
  return Status::OK();
}

const PCMapIndex PCMap::GetNearestPoint(const PointENU& position,
                                        double* d2) const {
  PCMapIndex point_index;
  std::tie(std::ignore, point_index) = GetNearestPointOpt(0, position, d2);
  return point_index;
}

const PCMapIndex PCMap::GetNearestPoint(const PointENU& position) const {
  return GetNearestPoint(position, nullptr);
}

const std::tuple<PCMapIndex, PCMapIndex> PCMap::GetNearestPointOpt(
    PCMapIndex node_index, const apollo::common::PointENU& position,
    double* d2) const {
  CHECK(node_index != (PCMapIndex)-1);

  auto x = position.x();
  auto y = position.y();
  auto px = GetMapX(x);
  auto py = GetMapX(y);
  do {
    if (!NodeRef(node_index).OnBoundary(px, py)) {
      node_index = NodeRef(node_index).p_index;
    } else {
      break;
    }
  } while (node_index != (PCMapIndex)-1);

  if (node_index == (PCMapIndex)-1) {
    if (d2 != nullptr) {
      *d2 = std::numeric_limits<double>::max();
    }
    return std::make_tuple((PCMapIndex)-1, (PCMapIndex)-1);
  }

  PCMapIndex point_index;
  double distance2;
  std::tie(node_index, point_index, distance2, std::ignore) =
      FindNearestPointInNode(node_index, px, py, x, y);

  if (d2 != nullptr) {
    *d2 = distance2;
  }
  return std::make_tuple(node_index, point_index);
}

const PCMapPoint& PCMap::Point(PCMapIndex index) const {
  return points_[index];
}

void PCMap::LoadLaneMarker(const OdometryLaneMarker& lane_marker) {
  auto seg_point_index = (PCMapIndex)-1;
  PCMapIndex last_node_index = 0;
  for (const auto& lane_point : lane_marker.points()) {
    auto point_index = FetchPoint();
    PointRef(point_index).Set(lane_point);

    last_node_index = InsertPoint(last_node_index, point_index);
    if (last_node_index == (PCMapIndex)-1) {
      StorePoint(point_index);
      seg_point_index = (PCMapIndex)-1;
      last_node_index = 0;
      continue;
    }

    if (seg_point_index == (PCMapIndex)-1) {
      if (PointRef(point_index).next == (PCMapIndex)-1) {
        seg_point_index = point_index;
      }
    } else {
      if (PointRef(point_index).prev == (PCMapIndex)-1) {
        PointRef(point_index).prev = seg_point_index;
        PointRef(seg_point_index).next = point_index;
      }
      if (PointRef(point_index).next == (PCMapIndex)-1) {
        seg_point_index = point_index;
      } else {
        seg_point_index = (PCMapIndex)-1;
      }
    }
  }
}

bool PCMap::GenerateOdometryLaneMarker(
    const LaneMarker& lanemarker, const PointENU position, const double heading,
    const double lane_length, const int point_number,
    OdometryLaneMarker* generated_odo) const {
  for (int i = 0; i < point_number; ++i) {
    auto point = generated_odo->add_points();
    auto relative_x = lane_length / point_number * i;
    auto relative_y = GetCurveVal(
        relative_x, lanemarker.c0_position(), lanemarker.c1_heading_angle(),
        lanemarker.c2_curvature(), lanemarker.c3_curvature_derivative());
    auto curvature_value = GetCurveVal(
        relative_x, lanemarker.c0_position(), lanemarker.c1_heading_angle(),
        lanemarker.c2_curvature(), lanemarker.c3_curvature_derivative());
    point->set_curvature(curvature_value);
    double enu_x, enu_y;
    RotateAxis(-heading, relative_x, relative_y, &enu_x, &enu_y);
    point->mutable_position()->set_x(position.x() + enu_x);
    point->mutable_position()->set_y(position.y() + enu_y);
    point->mutable_position()->set_z(position.z());
    double enu_x_direct, enu_y_direct;
    RotateAxis(-heading, 0, 1, &enu_x_direct, &enu_y_direct);
    point->mutable_direct()->set_x(enu_x_direct);
    point->mutable_direct()->set_y(enu_y_direct);
    point->mutable_direct()->set_z(0.0);
  }
  return true;
}

bool PCMap::PrepareLaneMarkers(
    const apollo::perception::LaneMarkers& source,
    const apollo::common::PointENU position, const double heading,
    const double lane_length, const int point_number,
    std::vector<OdometryLaneMarker>* generated_vector) {
  if (source.has_left_lane_marker()) {
    const auto& lanemarker = source.left_lane_marker();
    OdometryLaneMarker odo_lane_marker;
    GenerateOdometryLaneMarker(lanemarker, position, heading, lane_length,
                               point_number, &odo_lane_marker);
    generated_vector->emplace_back(odo_lane_marker);
  }
  if (source.has_right_lane_marker()) {
    const auto& lanemarker = source.right_lane_marker();
    OdometryLaneMarker odo_lane_marker;
    GenerateOdometryLaneMarker(lanemarker, position, heading, lane_length,
                               point_number, &odo_lane_marker);
    generated_vector->emplace_back(odo_lane_marker);
  }

  for (int i = 0; i < source.next_left_lane_marker_size(); ++i) {
    const auto& lanemarker = source.next_left_lane_marker(i);
    OdometryLaneMarker odo_lane_marker;
    GenerateOdometryLaneMarker(lanemarker, position, heading, lane_length,
                               point_number, &odo_lane_marker);
    generated_vector->emplace_back(odo_lane_marker);
  }

  for (int i = 0; i < source.next_right_lane_marker_size(); ++i) {
    const auto& lanemarker = source.next_right_lane_marker(i);
    OdometryLaneMarker odo_lane_marker;
    GenerateOdometryLaneMarker(lanemarker, position, heading, lane_length,
                               point_number, &odo_lane_marker);
    generated_vector->emplace_back(odo_lane_marker);
  }
  return true;
}

double PCMap::GetCurveVal(const double x_value, const double c0,
                          const double c1, const double c2,
                          const double c3) const {
  return c3 * pow(x_value, 3.0) + c2 * pow(x_value, 2.0) + c1 * x_value + c0;
}

double PCMap::GetDerivative(const double x_value, const double c0,
                            const double c1, const double c2,
                            const double c3) const {
  return 3 * c3 * pow(x_value, 2.0) + 2 * c2 + c1;
}

double PCMap::GetCurvity(const double x_value, const double c0, const double c1,
                         const double c2, const double c3) const {
  double derivative = GetDerivative(x_value, c0, c1, c2, c3);
  return abs(6 * c3 * x_value + 2 * c2) /
         pow(1 + pow(derivative, 2.0), (3.0 / 2));
}

PCMapIndex PCMap::InsertPoint(PCMapIndex node_index, PCMapIndex point_index) {
  auto px = GetMapX(PointRef(point_index).position.x());
  auto py = GetMapY(PointRef(point_index).position.y());
  return InsertPoint(node_index, point_index, px, py);
}

PCMapIndex PCMap::InsertPoint(PCMapIndex node_index, PCMapIndex point_index,
                              int64_t px, int64_t py) {
  if (NodeRef(node_index).OnBoundary(px, py)) {
    return InsertPointInNode(node_index, point_index, px, py);
  } else {
    if (NodeRef(node_index).p_index != (PCMapIndex)-1) {
      return InsertPoint(NodeRef(node_index).p_index, point_index, px, py);
    } else {
      return (PCMapIndex)-1;
    }
  }
}

PCMapIndex PCMap::InsertPointInNode(PCMapIndex node_index,
                                    PCMapIndex point_index, int64_t px,
                                    int64_t py) {
  auto c_pos = NodeRef(node_index).GetPos(px, py);
  auto c_index = NodeRef(node_index).c_index[c_pos];
  if (c_index == (PCMapIndex)-1) {
    NodeRef(node_index).SetPoint(c_pos, point_index);
    return node_index;
  }

  if (NodeRef(node_index).IsPoint(c_pos)) {
    auto cur_point_index = c_index;
    auto cur_px = GetMapX(PointRef(cur_point_index).position.x());
    auto cur_py = GetMapY(PointRef(cur_point_index).position.y());

    auto m_index = FetchNode();
    auto m_pos = c_pos;
    NodeRef(m_index).SetParentNode(node_index);
    NodeRef(node_index).SetChildNode(m_pos, m_index);
    NodeRef(m_index).cx = NodeRef(node_index).cx;
    NodeRef(m_index).cy = NodeRef(node_index).cy;
    NodeRef(m_index).level = NodeRef(node_index).level;
    while (NodeRef(m_index).level) {
      NodeRef(m_index).level--;
      NodeRef(m_index).SetCXY(NodeRef(m_index).cx, NodeRef(m_index).cy, m_pos);
      auto p_pos = NodeRef(m_index).GetPos(px, py);
      auto cp_pos = NodeRef(m_index).GetPos(cur_px, cur_py);
      if (cp_pos != p_pos) {
        NodeRef(m_index).SetPoint(p_pos, point_index);
        NodeRef(m_index).SetPoint(cp_pos, cur_point_index);
        return m_index;
      }
      m_pos = p_pos;
    }

    PointRef(point_index).prev = PointRef(cur_point_index).prev;
    PointRef(point_index).next = PointRef(cur_point_index).next;
    if (PointRef(point_index).prev != (PCMapIndex)-1) {
      PointRef(PointRef(point_index).prev).next = point_index;
    }
    if (PointRef(point_index).next != (PCMapIndex)-1) {
      PointRef(PointRef(point_index).next).prev = point_index;
    }
    StorePoint(cur_point_index);
    return m_index;
  } else {
    if (NodeRef(c_index).OnBoundary(px, py)) {
      return InsertPointInNode(c_index, point_index, px, py);
    }

    auto m_index = FetchNode();
    auto m_pos = c_pos;
    NodeRef(m_index).SetParentNode(node_index);
    NodeRef(node_index).SetChildNode(m_pos, m_index);
    NodeRef(m_index).cx = NodeRef(node_index).cx;
    NodeRef(m_index).cy = NodeRef(node_index).cy;
    NodeRef(m_index).level = NodeRef(node_index).level;
    while (true) {
      NodeRef(m_index).level--;
      NodeRef(m_index).SetCXY(NodeRef(m_index).cx, NodeRef(m_index).cy, m_pos);
      auto c_pos =
          NodeRef(m_index).GetPos(NodeRef(c_index).cx, NodeRef(c_index).cy);
      auto p_pos = NodeRef(m_index).GetPos(px, py);
      if (c_pos != p_pos) {
        NodeRef(c_index).SetParentNode(m_index);
        NodeRef(m_index).SetChildNode(c_pos, c_index);
        NodeRef(m_index).SetPoint(p_pos, point_index);
        return m_index;
      }
      m_pos = p_pos;
    }
  }
}

std::tuple<PCMapIndex, PCMapIndex, double, bool> PCMap::FindNearestPointInNode(
    PCMapIndex node_index, int64_t px, int64_t py, double x, double y) const {
  auto c_pos = NodeRef(node_index).GetPos(px, py);
  auto c_index = NodeRef(node_index).c_index[c_pos];

  auto nearest_node_index = (PCMapIndex)-1;
  auto nearest_point_index = (PCMapIndex)-1;
  auto nearest_distance2 = std::numeric_limits<double>::max();
  bool finished = false;

  if (c_index != (PCMapIndex)-1) {
    if (NodeRef(node_index).IsPoint(c_pos)) {
      nearest_node_index = node_index;
      nearest_point_index = c_index;
      nearest_distance2 =
          Vec2d(PointRef(c_index).position.x(), PointRef(c_index).position.y())
              .DistanceSquareTo(Vec2d(x, y));
    } else {
      if (NodeRef(c_index).OnBoundary(px, py)) {
        std::tie(nearest_node_index, nearest_point_index, nearest_distance2,
                 finished) = FindNearestPointInNode(c_index, px, py, x, y);
      } else {
        std::tie(nearest_node_index, nearest_point_index, nearest_distance2) =
            FindNearestPointOutNode(c_index, px, py, x, y, nearest_distance2);
      }
    }
  }

  if (!finished) {
    for (auto i = 0; i < 4; ++i) {
      auto c_index = NodeRef(node_index).c_index[i];
      if (i != c_pos && c_index != (PCMapIndex)-1) {
        if (NodeRef(node_index).IsPoint(i)) {
          auto distance2 = Vec2d(PointRef(c_index).position.x(),
                                 PointRef(c_index).position.y())
                               .DistanceSquareTo(Vec2d(x, y));
          if (distance2 < nearest_distance2) {
            nearest_node_index = node_index;
            nearest_point_index = c_index;
            nearest_distance2 = distance2;
          }
        } else {
          PCMapIndex r_node_index;
          PCMapIndex r_point_index;
          double distance2;
          std::tie(r_node_index, r_point_index, distance2) =
              FindNearestPointOutNode(c_index, px, py, x, y, nearest_distance2);

          if (distance2 < nearest_distance2) {
            nearest_node_index = r_node_index;
            nearest_point_index = r_point_index;
            nearest_distance2 = distance2;
          }
        }
      }
    }

    auto half_size = NodeRef(node_index).HalfSize();
    auto l = x - (NodeRef(node_index).cx - half_size) * kMapResolution;
    auto r = (NodeRef(node_index).cx + half_size) * kMapResolution - x;
    auto t = (NodeRef(node_index).cy + half_size) * kMapResolution - y;
    auto b = y - (NodeRef(node_index).cy - half_size) * kMapResolution;
    finished = (l * l > nearest_distance2) && (r * r > nearest_distance2) &&
               (t * t > nearest_distance2) && (b * b > nearest_distance2);
  }

  return std::make_tuple(nearest_node_index, nearest_point_index,
                         nearest_distance2, finished);
}

std::tuple<PCMapIndex, PCMapIndex, double> PCMap::FindNearestPointOutNode(
    PCMapIndex node_index, int64_t px, int64_t py, double x, double y,
    double range2) const {
  auto half_size = NodeRef(node_index).HalfSize();
  auto bl = NodeRef(node_index).cx - half_size;
  auto br = NodeRef(node_index).cx + half_size;
  auto bt = NodeRef(node_index).cy + half_size;
  auto bb = NodeRef(node_index).cy - half_size;

  auto get_distance2 = [&](int64_t xi, int64_t yi) {
    auto xd = xi * kMapResolution;
    auto yd = yi * kMapResolution;
    return Vec2d(xd, yd).DistanceSquareTo(Vec2d(x, y));
  };

  bool out;
  if (px < bl) {
    if (py < bb) {
      out = get_distance2(bl, bb) > range2;
    } else if (py >= bb && py < bt) {
      auto d = (bl - px) * kMapResolution;
      out = d * d > range2;
    } else {
      out = get_distance2(bl, bt) > range2;
    }
  } else if (px >= bl && px < br) {
    if (py < bb) {
      auto d = (bb - py) * kMapResolution;
      out = d * d > range2;
    } else {
      auto d = (py - bt) * kMapResolution;
      out = d * d > range2;
    }
  } else {
    if (py < bb) {
      out = get_distance2(br, bb) > range2;
    } else if (py >= bb && py < bt) {
      auto d = (px - br) * kMapResolution;
      out = d * d > range2;
    } else {
      out = get_distance2(br, bt) > range2;
    }
  }
  if (out)
    return std::make_tuple((PCMapIndex)-1, (PCMapIndex)-1,
                           std::numeric_limits<double>::max());

  auto nearest_node_index = (PCMapIndex)-1;
  auto nearest_point_index = (PCMapIndex)-1;
  auto nearest_distance2 = std::numeric_limits<double>::max();

  for (auto i = 0; i < 4; ++i) {
    auto c_index = NodeRef(node_index).c_index[i];
    if (c_index != (PCMapIndex)-1) {
      if (NodeRef(node_index).IsPoint(i)) {
        auto distance2 = Vec2d(PointRef(c_index).position.x(),
                               PointRef(c_index).position.y())
                             .DistanceSquareTo(Vec2d(x, y));
        if (distance2 < nearest_distance2 && distance2 < range2) {
          nearest_node_index = node_index;
          nearest_point_index = c_index;
          nearest_distance2 = distance2;
        }
      } else {
        PCMapIndex r_node_index;
        PCMapIndex r_point_index;
        double distance2;
        std::tie(r_node_index, r_point_index, distance2) =
            FindNearestPointOutNode(c_index, px, py, x, y, nearest_distance2);

        if (distance2 < nearest_distance2) {
          nearest_node_index = r_node_index;
          nearest_point_index = r_point_index;
          nearest_distance2 = distance2;
        }
      }
    }
  }

  return std::make_tuple(nearest_node_index, nearest_point_index,
                         nearest_distance2);
}

int64_t PCMap::GetMapX(double x) const { return (int64_t)(x / kMapResolution); }

int64_t PCMap::GetMapY(double y) const { return (int64_t)(y / kMapResolution); }

PCMapIndex PCMap::FetchPoint() {
  if (free_point_head_ != (PCMapIndex)-1) {
    auto index = free_point_head_;
    auto& point = points_[index];
    free_point_head_ = point.next;
    point.next = (PCMapIndex)-1;
    return index;
  } else {
    points_.resize(points_.size() + 1);
    return points_.size() - 1;
  }
}

void PCMap::StorePoint(PCMapIndex index) {
  auto& point = points_[index];
  point.prev = (PCMapIndex)-1;
  point.next = free_point_head_;
  free_point_head_ = index;
}

PCMapIndex PCMap::FetchNode() {
  if (free_node_head_ != (PCMapIndex)-1) {
    auto index = free_node_head_;
    auto& node = nodes_[index];
    free_node_head_ = node.next;
    node.next = (PCMapIndex)-1;
    return index;
  } else {
    nodes_.resize(nodes_.size() + 1);
    return nodes_.size() - 1;
  }
}

void PCMap::StoreNode(PCMapIndex index) {
  auto& node = nodes_[index];
  node.next = free_node_head_;
  free_node_head_ = index;
}

const PCMapPoint& PCMap::PointRef(PCMapIndex index) const {
  return points_[index];
}

PCMapPoint& PCMap::PointRef(PCMapIndex index) { return points_[index]; }

const PCMap::Node& PCMap::NodeRef(PCMapIndex index) const {
  return nodes_[index];
}

PCMap::Node& PCMap::NodeRef(PCMapIndex index) { return nodes_[index]; }

}  // namespace localization
}  // namespace apollo
