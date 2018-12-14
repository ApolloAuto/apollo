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

/**
 * @file pc_map.h
 * @brief The class of PCMap.
 */

#ifndef MODULES_LOCALIZATION_LMD_PREDICTOR_PERCEPTION_PC_MAP_H_
#define MODULES_LOCALIZATION_LMD_PREDICTOR_PERCEPTION_PC_MAP_H_

#include <tuple>
#include <vector>

#include "gtest/gtest.h"

#include "modules/common/math/math_utils.h"
#include "modules/common/proto/geometry.pb.h"
#include "modules/perception/proto/perception_obstacle.pb.h"

#include "modules/common/status/status.h"
#include "modules/localization/lmd/predictor/perception/lm_provider.h"

/**
 * @namespace apollo::localization
 * @brief apollo::localization
 */
namespace apollo {
namespace localization {

typedef std::size_t PCMapIndex;

/**
 * @struct PCMapPoint
 * @brief  Point stored in map.
 */
struct PCMapPoint {
  PCMapIndex prev = (PCMapIndex)-1;
  PCMapIndex next = (PCMapIndex)-1;

  apollo::common::PointENU position;
  apollo::common::Point3D direction;
  double curvature;

  PCMapPoint() = default;

  explicit PCMapPoint(const OdometryLaneMarkerPoint& point) { Set(point); }

  void Set(const OdometryLaneMarkerPoint& point) {
    position = point.position();
    direction = point.direct();
    curvature = point.curvature();
  }
};

/**
 * @class PCMap
 *
 * @brief  Map of point cloud.
 */
class PCMap {
  struct Node {
    PCMapIndex next;

    PCMapIndex p_index = (PCMapIndex)-1;
    PCMapIndex c_index[4];

    int64_t cx;
    int64_t cy;
    char level;
    unsigned lt_is_point : 1;
    unsigned rt_is_point : 1;
    unsigned lb_is_point : 1;
    unsigned rb_is_point : 1;

    Node() {
      for (auto& index : c_index) index = (PCMapIndex)-1;
    }

    int64_t HalfSize() const { return 1LL << level; }

    bool OnBoundary(int64_t x, int64_t y) const {
      auto half_size = HalfSize();
      return x >= cx - half_size && x < cx + half_size && y >= cy - half_size &&
             y < cy + half_size;
    }

    int GetPos(int64_t x, int64_t y) const {
      if (x < cx) {
        if (y < cy) {
          return 2;
        } else {
          return 0;
        }
      } else {
        if (y < cy) {
          return 3;
        } else {
          return 1;
        }
      }
    }

    void SetCXY(int64_t p_cx, int64_t p_cy, int pos) {
      auto half_size = HalfSize();
      if (pos == 0) {
        cx = p_cx - half_size;
        cy = p_cy + half_size;
      } else if (pos == 1) {
        cx = p_cx + half_size;
        cy = p_cy + half_size;
      } else if (pos == 2) {
        cx = p_cx - half_size;
        cy = p_cy - half_size;
      } else {
        cx = p_cx + half_size;
        cy = p_cy - half_size;
      }
    }

    bool IsPoint(int pos) const {
      if (pos == 0) {
        return lt_is_point;
      } else if (pos == 1) {
        return rt_is_point;
      } else if (pos == 2) {
        return lb_is_point;
      } else {
        return rb_is_point;
      }
    }

    void SetPoint(int pos, PCMapIndex point_index) {
      c_index[pos] = point_index;
      if (pos == 0) {
        lt_is_point = 1;
      } else if (pos == 1) {
        rt_is_point = 1;
      } else if (pos == 2) {
        lb_is_point = 1;
      } else {
        rb_is_point = 1;
      }
    }

    void SetChildNode(int pos, PCMapIndex node_index) {
      c_index[pos] = node_index;
      if (pos == 0) {
        lt_is_point = 0;
      } else if (pos == 1) {
        rt_is_point = 0;
      } else if (pos == 2) {
        lb_is_point = 0;
      } else {
        rb_is_point = 0;
      }
    }

    void SetParentNode(PCMapIndex node_index) { p_index = node_index; }
  };

 public:
  explicit PCMap(LMProvider* provider = nullptr);

  /**
   * @brief  Update map for range.
   * @param position The position of center point.
   * @param the radius.
   * @return Status::OK() if a suitable speed-data is created; error otherwise.
   */
  apollo::common::Status UpdateRange(const apollo::common::PointENU& position,
                                     double radius);

  /**
   * @brief  Find the nearest point in lane_marker according to the given
   * position.
   * @param position The given position.
   * @param d2 Distance squqre.
   * @return The index of nearest point.
   */
  const PCMapIndex GetNearestPoint(const apollo::common::PointENU& position,
                                   double* d2) const;

  /**
   * @brief  Find the nearest point in lane_marker according to the given
   * position.
   * @param position The given position.
   * @return The index of nearest point.
   */
  const PCMapIndex GetNearestPoint(
      const apollo::common::PointENU& position) const;

  /**
   * @brief  Find the nearest point in lane_marker according to the given
   * position, with faster speed.
   * @param node_index The index of near node.
   * @param position The given position.
   * @param d2 Distance squqre.
   * @return The index of node and index of nearest point.
   */
  const std::tuple<PCMapIndex, PCMapIndex> GetNearestPointOpt(
      PCMapIndex node_index, const apollo::common::PointENU& position,
      double* d2) const;

  /**
   * @brief  Get the point from index.
   * @param index The index of point.
   * @return The point.
   */
  const PCMapPoint& Point(PCMapIndex index) const;

  /**
   * @brief insert the points in the given OdometryLaneMarker to nodes.
   * @param lane_marker The lane_marker to sample points
   */
  void LoadLaneMarker(const OdometryLaneMarker& lane_marker);

  /**
   * @brief  Prepare odometry lane markers according to the source perception
   * lane_markers and given params
   * @param source The source perception lane markers.
   * @param position The ENU position after pc registration.
   * @param heading The heading value after pc registration.
   * @param lane_length The length to sample points.
   * @param point_number The total sample point number.
   * @param The vector contains the generated odometry lane markers.
   * @return true if succeed false otherwise
   */
  bool PrepareLaneMarkers(const apollo::perception::LaneMarkers& source,
                          const apollo::common::PointENU position,
                          const double heading, const double lane_length,
                          const int point_number,
                          std::vector<OdometryLaneMarker>* generated_vector);

  /**
   * @brief  Generate odometry lane marker according to the source perception
   * lane_marker and given params
   * @param lanemarker The source perception lanemarker.
   * @param position The ENU position after pc registration.
   * @param heading The heading value after pc registration.
   * @param lane_length The length to sample points.
   * @param point_number The total sample point number.
   * @param the ptr to the generated lane marker
   * @return true if succeed false otherwise.
   */
  bool GenerateOdometryLaneMarker(
      const apollo::perception::LaneMarker& lanemarker,
      const apollo::common::PointENU position, const double heading,
      const double lane_length, const int point_number,
      OdometryLaneMarker* generated_odo) const;

  /**
   * @brief  Calculate curve value by given curve params.
   * @param  x_value: value of x.
   * @param  c0: position.
   * @param  c1: heading_angle.
   * @param  c2: curvature.
   * @param  c3: curvature_derivative.
   * @return y = c3 * x**3 + c2 * x**2 + c1 * x + c0.
   */
  double GetCurveVal(const double x_value, const double c0, const double c1,
                     const double c2, const double c3) const;
  /**
   * @brief  Calculate the first derivative value according to x_value and curve
   * analysis formula y = c3 * x**3 + c2 * x**2 + c1 * x + c0
   * @param  x_value: value of x.
   * @param  c0: position.
   * @param  c1: heading_angle.
   * @param  c2: curvature.
   * @param  c3: curvature_derivative.
   * @return the first derivative value when x equal to x_value
   */
  double GetDerivative(const double x_value, const double c0, const double c1,
                       const double c2, const double c3) const;
  /**
    * @brief  Calculate the curvity value according to x_value and curve
    analysis formula y = c3 * x**3 + c2 * x**2 + c1 * x + c0
    * @param  x_value: value of x.
    * @param  c0: position.
    * @param  c1: heading_angle.
    * @param  c2: curvature.
    * @param  c3: curvature_derivative.
    * @return K = |y''| / (1 + y'**2)**(3.0/2)
              curvity_value K according to the analysis formula with x = x_value
    */
  double GetCurvity(const double x_value, const double c0, const double c1,
                    const double c2, const double c3) const;

  PCMapIndex InsertPoint(PCMapIndex node_index, PCMapIndex point_index);
  PCMapIndex InsertPoint(PCMapIndex node_index, PCMapIndex point_index,
                         int64_t px, int64_t py);
  PCMapIndex InsertPointInNode(PCMapIndex node_index, PCMapIndex point_index,
                               int64_t px, int64_t py);

  std::tuple<PCMapIndex, PCMapIndex, double, bool> FindNearestPointInNode(
      PCMapIndex node_index, int64_t px, int64_t py, double x, double y) const;
  std::tuple<PCMapIndex, PCMapIndex, double> FindNearestPointOutNode(
      PCMapIndex node_index, int64_t px, int64_t py, double x, double y,
      double range2) const;

  int64_t GetMapX(double x) const;
  int64_t GetMapY(double y) const;

  PCMapIndex FetchPoint();
  void StorePoint(PCMapIndex index);
  PCMapIndex FetchNode();
  void StoreNode(PCMapIndex index);
  const PCMapPoint& PointRef(PCMapIndex index) const;
  PCMapPoint& PointRef(PCMapIndex index);
  const Node& NodeRef(PCMapIndex index) const;
  Node& NodeRef(PCMapIndex index);

 private:
  LMProvider* provider_;
  std::vector<PCMapPoint> points_;
  PCMapIndex free_point_head_ = (PCMapIndex)-1;
  std::vector<Node> nodes_;
  PCMapIndex free_node_head_ = (PCMapIndex)-1;

  FRIEND_TEST(PCMapTest, FetchAndStore);
  FRIEND_TEST(PCMapTest, InsertPoint);
  FRIEND_TEST(PCMapTest, FindNearestPointInNode);
};

}  // namespace localization
}  // namespace apollo

#endif  // MODULES_LOCALIZATION_LMD_PREDICTOR_PERCEPTION_PC_MAP_H_
