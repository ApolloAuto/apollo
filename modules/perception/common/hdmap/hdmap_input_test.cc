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

#include "modules/perception/common/hdmap/hdmap_input.h"

#include "cyber/common/log.h"

namespace apollo {
namespace perception {
namespace map {

// using adu::hdmap::JunctionInfo;
// using adu::hdmap::JunctionInfoConstPtr;
// using adu::hdmap::RoiRoadBoundaryPtr;
class HDMapInputTest : public testing::Test {
 protected:
  virtual void SetUp() {
    //    char* cyber_path = "CYBER_PATH=";
    //    putenv(cyber_path);
    //    char* module_path = "MODULE_PATH=";
    //    putenv(module_path);
  }

 protected:
  HDMapInput hdmap_input_;
};
// TEST_F(HDMapInputTest, test_GetSignals) {
// EXPECT_TRUE(hdmap_input_.Init());
// // data reference to
// hdmap-library/src/test/get_forward_nearest_signals_on_lane_test.cpp in master
// std::string test_file_name = "/apollo/modules/perception/common/testdata/"
//     "map/hdmap/data/adu_common_ShaHeDaXueCheng-test01_1_5_0_15.bin";
// if (hdmap_input_.hdmap_->load_map_from_file(test_file_name) != 0) {
//   AERROR << "Failed to load get_nearest_lane_test hadmap file: "
//             << test_file_name;
//   EXPECT_TRUE(false);
// }
// AINFO << "In test_GetSignals,load hdmap file: " << test_file_name;
// base::PointD pointd;
// pointd.x = 438067.22;
// pointd.y = 4444193.53;
// pointd.z = 0;
// adu::common::hdmap::Point point;
// point.set_x(pointd.x);
// point.set_y(pointd.y);
// point.set_z(pointd.z);
// double distance = 100.0;
// std::vector<adu::hdmap::SignalInfoConstPtr> signals;
// ASSERT_TRUE(hdmap_input_.hdmap_!= nullptr);
// int ret = hdmap_input_.hdmap_->get_forward_nearest_signals_on_lane(point,
//                                                           distance,
//                                                           &signals);
// ASSERT_EQ(0, ret) << " get_forward_nearest_signals_on_lane return failed.";
// ASSERT_EQ(signals.size(), 2);
// EXPECT_EQ("2025", signals[0]->id().id());
// EXPECT_EQ("995", signals[1]->id().id());
// // begin to call function
// Eigen::Vector3d pointd1;
// pointd1 <<  438067.22, 4444193.53, 0;
// std::vector<adu::common::hdmap::Signal> signals1;
// ASSERT_TRUE(hdmap_input_.GetSignals(pointd1, 150, &signals1));
// ASSERT_EQ(signals1.size(), 2);
//}
// TEST_F(HDMapInputTest, test_GetNearestLaneDirection) {
// EXPECT_TRUE(hdmap_input_.Init());
// std::string test_file_name = "/apollo/modules/perception/common/testdata/"
//     "map/hdmap/data/sunnyvale_map.bin";
// if (hdmap_input_.hdmap_->load_map_from_file(test_file_name) != 0) {
//   AERROR << "Failed to load get_nearest_lane_test hadmap file:"
//             << test_file_name;
//   EXPECT_TRUE(false);
// }
// AINFO << "In test_GetNearestLaneDirection,load hdmap file: "
//          << test_file_name;
// // data reference to hdmap-library/src/test/get_nearest_lane_test.cpp in
// // master
// base::PointD pointd;
// pointd.x = 690;
// pointd.y = 225;
// pointd.z = 0;
// adu::common::hdmap::Point point;
// point.set_x(pointd.x);
// point.set_y(pointd.y);
// point.set_z(pointd.z);
// adu::hdmap::LaneInfoConstPtr lane_ptr;
// double nearest_s = 0.0;
// double nearest_l = 0.0;
// ASSERT_TRUE(hdmap_input_.hdmap_!= nullptr);
// int ret1 = hdmap_input_.hdmap_->get_nearest_lane(point,
//                                                  &lane_ptr,
//                                                  &nearest_s,
//                                                  &nearest_l);
// ASSERT_EQ(0, ret1)  << "  get_nearest_lane point return failed.";
// std::string expect_data = "131c0_1_-1";
// EXPECT_EQ(expect_data, lane_ptr->id().id()) << " return error lane id: "
//                                             <<  lane_ptr->id().id();
// Eigen::Vector3d lane_direction;
// EXPECT_TRUE(hdmap_input_.GetNearestLaneDirection(pointd, &lane_direction));
//}
// TEST_F(HDMapInputTest, test_Init) {
// EXPECT_TRUE(hdmap_input_.Init());
//}
// TEST_F(HDMapInputTest, test_Reset) {
// EXPECT_TRUE(hdmap_input_.Reset());
//}
// TEST_F(HDMapInputTest, test_GetRoiHDMapStruct) {
// EXPECT_TRUE(hdmap_input_.Init());
// // test the inner function get_road_boundaries()
// // reference to hdmap-library/src/test/hdmap_clinet_test.cpp in master
// adu::common::hdmap::Point point;
// point.set_x(454489.3);
// point.set_y(4439560.0);
// double distance = 100.0;
// std::vector<RoiRoadBoundaryPtr> road_boundaries;
// std::vector<JunctionInfoConstPtr> junctions;
// ASSERT_EQ(0, hdmap_input_.hdmap_->
//   get_road_boundaries(point, distance, &road_boundaries, &junctions));
// ASSERT_EQ(5, road_boundaries.size());
// for (size_t i = 0; i < road_boundaries.size(); ++i) {
//   AINFO << "road boundary "<< i << " belonged to road(id) "
//            << road_boundaries[i]->id.id().c_str();
//   AINFO << "road_boundary left boundary point size: "
//            << road_boundaries[i]->left_boundary.line_points.size()
//            << " x:"<< road_boundaries[i]->left_boundary.line_points[0].x()
//            <<" y:" << road_boundaries[i]->left_boundary.line_points[0].y()
//            << " x:"<< road_boundaries[i]->left_boundary.line_points[1].x()
//            <<" y:" << road_boundaries[i]->left_boundary.line_points[1].y();
//   AINFO << "road_boundary right boundary point size: "
//            << road_boundaries[i]->right_boundary.line_points.size()
//            << " x:"<< road_boundaries[i]->right_boundary.line_points[0].x()
//            <<" y:" << road_boundaries[i]->right_boundary.line_points[0].y()
//            << " x:"<< road_boundaries[i]->right_boundary.line_points[1].x()
//            <<" y:" << road_boundaries[i]->right_boundary.line_points[1].y();
// }
// ASSERT_EQ(1, junctions.size());
// ASSERT_EQ("9", junctions[0]->id().id());
// distance = 1.0;
// ASSERT_EQ(0, hdmap_input_.hdmap_->get_road_boundaries(
//   point, distance, &road_boundaries, &junctions));
// ASSERT_EQ(1, road_boundaries.size());
// ASSERT_EQ("3", road_boundaries[0]->id.id());
// ASSERT_EQ(0, junctions.size());
// // test function GetRoiHDMapStruct()
// base::PointD pointd;
// pointd.x = 454489.3;
// pointd.y = 4439560.0;
// distance = 100.0;
// base::HdmapStructPtr hdmap_struct_ptr(new base::HdmapStruct);
// EXPECT_TRUE(hdmap_input_.GetRoiHDMapStruct(pointd,
//                                            distance, hdmap_struct_ptr));
//}
// TEST_F(HDMapInputTest, test_InitHDMap) {
// EXPECT_TRUE(hdmap_input_.InitHDMap());
//}
// TEST_F(HDMapInputTest, test_InitInternal) {
// EXPECT_TRUE(hdmap_input_.InitInternal());
//}
// TEST_F(HDMapInputTest, test_MergeBoundaryJunction) {
// function has been test through test_GetRoiHDMapStruct
/*
  adu::hdmap::RoiRoadBoundaryPtr roiroadboundary1_ptr(new
  adu::hdmap::RoiRoadBoundary);
  // set the first boundary
  // set the left_boundary of the first boundary (2 points,(-1,0),(-1,1))
  adu::common::hdmap::Point point1;
  point1.set_x(-1.0);
  point1.set_y(0);
  adu::common::hdmap::Point point2;
  point2.set_x(-1.0);
  point2.set_y(1.0);
  std::vector<adu::common::hdmap::Point> temp_line1;
  temp_line1.push_back(point1);
  temp_line1.push_back(point2);
  adu::hdmap::BoundaryLine boundaryline1;
  boundaryline1.line_points = temp_line1;
  roiroadboundary1_ptr->left_boundary = boundaryline1;
  // set the right_boundary of the first boundary (2 points,(1,0),(1,1))
  adu::common::hdmap::Point point3;
  point3.set_x(1.0);
  point3.set_y(0);
  adu::common::hdmap::Point point4;
  point4.set_x(1.0);
  point4.set_y(1.0);
  std::vector<adu::common::hdmap::Point> temp_line2;
  temp_line2.push_back(point3);
  temp_line2.push_back(point3);
  adu::hdmap::BoundaryLine boundaryline2;
  boundaryline2.line_points = temp_line2;
  roiroadboundary1_ptr->right_boundary = boundaryline2;
  // set only 1 boundary to boundaries_vec
  std::vector<adu::hdmap::RoiRoadBoundaryPtr> boundaries_vec;
  boundaries_vec.push_back(roiroadboundary1_ptr);
  //set the junctions(size is 1 and the polyon in the junction is 0)
  adu::common::hdmap::Junction junction;
  adu::common::hdmap::Polygon* polygon_ptr= new adu::common::hdmap::Polygon();
  // cannot find a member function to add point
  junction.set_allocated_polygon(polygon_ptr); // TO BE SOLVED, CANNOT reference
  junction._polygon.
  const adu::common::hdmap::Junction const_junction = junction;
  const adu::hdmap::JunctionInfo const_junctioninfo(junction);
  adu::hdmap::JunctionInfoConstPtr junctioninfo_const_ptr =
  std::make_shared<const adu::hdmap::JunctionInfo>(const_junctioninfo);
  std::vector<JunctionInfoConstPtr> junctions_vec;
  junctions_vec.push_back(junctioninfo_const_ptr);
  std::vector<base::RoadBoundary> road_boundaries;
  std::vector<base::PolygonDType> road_polygons;
  std::vector<base::PolygonDType> junction_polygons;
  hdmap_input_.MergeBoundaryJunction(boundaries_vec, junctions_vec,
  &road_boundaries, &road_polygons, &junction_polygons);
  EXPECT_EQ(road_boundaries.size(),2);
  EXPECT_EQ(road_polygons.size(),2);
  EXPECT_EQ(junction_polygons.size(),1);
   */
// delete polygon_ptr;
//}
// TEST_F(HDMapInputTest, test_DownsamplePoints) {
// base::PointD point1;
// point1.x = 0;
// point1.y = 0;
// base::PointD point2;
// point2.x = 0;
// point2.y = 1.0;
// base::PointD point3;
// point3.x = 1.0;
// point3.y = 1.0;
// base::PointD point4;
// point4.x = 1.0;
// point4.y = 0;
// base::PointDCloudPtr point_cloud_ptr1(new base::PointDCloud);
// point_cloud_ptr1->push_back(point1);
// point_cloud_ptr1->push_back(point2);
// point_cloud_ptr1->push_back(point3);
// point_cloud_ptr1->push_back(point1);
// base::PolygonDType polygon;
// hdmap_input_.DownsamplePoints(point_cloud_ptr1, &polygon);
// EXPECT_EQ(polygon.size(), 4);
// polygon.clear();
// point_cloud_ptr1->clear();
// point_cloud_ptr1->push_back(point1);
// point_cloud_ptr1->push_back(point2);
// point_cloud_ptr1->push_back(point3);
// hdmap_input_.DownsamplePoints(point_cloud_ptr1, &polygon);
// EXPECT_EQ(polygon.size(), 3);
// polygon.clear();
// point_cloud_ptr1->clear();
// point_cloud_ptr1->push_back(point1);
// point_cloud_ptr1->push_back(point2);
// point_cloud_ptr1->push_back(point2);
// point_cloud_ptr1->push_back(point3);
// hdmap_input_.DownsamplePoints(point_cloud_ptr1, &polygon, 3);
// EXPECT_EQ(polygon.size(), 3);
// polygon.clear();
// point_cloud_ptr1->clear();
// point_cloud_ptr1->push_back(point1);
// point_cloud_ptr1->push_back(point1);
// point_cloud_ptr1->push_back(point2);
// point_cloud_ptr1->push_back(point3);
// hdmap_input_.DownsamplePoints(point_cloud_ptr1, &polygon, 3);
// EXPECT_EQ(polygon.size(), 3);
//}
// TEST_F(HDMapInputTest, test_SplitBoundary) {
// base::PointDCloud boundary_line;
// for (float coordinate = -1; coordinate < 2; coordinate += 0.25) {
//   base::PointD point;
//   point.x = coordinate;
//   point.y = coordinate;
//   boundary_line.push_back(point);
// }
// base::PointD point1;
// point1.x = 0;
// point1.y = 0;
// base::PointD point2;
// point2.x = 0;
// point2.y = 1.0;
// base::PointD point3;
// point3.x = 1.0;
// point3.y = 1.0;
// base::PointD point4;
// point4.x = 1.0;
// point4.y = 0;
// base::PolygonDType polygon1;
// polygon1.push_back(point1);
// polygon1.push_back(point2);
// polygon1.push_back(point3);
// polygon1.push_back(point4);
// std::vector<base::PolygonDType> junctions;
// junctions.push_back(polygon1);
// std::vector<base::PolygonDType> boundary_line_vec;
// hdmap_input_.SplitBoundary(boundary_line, junctions, &boundary_line_vec);
// EXPECT_EQ(boundary_line_vec.size(), 2);
// point1.x = -10.0;
// point1.y = -10.0;
// point2.x = -10.0;
// point2.y = 10.0;
// point3.x = 10.0;
// point3.y = 10.0;
// point4.x = 10.0;
// point4.y = -10.0;
// polygon1.clear();
// polygon1.push_back(point1);
// polygon1.push_back(point2);
// polygon1.push_back(point3);
// polygon1.push_back(point4);
// junctions.clear();
// junctions.push_back(polygon1);
// boundary_line_vec.clear();
// hdmap_input_.SplitBoundary(boundary_line, junctions, &boundary_line_vec);
// EXPECT_EQ(boundary_line_vec.size(), 0);
// point1.x = -2.0;
// point1.y = -2.0;
// point2.x = -2.0;
// point2.y = 0.0;
// point3.x = 0.0;
// point3.y = 0.0;
// point4.x = 0.0;
// point4.y = -2.0;
// polygon1.clear();
// polygon1.push_back(point1);
// polygon1.push_back(point2);
// polygon1.push_back(point3);
// polygon1.push_back(point4);
// junctions.clear();
// junctions.push_back(polygon1);
// boundary_line_vec.clear();
// hdmap_input_.SplitBoundary(boundary_line, junctions, &boundary_line_vec);
// EXPECT_EQ(boundary_line_vec.size(), 1);
// point1.x = -2.0;
// point1.y = -2.0;
// point2.x = -2.0;
// point2.y = 0.0;
// point3.x = 0.0;
// point3.y = 0.0;
// point4.x = 0.0;
// point4.y = -2.0;
// polygon1.clear();
// polygon1.push_back(point1);
// polygon1.push_back(point2);
// polygon1.push_back(point3);
// polygon1.push_back(point4);
// base::PointD point5;
// point5.x = -0.5;
// point5.y = -0.5;
// base::PointD point6;
// point6.x = -0.5;
// point6.y = 3.0;
// base::PointD point7;
// point7.x = 3.0;
// point7.y = 3.0;
// base::PointD point8;
// point8.x = 3.0;
// point8.y = -0.5;
// base::PolygonDType polygon2;
// polygon2.push_back(point5);
// polygon2.push_back(point6);
// polygon2.push_back(point7);
// polygon2.push_back(point8);
// junctions.clear();
// junctions.push_back(polygon1);
// junctions.push_back(polygon2);
// boundary_line_vec.clear();
// hdmap_input_.SplitBoundary(boundary_line, junctions, &boundary_line_vec);
// EXPECT_EQ(boundary_line_vec.size(), 0);
//}
// TEST_F(HDMapInputTest, test_GetRoadBoundaryFilteredByJunctions) {
// base::PolygonDType left_boundary;
// for (double coordinate = -3.0; coordinate < 3.5; coordinate += 0.5) {
//   base::PointD point;
//   point.x = coordinate;
//   point.y = coordinate + 0.5;
//   left_boundary.push_back(point);
// }
// base::PolygonDType right_boundary;
// for (double  coordinate = -3.0; coordinate < 3.5; coordinate += 0.5) {
//   base::PointD point;
//   point.x = coordinate;
//   point.y = coordinate - 0.5;
//   right_boundary.push_back(point);
// }
// base::RoadBoundary road_boundary;
// road_boundary.left_boundary = left_boundary;
// road_boundary.right_boundary = right_boundary;
// std::vector<base::RoadBoundary> road_boundaries;
// road_boundaries.push_back(road_boundary);
// base::PointD point1;
// point1.x = -1.0;
// point1.y = -1.0;
// base::PointD point2;
// point2.x = -1.0;
// point2.y = 1.0;
// base::PointD point3;
// point3.x = 1.0;
// point3.y = 1.0;
// base::PointD point4;
// point4.x = 1.0;
// point4.y = -1.0;
// base::PolygonDType polygon1;
// polygon1.push_back(point1);
// polygon1.push_back(point2);
// polygon1.push_back(point3);
// polygon1.push_back(point4);
// std::vector<base::PolygonDType> junctions;
// junctions.push_back(polygon1);
// std::vector<base::RoadBoundary> flt_road_boundaries;
// EXPECT_TRUE(hdmap_input_.GetRoadBoundaryFilteredByJunctions(
//   road_boundaries, junctions, &flt_road_boundaries));
// EXPECT_EQ(flt_road_boundaries.size(), 2);
// right_boundary.clear();
// for (double  coordinate = -3.0; coordinate < 1; coordinate += 0.5) {
//   base::PointD point;
//   point.x = coordinate;
//   point.y = coordinate - 0.5;
//   right_boundary.push_back(point);
// }
// road_boundary.right_boundary = right_boundary;
// road_boundaries.clear();
// road_boundaries.push_back(road_boundary);
// flt_road_boundaries.clear();
// EXPECT_TRUE(hdmap_input_.GetRoadBoundaryFilteredByJunctions(
//   road_boundaries, junctions, &flt_road_boundaries));
// EXPECT_EQ(flt_road_boundaries.size(), 2);
// right_boundary.clear();
// for (double  coordinate = -1.0; coordinate < 1; coordinate += 0.5) {
//   base::PointD point;
//   point.x = coordinate;
//   point.y = coordinate - 0.5;
//   right_boundary.push_back(point);
// }
// road_boundary.right_boundary = right_boundary;
// road_boundaries.clear();
// road_boundaries.push_back(road_boundary);
// flt_road_boundaries.clear();
// EXPECT_TRUE(hdmap_input_.GetRoadBoundaryFilteredByJunctions(
//   road_boundaries, junctions, &flt_road_boundaries));
// EXPECT_EQ(flt_road_boundaries.size(), 2);
// road_boundaries.push_back(road_boundary);
// flt_road_boundaries.clear();
// EXPECT_TRUE(hdmap_input_.GetRoadBoundaryFilteredByJunctions(
//   road_boundaries, junctions, &flt_road_boundaries));
// EXPECT_EQ(flt_road_boundaries.size(), 4);
// base::PointD point5;
// point5.x = -10;
// point5.y = -10;
// base::PointD point6;
// point6.x = -10;
// point6.y = 10;
// base::PointD point7;
// point7.x = 10.0;
// point7.y = 10.0;
// base::PointD point8;
// point8.x = 10.0;
// point8.y = -10.0;
// base::PolygonDType polygon2;
// polygon2.push_back(point5);
// polygon2.push_back(point6);
// polygon2.push_back(point7);
// polygon2.push_back(point8);
// junctions.push_back(polygon2);
// flt_road_boundaries.clear();
// EXPECT_TRUE(hdmap_input_.GetRoadBoundaryFilteredByJunctions(
//   road_boundaries, junctions, &flt_road_boundaries));
// EXPECT_EQ(flt_road_boundaries.size(), 0);
//}
}  // namespace map
}  // namespace perception
}  // namespace apollo
