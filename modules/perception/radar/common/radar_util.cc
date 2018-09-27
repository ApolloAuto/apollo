// Copyright 2018 Baidu Inc. All Rights Reserved.
// @author: Hui Yujiang (huiyujiang@baidu.com)
// @file: types.h
// @brief: radar util

#include "modules/perception/radar/common/radar_util.h"

namespace apollo {
namespace perception {
namespace radar {

void MockRadarPolygon(base::ObjectPtr object) {
  double theta = object->theta;
  const auto& center = object->center;
  double length = object->size(0);
  double width = object->size(1);
  Eigen::Matrix2d rotation;
  rotation << cos(theta), -sin(theta), sin(theta), cos(theta);
  Eigen::Vector2d local_poly(0, 0);
  Eigen::Vector2d world_poly;
  object->polygon.resize(4);
  local_poly(0) = -0.5 * length;
  local_poly(1) = -0.5 * width;
  world_poly = rotation * local_poly;
  object->polygon[0].x = center(0) + world_poly(0);
  object->polygon[0].y = center(1) + world_poly(1);
  object->polygon[0].z = center(2);
  local_poly(0) = -0.5 * length;
  local_poly(1) = +0.5 * width;
  world_poly = rotation * local_poly;
  object->polygon[1].x = center(0) + world_poly(0);
  object->polygon[1].y = center(1) + world_poly(1);
  object->polygon[1].z = center(2);
  local_poly(0) = +0.5 * length;
  local_poly(1) = +0.5 * width;
  world_poly = rotation * local_poly;
  object->polygon[2].x = center(0) + world_poly(0);
  object->polygon[2].y = center(1) + world_poly(1);
  object->polygon[2].z = center(2);
  local_poly(0) = +0.5 * length;
  local_poly(1) = -0.5 * width;
  world_poly = rotation * local_poly;
  object->polygon[3].x = center(0) + world_poly(0);
  object->polygon[3].y = center(1) + world_poly(1);
  object->polygon[3].z = center(2);
}

}  // namespace radar
}  // namespace perception
}  // namespace apollo
