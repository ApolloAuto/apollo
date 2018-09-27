// Copyright 2018 Baidu Inc. All Rights Reserved.
// @author: Yujiang Hui (huiyujiang@baidu.com)
// @file: base_filter.h
// @brief: radar filter interface

#ifndef RADAR_LIB_INTERFACE_BASE_FILTER_H_
#define RADAR_LIB_INTERFACE_BASE_FILTER_H_
#include <Eigen/Core>
#include <string>
#include "modules/perception/base/object.h"

namespace apollo {
namespace perception {
namespace radar {
class BaseFilter {
 public:
  BaseFilter() : name_("BaseFilter") {}
  virtual ~BaseFilter() {}
  virtual void Init(const base::Object& object) = 0;
  virtual Eigen::VectorXd Predict(double time_diff) = 0;
  virtual Eigen::VectorXd UpdateWithObject(const base::Object& new_object,
                                           double time_diff) = 0;
  virtual void GetState(Eigen::Vector3d* anchor_point,
                        Eigen::Vector3d* velocity) = 0;
  virtual Eigen::Matrix4d GetCovarianceMatrix() = 0;
  std::string Name() { return name_; }

 protected:
  std::string name_;
};

}  // namespace radar
}  // namespace perception
}  // namespace apollo
#endif  // RADAR_LIB_INTERFACE_BASE_FILTER_H_
