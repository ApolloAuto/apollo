/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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
 * @file qp_path_optimizer.h
 **/

#ifndef BAIDU_IDG_HOUSTON_OPTIMIZER_QP_SPLINE_PATH_OPTIMIZER_H_
#define BAIDU_IDG_HOUSTON_OPTIMIZER_QP_SPLINE_PATH_OPTIMIZER_H_

#include "optimizer/path_optimizer.h"

#include "boost/property_tree/ptree.hpp"

#include "optimizer/qp_spline_path_optimizer/qp_spline_path_generator.h"

namespace apollo {
namespace planning {

class QPSplinePathOptimizer : public PathOptimizer {
 public:
  explicit QPSplinePathOptimizer(const std::string& name,
                                 const boost::property_tree::ptree& ptree);

  virtual ErrorCode optimize(const DataCenter& data_center,
                             const SpeedData& speed_data,
                             const ReferenceLine& reference_line,
                             const ::adu::planning::TrajectoryPoint& init_point,
                             DecisionData* const decision_data,
                             PathData* const path_data) const override;

 private:
  std::unique_ptr<QPSplinePathGenerator> _path_generator = nullptr;
};

}  // namespace planning
}  // namespace apollo

#endif  // BAIDU_IDG_HOUSTON_OPTIMIZER_SPLINE_QP_PATH_OPTIMIZER_H_
