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
 * @file lm_sampler.h
 * @brief The class of LMSampler.
 */

#ifndef MODULES_LOCALIZATION_LMD_PREDICTOR_PERCEPTION_LM_SAMPLER_H_
#define MODULES_LOCALIZATION_LMD_PREDICTOR_PERCEPTION_LM_SAMPLER_H_

#include <vector>

#include "modules/perception/proto/perception_obstacle.pb.h"

#include "modules/localization/lmd/predictor/perception/pc_registrator.h"

/**
 * @namespace apollo::localization
 * @brief apollo::localization
 */
namespace apollo {
namespace localization {

/**
 * @class LMSampler
 *
 * @brief  Sampler for lane markers from percption.
 */
class LMSampler {
 public:
  LMSampler();
  virtual ~LMSampler();

  /**
   * @brief  Update map for range.
   * @param lane_markers The lane marker from percption.
   * @return The sampling points.
   */
  const std::vector<PCSourcePoint>& Sampling(
      const apollo::perception::LaneMarkers& lane_markers);

 private:
  /**
   * @brief  Get curve value by  params.
   * @param  x_value: value of x.
   *         c0: position.
   *         c1: heading_angle.
   *         c2: curvature.
   *         c3: curvature_derivative.
   * @return y = c3 * x**3 + c2 * x**2 + c1 * x + c0.
   */
  double get_curve_value(double x_value, double c0, double c1, double c2,
                         double c3) const;

  /**
   * @brief  Get the first derivative value according to x_value and curve
   * analysis formula y = c3 * x**3 + c2 * x**2 + c1 * x + c0
   * @param  x_value: value of x.
   *         c0: position.
   *         c1: heading_angle.
   *         c2: curvature.
   *         c3: curvature_derivative.
   * @return the first derivative value when x equal to x_value
   */
  double calculate_derivative(double x_value, double c0, double c1, double c2,
                              double c3) const;

  /**
    * @brief  Get the curvity value according to x_value and curve analysis
    formula y = c3 * x**3 + c2 * x**2 + c1 * x + c0
    * @param  x_value: value of x.
    *         c0: position.
    *         c1: heading_angle.
    *         c2: curvature.
    *         c3: curvature_derivative.
    * @return K = |y''| / (1 + y'**2)**(3.0/2)
             curvity_value K according to the analysis formula with x = x_value
    */
  double calculate_curvity(double x_value, double c0, double c1, double c2,
                           double c3) const;

  /**
   * @brief  Sampling points frlane_markerker.
   * @param  One lane marker oflane_markerrom percption.
   * @return The run status.
   */
  int SamplingForOneLaneMarker(
      const apollo::perception::LaneMarker& lane_marker);

 private:
  std::vector<PCSourcePoint> pc_sourcepoint_;
};

}  // namespace localization
}  // namespace apollo

#endif  // MODULES_LOCALIZATION_LMD_PREDICTOR_PERCEPTION_LM_SAMPLER_H_
