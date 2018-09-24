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
#ifndef PERCEPTION_LIDAR_LIB_TRACKER_HM_TRACKER_BASE_FILTER_H
#define PERCEPTION_LIDAR_LIB_TRACKER_HM_TRACKER_BASE_FILTER_H

#include <string>

#include <Eigen/Core>

#include "modules/perception/lib/registerer/registerer.h"
#include "modules/perception/lidar/lib/tracker/common/track_data.h"

namespace apollo {
namespace perception {
namespace lidar {

struct FilterOption {};

class BaseFilter {
 public:
  BaseFilter() {}

  virtual ~BaseFilter() {}

  virtual bool Init(const FilterOption& option = FilterOption()) = 0;

  // @brief predict the state of filter
  // @params[IN] track_data: according track data to predict state at time
  // @params[IN] time: time for predicting
  // @return predicted states of filtering
  virtual Eigen::VectorXd Predict(TrackDataPtr track_data, double time) = 0;

  // @brief update filter with object
  // @params[IN] new_object: new object for current updating
  // @params[IN] old_object: old object for last updating
  // @params[IN] time_diff: time interval from last updating
  // @return nothing
  virtual void UpdateWithObject(TrackDataPtr track_data,
                                TrackedObjectPtr new_object, double time) = 0;

  virtual std::string name() const { return "BaseFilter"; }

 protected:
};  // class BaseFilter

typedef std::shared_ptr<BaseFilter> BaseFilterPtr;
typedef std::shared_ptr<const BaseFilter> BaseFilterConstPtr;

PERCEPTION_REGISTER_REGISTERER(BaseFilter);
#define PERCEPTION_REGISTER_TRACKFILTER(name) \
  PERCEPTION_REGISTER_CLASS(BaseFilter, name)

}  // namespace lidar
}  // namespace perception
}  // namespace apollo

#endif  // PERCEPTION_LIDAR_LIB_TRACKER_HM_TRACKER_BASE_FILTER_H
