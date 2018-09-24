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
#ifndef PERCEPTION_FUSION_BASE_SENSOR_OBJECT_H_
#define PERCEPTION_FUSION_BASE_SENSOR_OBJECT_H_
#include <memory>
#include <string>
#include "Eigen/Core"

#include "modules/perception/base/object.h"
#include "modules/perception/base/sensor_meta.h"
#include "modules/perception/fusion/base/base_forward_declaration.h"

namespace apollo {
namespace perception {
namespace fusion {

class SensorObject {
 public:
  SensorObject() = delete;

  SensorObject(const base::ObjectConstPtr& object_ptr,
               const SensorFramePtr& frame_ptr);

  // Getter
  // @brief get frame timestamp which might be different with object timestamp
  double GetTimestamp() const;
  bool GetRelatedFramePose(Eigen::Affine3d* pose) const;

  std::string GetSensorId() const;
  base::SensorType GetSensorType() const;

  inline base::ObjectConstPtr GetBaseObject() const { return object_; }

  inline double GetInvisiblePeriod() const { return invisible_period_; }

  inline void SetInvisiblePeriod(double period) { invisible_period_ = period; }

 private:
  inline bool CheckFrameExist() const {
    bool expired = frame_ptr_.expired();
    return !expired;
  }

 private:
  base::ObjectConstPtr object_;
  double invisible_period_ = 0.0;
  std::weak_ptr<const SensorFrame> frame_ptr_;
};

typedef std::shared_ptr<SensorObject> SensorObjectPtr;
typedef std::shared_ptr<const SensorObject> SensorObjectConstPtr;

class FusedObject {
 public:
  FusedObject();
  ~FusedObject() = default;

  inline double GetTimestamp() const { return object_->latest_tracked_time; }

  inline base::ObjectPtr GetBaseObject() { return object_; }

 private:
  base::ObjectPtr object_;
};

typedef std::shared_ptr<FusedObject> FusedObjectPtr;

bool IsLidar(const SensorObjectConstPtr& obj);
bool IsRadar(const SensorObjectConstPtr& obj);
bool IsCamera(const SensorObjectConstPtr& obj);

}  // namespace fusion
}  // namespace perception
}  // namespace apollo

#endif  // PERCEPTION_FUSION_BASE_SENSOR_OBJECT_H_
