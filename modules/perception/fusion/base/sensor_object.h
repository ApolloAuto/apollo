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
#pragma once

#include <memory>
#include <string>

#include "Eigen/Core"
#include "gtest/gtest_prod.h"

#include "modules/perception/base/object.h"
#include "modules/perception/base/sensor_meta.h"
#include "modules/perception/fusion/base/base_forward_declaration.h"

namespace apollo {
namespace perception {
namespace fusion {

class SensorObject {
 public:
  SensorObject() = delete;

  explicit SensorObject(const std::shared_ptr<const base::Object>& object_ptr);

  SensorObject(const std::shared_ptr<const base::Object>& object_ptr,
               const std::shared_ptr<const SensorFrameHeader>& frame_header);

  SensorObject(const std::shared_ptr<const base::Object>& object_ptr,
               const std::shared_ptr<SensorFrame>& frame_ptr);

  // Getter
  // @brief get frame timestamp which might be different with object timestamp
  double GetTimestamp() const;
  bool GetRelatedFramePose(Eigen::Affine3d* pose) const;

  std::string GetSensorId() const;
  base::SensorType GetSensorType() const;

  inline std::shared_ptr<const base::Object> GetBaseObject() const {
    return object_;
  }

  inline double GetInvisiblePeriod() const { return invisible_period_; }

  inline void SetInvisiblePeriod(double period) { invisible_period_ = period; }

 private:
  FRIEND_TEST(SensorObjectTest, test);

  std::shared_ptr<const base::Object> object_;
  double invisible_period_ = 0.0;
  std::shared_ptr<const SensorFrameHeader> frame_header_ = nullptr;
};

typedef std::shared_ptr<SensorObject> SensorObjectPtr;
typedef std::shared_ptr<const SensorObject> SensorObjectConstPtr;

class FusedObject {
 public:
  FusedObject();
  ~FusedObject() = default;

  inline double GetTimestamp() const { return object_->latest_tracked_time; }

  inline std::shared_ptr<base::Object> GetBaseObject() { return object_; }

 private:
  std::shared_ptr<base::Object> object_;
};

typedef std::shared_ptr<FusedObject> FusedObjectPtr;

bool IsLidar(const SensorObjectConstPtr& obj);
bool IsRadar(const SensorObjectConstPtr& obj);
bool IsCamera(const SensorObjectConstPtr& obj);

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
