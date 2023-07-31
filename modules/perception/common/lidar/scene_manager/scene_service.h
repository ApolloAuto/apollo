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

#include "cyber/common/macros.h"
#include "modules/perception/common/lib/registerer/registerer.h"
#include "modules/perception/common/lib/interface/base_init_options.h"

namespace apollo {
namespace perception {
namespace lidar {

using apollo::perception::BaseInitOptions;

class SceneServiceContent {
 public:
  SceneServiceContent() = default;
  virtual ~SceneServiceContent() = default;
  // @brief: get service content name
  // @return: name
  virtual std::string Name() const = 0;
  // @brief: get a copy of this service content
  // @param [out]: copy of the service content
  virtual void GetCopy(SceneServiceContent* content) const = 0;
  // @brief: set service content from outside
  // @param [in]: input service content
  virtual void SetContent(const SceneServiceContent& content) = 0;
  // @brief: whether service is ready
  // @return: status
  bool IsServiceReady() const { return service_ready_; }

 protected:
  bool service_ready_ = false;

 private:
  DISALLOW_COPY_AND_ASSIGN(SceneServiceContent);
};

PERCEPTION_REGISTER_REGISTERER(SceneServiceContent);
#define PERCEPTION_REGISTER_SCENESERVICECONTENT(name) \
  PERCEPTION_REGISTER_CLASS(SceneServiceContent, name)

typedef std::shared_ptr<SceneServiceContent> SceneServiceContentPtr;
typedef std::shared_ptr<const SceneServiceContent> SceneServiceContentConstPtr;

struct SceneServiceInitOptions : public BaseInitOptions {};

class SceneService {
 public:
  SceneService() = default;
  virtual ~SceneService() = default;
  // @brief: initialize scene service
  // @param [in]: init options
  virtual bool Init(
      const SceneServiceInitOptions& options = SceneServiceInitOptions()) = 0;
  // @brief: get service name
  // @return: name
  virtual std::string Name() const = 0;
  // @brief: get a copy of service content
  // @param [in]: service content
  void GetServiceContentCopy(SceneServiceContent* content) {
    std::lock_guard<std::mutex> lock(mutex_);
    self_content_->GetCopy(content);
  }
  // @brief: update service content from copy
  // @param [in]: service content
  void UpdateServiceContent(const SceneServiceContent& content) {
    std::lock_guard<std::mutex> lock(mutex_);
    self_content_->SetContent(content);
  }

 protected:
  SceneServiceContentPtr self_content_;
  std::mutex mutex_;

 private:
  DISALLOW_COPY_AND_ASSIGN(SceneService);
};

PERCEPTION_REGISTER_REGISTERER(SceneService);
#define PERCEPTION_REGISTER_SCENESERVICE(name) \
  PERCEPTION_REGISTER_CLASS(SceneService, name)

typedef std::shared_ptr<SceneService> SceneServicePtr;
typedef std::shared_ptr<const SceneService> SceneServiceConstPtr;

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
