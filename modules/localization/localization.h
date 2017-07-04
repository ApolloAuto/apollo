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
 * @file localization.h
 * @brief The class of Localization
 */

#ifndef MODULES_LOCALIZATION_LOCALIZATION_H_
#define MODULES_LOCALIZATION_LOCALIZATION_H_

#include <memory>
#include <string>

#include "modules/common/apollo_app.h"
#include "modules/common/status/status.h"
#include "modules/common/util/factory.h"
#include "modules/localization/localization_base.h"
#include "modules/localization/proto/localization_config.pb.h"

/**
 * @namespace apollo::localization
 * @brief apollo::localization
 */
namespace apollo {
namespace localization {

/**
 * @class Localization
 *
 * @brief Localization module main class. It processes GPS and IMU as input,
 * to generate localization info.
 */
class Localization : public apollo::common::ApolloApp {
 public:
  /**
   * @brief module name
   * @return module name
   */
  std::string Name() const override;

  /**
   * @brief module initialization function
   * @return initialization status
   */
  apollo::common::Status Init() override;

  /**
   * @brief module start function
   * @return start status
   */
  apollo::common::Status Start() override;

  /**
   * @brief module stop function
   * @return stop status
   */
  void Stop() override;

 private:
  void RegisterLocalizationMethods();

  std::unique_ptr<LocalizationBase> localization_;
  apollo::common::util::Factory<LocalizationConfig::LocalizationType,
                                LocalizationBase>
      localization_factory_;
  LocalizationConfig config_;
};

}  // namespace localization
}  // namespace apollo

#endif  // MODULES_LOCALIZATION_LOCALIZATION_H_
