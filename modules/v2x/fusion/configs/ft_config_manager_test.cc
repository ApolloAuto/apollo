/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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
#include "modules/v2x/fusion/configs/ft_config_manager.h"

#include "gtest/gtest.h"

namespace apollo {
namespace v2x {
namespace ft {

TEST(FTConfigManager, read_and_write) {
  FTConfigManager* ft_config_manager_ptr = FTConfigManager::Instance();
  auto& fusion_params = ft_config_manager_ptr->fusion_params_.params;
  EXPECT_FALSE(fusion_params.score_params().check_type());
  EXPECT_EQ(fusion_params.score_params().prob_scale(), 0.125);
}

}  // namespace ft
}  // namespace v2x
}  // namespace apollo
