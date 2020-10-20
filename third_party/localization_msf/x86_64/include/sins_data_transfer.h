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

#pragma once

#include "sins_struct.h"
#include "local_sins/navigation_struct.hpp"

namespace apollo {
namespace localization {
namespace msf {

typedef adu::localization::integrated_navigation::SinsImuData SinsImuData;

class SinsDataTransfer {
 public:
  void Transfer(const MeasureData &data,
      adu::localization::integrated_navigation::MeasureData *out_data);
  void Transfer(const Position &data,
      adu::localization::integrated_navigation::Position *out_data);
  void Transfer(const Velocity &data,
      adu::localization::integrated_navigation::Velocity *out_data);
  void Transfer(const Attitude &data,
      adu::localization::integrated_navigation::Attitude *out_data);
  void Transfer(const MeasureType &data,
      adu::localization::integrated_navigation::MeasureType *out_data);
  void Transfer(const FrameType &data,
      adu::localization::integrated_navigation::FrameType *out_data);
  void Transfer(const WheelspeedData &data,
      adu::localization::integrated_navigation::WheelspeedData *out_data);
  void Transfer(const ImuData &data, SinsImuData *out_data);
  void Transfer(const SinsIntegUpdateType &data,
      adu::localization::integrated_navigation::SinsIntegUpdateType *out_data);
  void Transfer(const InsPva &data,
      adu::localization::integrated_navigation::InsPva *out_data);

  void Transfer(const adu::localization::integrated_navigation::Position &data,
      Position *out_data);
  void Transfer(const adu::localization::integrated_navigation::Velocity &data,
      Velocity *out_data);
  void Transfer(const adu::localization::integrated_navigation::Attitude &data,
      Attitude *out_data);
  void Transfer(const adu::localization::integrated_navigation::InsPva &data,
      InsPva *out_data);
};

} // msf
} // localization
} // apollo