/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#pragma once

namespace apollo {
namespace perception {
namespace radar {

const double PI = 3.1415926535898;
const int MAX_RADAR_IDX = 2147483647;
const double CONTI_ARS_INTERVAL = 0.074;
const double MIN_PROBEXIST = 0.08;

enum ContiObjectType {
  CONTI_POINT = 0,
  CONTI_CAR = 1,
  CONTI_TRUCK = 2,
  CONTI_PEDESTRIAN = 3,
  CONTI_MOTOCYCLE = 4,
  CONTI_BICYCLE = 5,
  CONTI_WIDE = 6,
  CONTI_TYPE_UNKNOWN = 7,
  CONTI_MAX_OBJECT_TYPE = 8
};

enum ContiMeasState {
  CONTI_DELETED = 0,
  CONTI_NEW = 1,
  CONTI_MEASURED = 2,
  CONTI_PREDICTED = 3,
  CONTI_DELETED_FOR = 4,
  CONTI_NEW_FROM_MERGE = 5
};

enum ContiDynProp {
  CONTI_MOVING = 0,
  CONTI_STATIONARY = 1,
  CONTI_ONCOMING = 2,
  CONTI_STATIONARY_CANDIDATE = 3,
  CONTI_DYNAMIC_UNKNOWN = 4,
  CONTI_CROSSING_STATIONARY = 5,
  CONTI_CROSSING_MOVING = 6,
  CONTI_STOPPED = 7
};

}  // namespace radar
}  // namespace perception
}  // namespace apollo
