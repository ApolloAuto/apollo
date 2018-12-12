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

#include "cyber/common/log.h"

// 500-599 reserve for fusion module error
#define FUSION_GET_POSE_ERROR 500          // get pose error
#define FUSION_GET_HDMAP_ERROR 501         // get hdmap error
#define FUSION_COMPONENT_CONF_ERROR 502    // get component config error
#define FUSION_GET_CAR_CENTER_ERROR 503    // get main car center point error,
#define FUSION_GET_INTRINSICS_ERROR 504    // get intrinsics error
#define FUSION_GET_INSTANCE_ERROR 510      // get algorithm instance error
#define FUSION_ALGORITHM_INIT_ERROR 511    // algorithm init error
#define FUSION_ALGORITHM_RUNNIG_ERROR 512  // algorithm running error
#define FUSION_SEND_MESSAGE_ERROR 513      // send message error
#define FUSION_GET_CONFIG_ERROR 520        // get parameter config error
#define FUSION_COMPUTE_CENTER_ERROR 521    // compute points center error
#define FUSION_MESSAGE_ORDER_ERROR 522     // the received message order error
#define FUSION_TRACK_DATA_EMPTY 523        // no sensor data of the tracker
#define FUSION_HM_ASSIGN_ERROR 524         // hm assign error
#define FUSION_BBAMANAGER_ERROR 525        // BBAManager throw error
