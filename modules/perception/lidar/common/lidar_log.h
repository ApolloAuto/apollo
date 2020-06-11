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

// 100-199 reserve for hd lidar(64...) module error
#define HDLIDAR_SOURCE_DATA_ERROR 100
#define HDLIDAR_GET_POSE_ERROR 101
#define HDLIDAR_GET_HDMAP_ERROR 102
#define HDLIDAR_GET_STATICMAP_ERROR 103
#define HDLIDAR_GET_INSTANCE_ERROR 110
#define HDLIDAR_ALGORITHM_INIT_ERROR 111
#define HDLIDAR_ALGORITHM_RUNNIG_ERROR 112
#define HDLIDAR_SEND_MESSAGE_ERROR 113
#define HDLIDAR_GET_CONFIG_ERROR 120

// 200-299 reserve for ld lidar(1/16...) module error
#define LDLIDAR_SOURCE_DATA_ERROR 200
#define LDLIDAR_GET_POSE_ERROR 201
#define LDLIDAR_GET_HDMAP_ERROR 202
#define LDLIDAR_DATA_TIMESTAMP_ERROR 203
#define LDLIDAR_GET_INSTANCE_ERROR 210
#define LDLIDAR_ALGORITHM_INIT_ERROR 211
#define LDLIDAR_ALGORITHM_RUNNIG_ERROR 212
#define LDLIDAR_SEND_MESSAGE_ERROR 213
#define LDLIDAR_TIME_SYNCHRONIZE_ERROR 214
#define LDLIDAR_GET_CONFIG_ERROR 220
