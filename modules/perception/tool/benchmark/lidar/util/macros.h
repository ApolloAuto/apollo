/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include <iomanip>

#ifdef GLOG_TIMESTAMP
#undef GLOG_TIMESTAMP
#endif

#define GLOG_TIMESTAMP(timestamp) \
  std::fixed << std::setprecision(9) << timestamp

#define GLOG_DOUBLE(value) std::fixed << std::setprecision(6) << value

#define GLOG_DOUBLE_WITH_PRECISION(value, precision) \
  std::fixed << std::setprecision(precision) << value

// for unsed variable
#undef UNUSED
#define UNUSED(x) (void)(x)
