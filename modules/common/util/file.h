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

// * Deprecated *
// Please note that this util has been deprecated. Please include and use
// cyber/common/file.h directly.

#pragma once

#include "cyber/common/file.h"
#include "modules/common/util/string_util.h"

/**
 * @namespace apollo::common::util
 * @brief apollo::common::util
 */
namespace apollo {
namespace common {
namespace util {

// TODO(all): The file utils have been moved into cyber. After migrating the
// usages we'll retire the aliases here.
using apollo::cyber::common::SetProtoToASCIIFile;
using apollo::cyber::common::GetProtoFromASCIIFile;
using apollo::cyber::common::SetProtoToBinaryFile;
using apollo::cyber::common::GetProtoFromBinaryFile;
using apollo::cyber::common::GetProtoFromFile;
using apollo::cyber::common::GetContent;
using apollo::cyber::common::GetAbsolutePath;
using apollo::cyber::common::PathExists;
using apollo::cyber::common::DirectoryExists;
using apollo::cyber::common::Glob;
using apollo::cyber::common::CopyFile;
using apollo::cyber::common::CopyDir;
using apollo::cyber::common::Copy;
using apollo::cyber::common::EnsureDirectory;
using apollo::cyber::common::RemoveAllFiles;
using apollo::cyber::common::ListSubPaths;
using apollo::cyber::common::GetFileName;

}  // namespace util
}  // namespace common
}  // namespace apollo
