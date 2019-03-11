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

/**
 * @file
 * @brief This file provides the declaration of the class
 * "FileOperator".
 */

#ifndef MODULES_TOOLS_NAVI_GENERATOR_BACKEND_UTIL_FILE_OPERATOR_H_
#define MODULES_TOOLS_NAVI_GENERATOR_BACKEND_UTIL_FILE_OPERATOR_H_

#include <list>
#include <string>
#include <vector>

#include "modules/planning/reference_line/reference_point.h"

/**
 * @namespace apollo::navi_generator::util
 * @brief apollo::navi_generator::util
 */
namespace apollo {
namespace navi_generator {
namespace util {

class FileOperator {
 public:
  FileOperator() = default;
  ~FileOperator() = default;

 public:
  /**
   * @brief Import smoothed file.
   * @return  Return true for success.
   */
  bool Import(const std::string& filename,
              std::vector<apollo::planning::ReferencePoint>* const lanepoints);
  /**
   * @brief Import smoothed file.
   * @return  Return true for success.
   */
  bool Import(const std::string& filename,
              std::vector<unsigned char>* const data);
  /**
   * @brief Export smoothed file.
   * @return  Return true for success.
   */
  bool Export(const std::string& filename,
              const std::vector<apollo::planning::ReferencePoint>& lanepoints);
  /**
   * @brief Export smoothed file.
   * @return  Return true for success.
   */
  bool Export(const std::string& filename,
              const std::vector<unsigned char>& data);
};

}  // namespace util
}  // namespace navi_generator
}  // namespace apollo

#endif  // MODULES_TOOLS_NAVI_GENERATOR_BACKEND_UTIL_FILE_OPERATOR_H_
