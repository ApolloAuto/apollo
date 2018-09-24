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
#include "modules/perception/lib/utils/string_util.h"

#include <algorithm>

#include "modules/perception/base/log.h"

namespace apollo {
namespace perception {
namespace lib {

using std::string;
using std::vector;

void StringUtil::Explode(const string &str, char c, vector<string> *terms_vec) {
  string term;
  for (auto &cur_char : str) {
    if (cur_char != c) {
      term += cur_char;
      continue;
    }
    // find one
    if (!term.empty()) {
      terms_vec->push_back(term);
      term.clear();
    }
  }
  if (!term.empty()) {
    terms_vec->push_back(term);
  }
}

void StringUtil::Trim(TrimType type, string *result_str) {
  string &str = *result_str;
  switch (type) {
    case TRIM_LEFT:
      str.erase(str.begin(),
                std::find_if_not(str.begin(), str.end(),
                                 [](int c) { return isspace(c) != 0; }));
      break;
    case TRIM_RIGHT:
      str.erase(std::find_if_not(str.rbegin(), str.rend(),
                                 [](int c) { return isspace(c) != 0; })
                    .base(),
                str.end());
      break;
    case TRIM_BOTH:
      Trim(TRIM_LEFT, result_str);
      Trim(TRIM_RIGHT, result_str);
      break;
    default:
      LOG_ERROR << "Invalid trim type, type: " << type;
      break;
  }
}

string StringUtil::Trim(TrimType type, const string &str) {
  string result_str(str);
  StringUtil::Trim(type, &result_str);
  return result_str;
}

string StringUtil::LTrim(const std::string &str) {
  return Trim(TRIM_LEFT, str);
}

string StringUtil::RTrim(const std::string &str) {
  return Trim(TRIM_RIGHT, str);
}

string StringUtil::Trim(const std::string &str) { return Trim(TRIM_BOTH, str); }

bool StringUtil::StartWith(const string &raw_str, const string &prefix) {
  if (raw_str.size() < prefix.size()) {
    return false;
  }

  auto raw_iter = raw_str.cbegin();
  auto pre_iter = prefix.cbegin();
  for (; pre_iter != prefix.end(); ++raw_iter, ++pre_iter) {
    if (*raw_iter != *pre_iter) {
      return false;
    }
  }

  return true;
}

bool StringUtil::EndWith(const string &raw_str, const string &suffix) {
  if (raw_str.size() < suffix.size()) {
    return false;
  }

  auto raw_iter = raw_str.crbegin();
  auto suffix_iter = suffix.crbegin();
  for (; suffix_iter != suffix.crend(); ++suffix_iter, ++raw_iter) {
    if (*suffix_iter != *raw_iter) {
      return false;
    }
  }

  return true;
}

bool StringUtil::StrToInt(const string &str, int *ret_val) {
  try {
    *ret_val = std::stoi(str);
    return true;
  } catch (...) {
    LOG_ERROR << "std::stoi() failed.str:" << str;
    return false;
  }
}

}  // namespace lib
}  // namespace perception
}  // namespace apollo
