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
#ifndef PERCEPTION_LIB_UTILS_STRING_UTIL_H_
#define PERCEPTION_LIB_UTILS_STRING_UTIL_H_

#include <string>
#include <vector>

namespace apollo {
namespace perception {
namespace lib {

enum TrimType {
  TRIM_UNKNOWN = 0,
  TRIM_LEFT = 1,
  TRIM_RIGHT = 2,
  TRIM_BOTH = 3,
};

class StringUtil {
 public:
  StringUtil() = default;
  ~StringUtil() = default;

  // @brief: split string by one character
  //         a,b,c => a b c
  // @param [in]: the string you want to explode
  // @param [in]: the character used to split the string
  // @param [out]: result strings after exploded by character
  static void Explode(const std::string &str, char c,
                      std::vector<std::string> *terms_vec);

  // @brief: trim the string, in place vesion
  // @param [in]: type, see TrimType enum
  // @param [out]: trimed string
  // @return: void
  static void Trim(TrimType type, std::string *str);

  // @brief: trim the string, copy version
  // @param [in]: type, see TrimType
  // @param [in]: str, the string you want to trim
  // @return: trimed string
  static std::string Trim(TrimType type, const std::string &str);

  // @brief: other convenient versions for trim function
  //         you can use functions below with fewer args
  static std::string LTrim(const std::string &str);
  static std::string RTrim(const std::string &str);
  static std::string Trim(const std::string &str);

  // @breif: convert digit to string
  //         StringUtil::Digit2String(123) = "123"
  // @param [in]: int long double
  // @return: converted string
  // @note: when you use float or double, you will get
  //        StringUtil::Digit2String(-123.1)="-123.100000"
  template <typename T>
  static std::string Digit2String(T number);

  // @brief: judge raw_str start with prefix or not
  //         StringUtil::StartWith("abcedf.txt", "abc") = true
  static bool StartWith(const std::string &raw_str, const std::string &prefix);

  // @brief: judge raw_str end with suffix or not
  //         StringUtil::EndWith("abc.txt", "txt") = true
  static bool EndWith(const std::string &raw_str, const std::string &suffix);

  // string to int
  static bool StrToInt(const std::string &str, int *ret_val);

  StringUtil(const StringUtil &) = delete;
  StringUtil &operator=(const StringUtil &) = delete;

 private:
};

template <typename T>
std::string StringUtil::Digit2String(T number) {
  return std::to_string(number);
}

}  // namespace lib
}  // namespace perception
}  // namespace apollo

#endif  // PERCEPTION_LIB_UTILS_STRING_UTIL_H_
