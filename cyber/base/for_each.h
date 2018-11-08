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

#ifndef CYBER_BASE_FOR_EACH_H_
#define CYBER_BASE_FOR_EACH_H_

namespace apollo {
namespace cyber {
namespace base {

template <typename Lhs, typename Rhs>
class HasLess {
  template <typename, typename>
  static int test(...);

  template <typename A, typename B>
  static char test(decltype(A() < B())*);

 public:
  static constexpr bool value = sizeof(test<Lhs, Rhs>(nullptr)) == 1;
};

template <class Value, class End>
typename std::enable_if<HasLess<Value, End>::value, bool>::type LessThan(
    const Value& val, const End& end) {
  return val < end;
}

template <class Value, class End>
typename std::enable_if<!HasLess<Value, End>::value, bool>::type LessThan(
    const Value& val, const End& end) {
  return val != end;
}

#define FOR_EACH(i, begin, end)           \
  for (auto i = (true ? (begin) : (end)); \
       apollo::cyber::base::LessThan(i, (end)); ++i)

}  // namespace base
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_BASE_FOR_EACH_H_
