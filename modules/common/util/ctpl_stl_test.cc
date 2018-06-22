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

#include "modules/common/util/ctpl_stl.h"

#include <algorithm>
#include <atomic>
#include <iterator>
#include <set>
#include <sstream>
#include <string>

#include "gtest/gtest.h"

namespace apollo {
namespace common {
namespace util {

namespace {

std::atomic<int> n(0);

void simple_add() { n++; }
void simple_minus() { n--; }

// Attention: don't use overloaded functions, otherwise the compiler can't
// deduce the correct edition.
std::string filter_duplicates_str(int id, const char* str1, const char* str2,
                                  const char* str3, const char* str4) {
  // id is unused.

  std::stringstream ss_in;
  ss_in << str1 << " " << str2 << " " << str3 << " " << str4;

  std::set<std::string> string_set;
  std::istream_iterator<std::string> beg(ss_in);
  std::istream_iterator<std::string> end;
  std::copy(beg, end, std::inserter(string_set, string_set.end()));
  std::stringstream ss_out;
  std::copy(std::begin(string_set), std::end(string_set),
            std::ostream_iterator<std::string>(ss_out, " "));

  return ss_out.str();
}

std::string filter_duplicates(int id) {
  // id is unused.

  std::stringstream ss_in;
  ss_in
      << "a a b b b c foo foo bar foobar foobar hello world hello hello world";
  std::set<std::string> string_set;
  std::istream_iterator<std::string> beg(ss_in);
  std::istream_iterator<std::string> end;
  std::copy(beg, end, std::inserter(string_set, string_set.end()));

  std::stringstream ss_out;
  std::copy(std::begin(string_set), std::end(string_set),
            std::ostream_iterator<std::string>(ss_out, " "));

  return ss_out.str();
}

}  // namespace

TEST(ThreadPool, simple) {
  ThreadPool p(5);
  std::vector<std::future<void>> k;
  for (int i = 0; i < 1000; ++i) {
    auto f1 = std::bind(simple_add);
    k.push_back(std::move(p.Push(f1)));
  }
  for (auto& task : k) {
    task.wait();
  }
  EXPECT_EQ(n.load(), 1000);

  k.clear();

  for (int i = 0; i < 500; ++i) {
    auto f1 = std::bind(simple_add);
    auto f2 = std::bind(simple_minus);
    auto t1 = p.Push(f1);
    auto t2 = p.Push(f2);
    k.push_back(std::move(t1));
    k.push_back(std::move(t2));
  }
  for (auto& task : k) {
    task.wait();
  }
  EXPECT_EQ(n.load(), 1000);
}

TEST(ThreadPool, filter_duplicates) {
  const unsigned int hardware_threads = std::thread::hardware_concurrency();
  const unsigned int threads =
      std::min(hardware_threads != 0 ? hardware_threads : 2, 50U);
  ThreadPool p(threads);

  std::vector<std::future<std::string>> futures1, futures2;
  for (int i = 0; i < 1000; ++i) {
    futures1.push_back(std::move(p.Push(
        filter_duplicates_str, "thread pthread", "pthread thread good news",
        "today is a good day", "she is a six years old girl")));
    futures2.push_back(std::move(p.Push(filter_duplicates)));
  }

  for (int i = 0; i < 1000; ++i) {
    std::string result1 = futures1[i].get();
    std::string result2 = futures2[i].get();
    EXPECT_STREQ(
        result1.c_str(),
        "a day girl good is news old pthread she six thread today years ");
    EXPECT_STREQ(result2.c_str(), "a b bar c foo foobar hello world ");
  }
}

}  // namespace util
}  // namespace common
}  // namespace apollo
