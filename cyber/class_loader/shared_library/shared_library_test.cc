/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#include "cyber/class_loader/shared_library/shared_library.h"

#include <cmath>

#include "gtest/gtest.h"

namespace {

const std::string& SAMPLE_LIB =
    "/apollo/bazel-bin/cyber/class_loader/shared_library/libcyber_sample.so";
constexpr double epsilon = 1.0e-8;

}  // namespace

namespace apollo {
namespace cyber {
namespace class_loader {

TEST(SharedLibraryTest, symbol_test_1) {
  auto shared_lib = std::make_shared<SharedLibrary>();
  EXPECT_TRUE(shared_lib->GetPath().empty());
  EXPECT_FALSE(shared_lib->IsLoaded());

  EXPECT_NO_THROW(shared_lib->Load(SAMPLE_LIB));
  EXPECT_TRUE(shared_lib->IsLoaded());

  EXPECT_THROW(shared_lib->Load(SAMPLE_LIB), LibraryAlreadyLoadedException);

  EXPECT_TRUE(shared_lib->HasSymbol("sample_add"));
  auto symbol = shared_lib->GetSymbol("sample_add");
  EXPECT_TRUE(symbol != nullptr);

  typedef int (*BinaryFunc)(int, int);
  auto pf = reinterpret_cast<BinaryFunc>(symbol);
  EXPECT_EQ(pf(3, 5), 8);
  shared_lib->Unload();
}

TEST(SharedLibraryTest, symbol_test_2) {
  auto shared_lib = std::make_shared<SharedLibrary>(SAMPLE_LIB);
  EXPECT_TRUE(shared_lib->IsLoaded());

  typedef double (*UnaryFunc)(double);
  auto symbol = shared_lib->GetSymbol("sample_sin");
  EXPECT_TRUE(symbol != nullptr);

  auto sample_sin = reinterpret_cast<UnaryFunc>(symbol);
  EXPECT_NEAR(sample_sin(M_PI / 2), 1.0, epsilon);

  shared_lib->Unload();
}

}  // namespace class_loader
}  // namespace cyber
}  // namespace apollo
