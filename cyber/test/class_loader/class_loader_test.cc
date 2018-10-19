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

#include <gtest/gtest.h>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "cyber/class_loader/class_loader.h"
#include "cyber/class_loader/class_loader_manager.h"
#include "cyber/cyber.h"
#include "cyber/test/class_loader/base.h"

namespace apollo {
namespace cyber {
namespace class_loader {

using utility::IsLibraryLoadedByAnybody;

class ClassLoaderTest : public ::testing::Test {
 protected:
  ClassLoaderTest() {
    const char* cyber_path = ::getenv("CYBER_PATH");
    std::string test_path(cyber_path);
    test_path += "/test/cyber/unit_test/";
    library_1_ = test_path + "libTestPlugins1.so";
    library_2_ = test_path + "libTestPlugins2.so";
  }

  virtual ~ClassLoaderTest() {}

  virtual void SetUp() {}

  virtual void TearDown() {}

  class InvalidBaseClass {};

  void CreateObj(ClassLoaderManager* loader) {
    std::vector<std::string> classes = loader->GetValidClassNames<Base>();
    for (size_t i = 0; i < classes.size(); ++i) {
      loader->CreateClassObj<Base>(classes[i])->DoSomething();
    }
  }

  void LoadLib(ClassLoaderManager* loaderMgr) {
    loaderMgr->LoadLibrary(library_1_);
    ASSERT_TRUE(loaderMgr->IsLibraryValid(library_1_));
  }

  std::string library_1_;
  std::string library_2_;
};

TEST_F(ClassLoaderTest, createClassObj) {
  ClassLoader loader1(library_1_);
  EXPECT_EQ(library_1_, loader1.GetLibraryPath());
  auto rect_obj = loader1.CreateClassObj<Base>("Rect");
  EXPECT_TRUE(nullptr != rect_obj);
  if (rect_obj) {
    rect_obj->DoSomething();
  }
  EXPECT_EQ(nullptr, loader1.CreateClassObj<Base>("Xeee"));
  SUCCEED();
}

TEST_F(ClassLoaderTest, loadLibCounts) {
  ClassLoader loader1(library_1_);
  ASSERT_TRUE(loader1.IsLibraryLoaded());

  loader1.LoadLibrary();
  loader1.LoadLibrary();
  ASSERT_TRUE(loader1.IsLibraryLoaded());

  loader1.UnloadLibrary();
  ASSERT_TRUE(loader1.IsLibraryLoaded());

  loader1.UnloadLibrary();
  ASSERT_TRUE(loader1.IsLibraryLoaded());

  loader1.UnloadLibrary();
  ASSERT_FALSE(loader1.IsLibraryLoaded());

  loader1.UnloadLibrary();
  ASSERT_FALSE(loader1.IsLibraryLoaded());

  loader1.LoadLibrary();
  ASSERT_TRUE(loader1.IsLibraryLoaded());

  SUCCEED();
}

TEST_F(ClassLoaderTest, multiTimesLoadunload) {
  ClassLoader loader1(library_1_);
  ASSERT_TRUE(loader1.LoadLibrary());
  loader1.LoadLibrary();
  ASSERT_TRUE(IsLibraryLoadedByAnybody(library_1_));
  loader1.UnloadLibrary();
  ASSERT_TRUE(IsLibraryLoadedByAnybody(library_1_));
  loader1.UnloadLibrary();
  ASSERT_TRUE(IsLibraryLoadedByAnybody(library_1_));
  loader1.UnloadLibrary();
  ASSERT_FALSE(IsLibraryLoadedByAnybody(library_1_));

  return;
}

TEST_F(ClassLoaderTest, testClassLoaderManager) {
  ClassLoaderManager loader_mgr;
  ASSERT_TRUE(loader_mgr.LoadLibrary(library_1_));
  ASSERT_TRUE(loader_mgr.LoadLibrary(library_2_));
  for (int i = 0; i < 2; ++i) {
    loader_mgr.CreateClassObj<Base>("Rect")->DoSomething();
    loader_mgr.CreateClassObj<Base>("Circle")->DoSomething();
    loader_mgr.CreateClassObj<Base>("Apple")->DoSomething();
  }

  auto pear_obj = loader_mgr.CreateClassObj<Base>("Pear", library_2_);
  EXPECT_TRUE(nullptr != pear_obj);
  pear_obj->DoSomething();

  auto null_obj = loader_mgr.CreateClassObj<Base>("Pear", library_1_);
  EXPECT_TRUE(nullptr == null_obj);
  auto null_obj1 = loader_mgr.CreateClassObj<Base>("ＣlassNull", "libNull.so");
  EXPECT_TRUE(nullptr == null_obj1);

  EXPECT_TRUE(loader_mgr.IsClassValid<Base>("Rect"));
  EXPECT_TRUE(loader_mgr.IsClassValid<Base>("Circle"));
  EXPECT_TRUE(loader_mgr.IsClassValid<Base>("Triangle"));
  EXPECT_TRUE(loader_mgr.IsClassValid<Base>("Star"));
  EXPECT_TRUE(loader_mgr.IsClassValid<Base>("Apple"));
  EXPECT_TRUE(loader_mgr.IsClassValid<Base>("Pear"));
  EXPECT_TRUE(loader_mgr.IsClassValid<Base>("Banana"));
  EXPECT_TRUE(loader_mgr.IsClassValid<Base>("Peach"));
  EXPECT_FALSE(loader_mgr.IsClassValid<Base>("Hamburger"));
  EXPECT_FALSE(loader_mgr.IsClassValid<Base>("Cake"));

  EXPECT_TRUE(loader_mgr.LoadLibrary(library_1_));
  SUCCEED();
}

TEST_F(ClassLoaderTest, createObjThreadSafety) {
  ClassLoaderManager loader_mgr;
  ASSERT_TRUE(loader_mgr.LoadLibrary(library_1_));
  ASSERT_TRUE(loader_mgr.IsLibraryValid(library_1_));

  std::vector<std::thread*> client_threads;

  for (unsigned int i = 0; i < 100; i++) {
    client_threads.emplace_back(new std::thread(
        std::bind(&ClassLoaderTest::CreateObj, this, &loader_mgr)));
  }

  for (unsigned int i = 0; i < client_threads.size(); i++) {
    client_threads[i]->join();
  }

  for (unsigned int i = 0; i < client_threads.size(); i++) {
    delete (client_threads[i]);
  }
}

TEST_F(ClassLoaderTest, loadLibThreadSafety) {
  ClassLoaderManager loaderMgr;
  std::vector<std::thread*> client_threads;

  for (unsigned int i = 0; i < 100; i++) {
    client_threads.emplace_back(new std::thread(
        std::bind(&ClassLoaderTest::LoadLib, this, &loaderMgr)));
  }

  for (unsigned int i = 0; i < client_threads.size(); i++) {
    client_threads[i]->join();
  }
  ASSERT_TRUE(loaderMgr.IsLibraryValid(library_1_));
  for (unsigned int i = 0; i < client_threads.size(); i++) {
    delete (client_threads[i]);
  }

  loaderMgr.UnloadAllLibrary();
  ASSERT_FALSE(loaderMgr.IsLibraryValid(library_1_));
}

TEST_F(ClassLoaderTest, invalidBaseClass) {
  ClassLoader loader1(library_1_);
  ASSERT_FALSE(loader1.IsClassValid<InvalidBaseClass>("InvalidBaseClass"));
  if (loader1.IsClassValid<InvalidBaseClass>("Rect")) {
    FAIL() << "Rect is invalid for InvalidBaseClass";
  } else if (loader1.IsClassValid<Base>("Rect")) {
    SUCCEED();
    return;
  } else {
    FAIL() << "baseClass not valid for Rect";
  }
}

}  // namespace class_loader
}  // namespace cyber
}  // namespace apollo

int main(int argc, char** argv) {
  apollo::cyber::Init(argv[0]);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
