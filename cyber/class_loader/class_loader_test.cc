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
#include "cyber/class_loader/class_loader.h"

#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"

#include "cyber/class_loader/class_loader_manager.h"
#include "cyber/class_loader/test/base.h"
#include "cyber/cyber.h"

const char LIBRARY_1[] =
    "/apollo/bazel-bin/cyber/class_loader/test/libplugin1.so";
const char LIBRARY_2[] =
    "/apollo/bazel-bin/cyber/class_loader/test/libplugin2.so";
using apollo::cyber::class_loader::ClassLoader;
using apollo::cyber::class_loader::ClassLoaderManager;
using apollo::cyber::class_loader::utility::IsLibraryLoadedByAnybody;

TEST(ClassLoaderTest, createClassObj) {
  ClassLoader loader1(LIBRARY_1);
  EXPECT_EQ(LIBRARY_1, loader1.GetLibraryPath());
  auto rect_obj = loader1.CreateClassObj<Base>("Rect");
  EXPECT_NE(nullptr, rect_obj);
  rect_obj->DoSomething();
  EXPECT_EQ(nullptr, loader1.CreateClassObj<Base>("Xeee"));

  SUCCEED();
}

TEST(ClassLoaderTest, loadLibCounts) {
  ClassLoader loader1(LIBRARY_1);
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

TEST(ClassLoaderTest, multiTimesLoadunload) {
  ClassLoader loader1(LIBRARY_1);
  ASSERT_TRUE(loader1.LoadLibrary());
  loader1.LoadLibrary();
  ASSERT_TRUE(IsLibraryLoadedByAnybody(LIBRARY_1));
  loader1.UnloadLibrary();
  ASSERT_TRUE(IsLibraryLoadedByAnybody(LIBRARY_1));
  loader1.UnloadLibrary();
  ASSERT_TRUE(IsLibraryLoadedByAnybody(LIBRARY_1));
  loader1.UnloadLibrary();
  ASSERT_FALSE(IsLibraryLoadedByAnybody(LIBRARY_1));
}

TEST(ClassLoaderManagerTest, testClassLoaderManager) {
  ClassLoaderManager loader_mgr;
  ASSERT_TRUE(loader_mgr.LoadLibrary(LIBRARY_1));
  ASSERT_TRUE(loader_mgr.LoadLibrary(LIBRARY_2));
  for (int i = 0; i < 2; ++i) {
    loader_mgr.CreateClassObj<Base>("Rect")->DoSomething();
    loader_mgr.CreateClassObj<Base>("Circle")->DoSomething();
    loader_mgr.CreateClassObj<Base>("Apple")->DoSomething();
  }

  auto pear_obj = loader_mgr.CreateClassObj<Base>("Pear", LIBRARY_2);
  EXPECT_NE(nullptr, pear_obj);
  pear_obj->DoSomething();

  auto null_obj = loader_mgr.CreateClassObj<Base>("Pear", LIBRARY_1);
  EXPECT_EQ(nullptr, null_obj);
  auto null_obj1 = loader_mgr.CreateClassObj<Base>("ClassNull", "libNull.so");
  EXPECT_EQ(nullptr, null_obj1);

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

  EXPECT_TRUE(loader_mgr.LoadLibrary(LIBRARY_1));
  SUCCEED();
}

void CreateObj(ClassLoaderManager* loader) {
  std::vector<std::string> classes = loader->GetValidClassNames<Base>();
  for (unsigned int i = 0; i < classes.size(); i++) {
    loader->CreateClassObj<Base>(classes[i])->DoSomething();
  }
}

TEST(ClassLoaderTest, createObjThreadSafety) {
  ClassLoaderManager loader_mgr;
  ASSERT_TRUE(loader_mgr.LoadLibrary(LIBRARY_1));
  ASSERT_TRUE(loader_mgr.IsLibraryValid(LIBRARY_1));

  std::vector<std::thread*> client_threads;

  for (unsigned int i = 0; i < 100; i++) {
    client_threads.emplace_back(
        new std::thread(std::bind(&CreateObj, &loader_mgr)));
  }

  for (unsigned int i = 0; i < client_threads.size(); i++) {
    client_threads[i]->join();
  }

  for (unsigned int i = 0; i < client_threads.size(); i++) {
    delete (client_threads[i]);
  }
}

void LoadLib(ClassLoaderManager* loaderMgr) {
  loaderMgr->LoadLibrary(LIBRARY_1);
  ASSERT_TRUE(loaderMgr->IsLibraryValid(LIBRARY_1));
}

TEST(ClassLoaderTest, loadLibThreadSafety) {
  ClassLoaderManager loaderMgr;
  std::vector<std::thread*> client_threads;

  for (unsigned int i = 0; i < 100; i++) {
    client_threads.emplace_back(
        new std::thread(std::bind(&LoadLib, &loaderMgr)));
  }

  for (unsigned int i = 0; i < client_threads.size(); i++) {
    client_threads[i]->join();
  }
  ASSERT_TRUE(loaderMgr.IsLibraryValid(LIBRARY_1));
  for (unsigned int i = 0; i < client_threads.size(); i++) {
    delete (client_threads[i]);
  }

  loaderMgr.UnloadAllLibrary();
  ASSERT_FALSE(loaderMgr.IsLibraryValid(LIBRARY_1));
}

class InvalidBaseClass {};

TEST(ClassLoaderTest, util_test) {
  ClassLoader loader1(LIBRARY_1);
  apollo::cyber::class_loader::utility::LoadLibrary("1", &loader1);
  apollo::cyber::class_loader::utility::UnloadLibrary("1", nullptr);
  apollo::cyber::class_loader::utility::IsLibraryLoaded(LIBRARY_1, &loader1);
  apollo::cyber::class_loader::utility::IsLibraryLoaded(LIBRARY_2, &loader1);
}

int main(int argc, char** argv) {
  apollo::cyber::Init(argv[0]);
  testing::InitGoogleTest(&argc, argv);
  const int output = RUN_ALL_TESTS();
  google::protobuf::ShutdownProtobufLibrary();
  return output;
}
