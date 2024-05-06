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
#include "modules/perception/common/base/object_pool.h"

#include "modules/perception/common/base/light_object_pool.h"
#include "modules/perception/common/base/object.h"
#include "modules/perception/common/base/object_pool_types.h"

#include "gtest/gtest.h"

namespace apollo {
namespace perception {
namespace base {

TEST(ObjectPoolTest, basic_test) {
#ifndef PERCEPTION_BASE_DISABLE_POOL
  EXPECT_EQ(ObjectPool::Instance().RemainedNum(), kObjectPoolSize);
  EXPECT_EQ(PointFCloudPool::Instance().RemainedNum(), kPointCloudPoolSize);
  EXPECT_EQ(PointDCloudPool::Instance().RemainedNum(), kPointCloudPoolSize);
  EXPECT_EQ(FramePool::Instance().RemainedNum(), kFramePoolSize);
#endif
}

TEST(ObjectPoolTest, dummy_object_pool_test) {
  typedef DummyObjectPool<Object> TestObjectPool;
  std::shared_ptr<Object> obj = TestObjectPool::Instance().Get();
  EXPECT_NE(obj, nullptr);
  TestObjectPool::Instance().set_capacity(10);
  EXPECT_EQ(TestObjectPool::Instance().get_capacity(), 0);
  EXPECT_EQ(TestObjectPool::Instance().RemainedNum(), 0);

  obj->id = 0;
  // vector test
  std::vector<std::shared_ptr<Object>> objects_vector;
  objects_vector.push_back(obj);
  EXPECT_EQ(objects_vector.size(), 1);
  TestObjectPool::Instance().BatchGet(5, &objects_vector);
  EXPECT_EQ(objects_vector.size(), 6);
  EXPECT_EQ(objects_vector.front()->id, 0);
  for (size_t i = 1; i < 6; ++i) {
    EXPECT_EQ(objects_vector[i]->id, -1);
  }
  // list test
  std::list<std::shared_ptr<Object>> objects_list;
  objects_list.push_front(obj);
  EXPECT_EQ(objects_list.size(), 1);
  TestObjectPool::Instance().BatchGet(2, true, &objects_list);
  EXPECT_EQ(objects_list.size(), 3);
  TestObjectPool::Instance().BatchGet(2, false, &objects_list);
  EXPECT_EQ(objects_list.size(), 5);
  objects_list.pop_front();
  objects_list.pop_front();
  EXPECT_EQ(objects_list.front()->id, 0);
  // dequeue test
  std::deque<std::shared_ptr<Object>> objects_dequeue;
  objects_dequeue.push_front(obj);
  EXPECT_EQ(objects_dequeue.size(), 1);
  TestObjectPool::Instance().BatchGet(2, true, &objects_dequeue);
  EXPECT_EQ(objects_dequeue.size(), 3);
  TestObjectPool::Instance().BatchGet(2, false, &objects_dequeue);
  EXPECT_EQ(objects_dequeue.size(), 5);
  EXPECT_EQ(objects_dequeue[0]->id, -1);
  EXPECT_EQ(objects_dequeue[1]->id, -1);
  EXPECT_EQ(objects_dequeue[2]->id, 0);
  EXPECT_EQ(objects_dequeue[3]->id, -1);
  EXPECT_EQ(objects_dequeue[4]->id, -1);
}

TEST(ObjectPoolTest, concurrent_object_pool_capacity_test) {
#ifndef PERCEPTION_BASE_DISABLE_POOL
  typedef ConcurrentObjectPool<Object> TestObjectPool;
  size_t capacity = TestObjectPool::Instance().RemainedNum();
  TestObjectPool::Instance().set_capacity(capacity - 10);
  EXPECT_EQ(TestObjectPool::Instance().RemainedNum(), capacity);
  TestObjectPool::Instance().set_capacity(capacity + 10);
  EXPECT_EQ(TestObjectPool::Instance().RemainedNum(), capacity + 10);
#endif
}

TEST(ObjectPoolTest, concurrent_object_pool_get_test) {
  typedef ConcurrentObjectPool<Object> TestObjectPool;
  auto& instance = TestObjectPool::Instance();
  size_t size = instance.RemainedNum();
  std::vector<std::shared_ptr<Object>> memory;
  for (size_t i = 0; i < size; ++i) {
    memory.push_back(instance.Get());
  }
#ifndef PERCEPTION_BASE_DISABLE_POOL
  EXPECT_EQ(instance.RemainedNum(), 0);
#endif
  {
    std::shared_ptr<Object> obj = instance.Get();
    EXPECT_NE(obj, nullptr);
  }
#ifndef PERCEPTION_BASE_DISABLE_POOL
  EXPECT_GE(instance.RemainedNum(), 1);
#endif
}

TEST(ObjectPoolTest, concurrent_object_pool_batch_get_vec_test) {
  typedef ConcurrentObjectPool<Object> TestObjectPool;
  auto& instance = TestObjectPool::Instance();
#ifndef PERCEPTION_BASE_DISABLE_POOL
  size_t size = instance.RemainedNum();
  std::vector<std::shared_ptr<Object>> memory;
  for (size_t i = 0; i < size - 1; ++i) {
    memory.push_back(instance.Get());
  }
  EXPECT_EQ(instance.RemainedNum(), 1);
#endif
  std::shared_ptr<Object> obj = instance.Get();
  EXPECT_NE(obj, nullptr);
  obj->id = 0;
  // vector test
  {
    std::vector<std::shared_ptr<Object>> objects_vector;
    objects_vector.push_back(obj);
    EXPECT_EQ(objects_vector.size(), 1);
    TestObjectPool::Instance().BatchGet(5, &objects_vector);
    EXPECT_EQ(objects_vector.size(), 6);
    EXPECT_EQ(objects_vector.front()->id, 0);
    for (size_t i = 1; i < 6; ++i) {
      EXPECT_EQ(objects_vector[i]->id, -1);
    }
  }
#ifndef PERCEPTION_BASE_DISABLE_POOL
  EXPECT_GE(instance.RemainedNum(), 5);
#endif
}

TEST(ObjectPoolTest, concurrent_object_pool_batch_get_list_test) {
  typedef ConcurrentObjectPool<Object> TestObjectPool;
  auto& instance = TestObjectPool::Instance();
#ifndef PERCEPTION_BASE_DISABLE_POOL
  size_t size = instance.RemainedNum();
  std::vector<std::shared_ptr<Object>> memory;
  for (size_t i = 0; i < size - 1; ++i) {
    memory.push_back(instance.Get());
  }
  EXPECT_EQ(instance.RemainedNum(), 1);
#endif
  std::shared_ptr<Object> obj = instance.Get();
  EXPECT_NE(obj, nullptr);
  obj->id = 0;
  // list test
  {
    std::list<std::shared_ptr<Object>> objects_list;
    objects_list.push_front(obj);
    EXPECT_EQ(objects_list.size(), 1);
    TestObjectPool::Instance().BatchGet(2, true, &objects_list);
    EXPECT_EQ(objects_list.size(), 3);
    TestObjectPool::Instance().BatchGet(2, false, &objects_list);
    EXPECT_EQ(objects_list.size(), 5);
    objects_list.pop_front();
    objects_list.pop_front();
    EXPECT_EQ(objects_list.front()->id, 0);
  }
#ifndef PERCEPTION_BASE_DISABLE_POOL
  EXPECT_GE(instance.RemainedNum(), 4);
#endif
}

TEST(ObjectPoolTest, concurrent_object_pool_batch_get_deque_test) {
  typedef ConcurrentObjectPool<Object> TestObjectPool;
  auto& instance = TestObjectPool::Instance();
#ifndef PERCEPTION_BASE_DISABLE_POOL
  size_t size = instance.RemainedNum();
  std::vector<std::shared_ptr<Object>> memory;
  for (size_t i = 0; i < size - 1; ++i) {
    memory.push_back(instance.Get());
  }
  EXPECT_EQ(instance.RemainedNum(), 1);
#endif
  std::shared_ptr<Object> obj = instance.Get();
  EXPECT_NE(obj, nullptr);
  obj->id = 0;
  // dequeue test
  {
    std::deque<std::shared_ptr<Object>> objects_dequeue;
    objects_dequeue.push_front(obj);
    EXPECT_EQ(objects_dequeue.size(), 1);
    TestObjectPool::Instance().BatchGet(2, true, &objects_dequeue);
    EXPECT_EQ(objects_dequeue.size(), 3);
    TestObjectPool::Instance().BatchGet(2, false, &objects_dequeue);
    EXPECT_EQ(objects_dequeue.size(), 5);
    EXPECT_EQ(objects_dequeue[0]->id, -1);
    EXPECT_EQ(objects_dequeue[1]->id, -1);
    EXPECT_EQ(objects_dequeue[2]->id, 0);
    EXPECT_EQ(objects_dequeue[3]->id, -1);
    EXPECT_EQ(objects_dequeue[4]->id, -1);
  }
#ifndef PERCEPTION_BASE_DISABLE_POOL
  EXPECT_GE(instance.RemainedNum(), 4);
#endif
}

#ifndef PERCEPTION_BASE_DISABLE_POOL
TEST(ObjectPoolTest, concurrent_object_pool_constructor_test) {
  typedef ConcurrentObjectPool<Object, 10> TestObjectPool;
  auto& pool = TestObjectPool::Instance();
  EXPECT_EQ(pool.RemainedNum(), 10);
  EXPECT_EQ(pool.get_capacity(), 10);
}
#endif

struct TestObjectPoolInitializer {
  void operator()(Object* t) const { t->id = 1; }
};

TEST(ObjectPoolTest, concurrent_object_pool_initializer_test) {
#ifndef PERCEPTION_BASE_DISABLE_POOL
  {
    typedef ConcurrentObjectPool<Object, 10, TestObjectPoolInitializer>
        TestObjectPool;
    std::shared_ptr<Object> ptr = TestObjectPool::Instance().Get();
    EXPECT_EQ(ptr->id, 1);
    {
      std::vector<std::shared_ptr<Object>> object_vec;
      TestObjectPool::Instance().BatchGet(2, &object_vec);
      for (auto& ptr : object_vec) {
        EXPECT_EQ(ptr->id, 1);
      }
    }
    {
      std::list<std::shared_ptr<Object>> object_list;
      TestObjectPool::Instance().BatchGet(2, true, &object_list);
      TestObjectPool::Instance().BatchGet(2, false, &object_list);
      for (auto& ptr : object_list) {
        EXPECT_EQ(ptr->id, 1);
      }
    }
    {
      std::deque<std::shared_ptr<Object>> object_deque;
      TestObjectPool::Instance().BatchGet(2, true, &object_deque);
      TestObjectPool::Instance().BatchGet(2, false, &object_deque);
      for (auto& ptr : object_deque) {
        EXPECT_EQ(ptr->id, 1);
      }
    }
  }
#endif
  {
    typedef ConcurrentObjectPool<Object> TestObjectPool;
    std::shared_ptr<Object> ptr = TestObjectPool::Instance().Get();
    EXPECT_EQ(ptr->id, -1);
    {
      std::vector<std::shared_ptr<Object>> object_vec;
      TestObjectPool::Instance().BatchGet(2, &object_vec);
      for (auto& ptr : object_vec) {
        EXPECT_EQ(ptr->id, -1);
      }
    }
    {
      std::list<std::shared_ptr<Object>> object_list;
      TestObjectPool::Instance().BatchGet(2, true, &object_list);
      TestObjectPool::Instance().BatchGet(2, false, &object_list);
      for (auto& ptr : object_list) {
        EXPECT_EQ(ptr->id, -1);
      }
    }
    {
      std::deque<std::shared_ptr<Object>> object_deque;
      TestObjectPool::Instance().BatchGet(2, true, &object_deque);
      TestObjectPool::Instance().BatchGet(2, false, &object_deque);
      for (auto& ptr : object_deque) {
        EXPECT_EQ(ptr->id, -1);
      }
    }
  }
}

TEST(ObjectPoolTest, light_object_pool_capacity_test) {
  typedef LightObjectPool<Object, kPoolDefaultSize, TestObjectPoolInitializer,
                          SensorType::UNKNOWN_SENSOR_TYPE>
      TestObjectPool;

  size_t capacity = TestObjectPool::Instance().RemainedNum();
  TestObjectPool::Instance().set_capacity(capacity - 10);
  EXPECT_EQ(TestObjectPool::Instance().RemainedNum(), capacity);
  TestObjectPool::Instance().set_capacity(capacity + 10);
  EXPECT_EQ(TestObjectPool::Instance().RemainedNum(), capacity + 10);
}

TEST(ObjectPoolTest, light_object_pool_get_test) {
  typedef LightObjectPool<Object> TestObjectPool;
  auto& instance = TestObjectPool::Instance();
  size_t size = instance.RemainedNum();
  std::vector<std::shared_ptr<Object>> memory;
  for (size_t i = 0; i < size; ++i) {
    memory.push_back(instance.Get());
  }
  EXPECT_EQ(instance.RemainedNum(), 0);
  {
    std::shared_ptr<Object> obj = instance.Get();
    EXPECT_NE(obj, nullptr);
  }
  EXPECT_GE(instance.RemainedNum(), 1);
}

TEST(ObjectPoolTest, light_object_pool_batch_get_vec_test) {
  typedef LightObjectPool<Object> TestObjectPool;
  auto& instance = TestObjectPool::Instance();
  size_t size = instance.RemainedNum();
  std::vector<std::shared_ptr<Object>> memory;
  for (size_t i = 0; i < size - 1; ++i) {
    memory.push_back(instance.Get());
  }
  EXPECT_EQ(instance.RemainedNum(), 1);
  std::shared_ptr<Object> obj = instance.Get();
  EXPECT_NE(obj, nullptr);
  obj->id = 0;
  // vector test
  {
    std::vector<std::shared_ptr<Object>> objects_vector;
    objects_vector.push_back(obj);
    EXPECT_EQ(objects_vector.size(), 1);
    TestObjectPool::Instance().BatchGet(5, &objects_vector);
    EXPECT_EQ(objects_vector.size(), 6);
    EXPECT_EQ(objects_vector.front()->id, 0);
    for (size_t i = 1; i < 6; ++i) {
      EXPECT_EQ(objects_vector[i]->id, -1);
    }
  }
  EXPECT_GE(instance.RemainedNum(), 5);
}

TEST(ObjectPoolTest, light_object_pool_batch_get_list_test) {
  typedef LightObjectPool<Object> TestObjectPool;
  auto& instance = TestObjectPool::Instance();
  size_t size = instance.RemainedNum();
  std::vector<std::shared_ptr<Object>> memory;
  for (size_t i = 0; i < size - 1; ++i) {
    memory.push_back(instance.Get());
  }
  EXPECT_EQ(instance.RemainedNum(), 1);
  std::shared_ptr<Object> obj = instance.Get();
  EXPECT_NE(obj, nullptr);
  obj->id = 0;
  // list test
  {
    std::list<std::shared_ptr<Object>> objects_list;
    objects_list.push_front(obj);
    EXPECT_EQ(objects_list.size(), 1);
    TestObjectPool::Instance().BatchGet(2, true, &objects_list);
    EXPECT_EQ(objects_list.size(), 3);
    TestObjectPool::Instance().BatchGet(2, false, &objects_list);
    EXPECT_EQ(objects_list.size(), 5);
    objects_list.pop_front();
    objects_list.pop_front();
    EXPECT_EQ(objects_list.front()->id, 0);
  }
  EXPECT_GE(instance.RemainedNum(), 4);
}

TEST(ObjectPoolTest, light_object_pool_batch_get_deque_test) {
  typedef LightObjectPool<Object> TestObjectPool;
  auto& instance = TestObjectPool::Instance();
  size_t size = instance.RemainedNum();
  std::vector<std::shared_ptr<Object>> memory;
  for (size_t i = 0; i < size - 1; ++i) {
    memory.push_back(instance.Get());
  }
  EXPECT_EQ(instance.RemainedNum(), 1);
  std::shared_ptr<Object> obj = instance.Get();
  EXPECT_NE(obj, nullptr);
  obj->id = 0;
  // dequeue test
  {
    std::deque<std::shared_ptr<Object>> objects_dequeue;
    objects_dequeue.push_front(obj);
    EXPECT_EQ(objects_dequeue.size(), 1);
    TestObjectPool::Instance().BatchGet(2, true, &objects_dequeue);
    EXPECT_EQ(objects_dequeue.size(), 3);
    TestObjectPool::Instance().BatchGet(2, false, &objects_dequeue);
    EXPECT_EQ(objects_dequeue.size(), 5);
    EXPECT_EQ(objects_dequeue[0]->id, -1);
    EXPECT_EQ(objects_dequeue[1]->id, -1);
    EXPECT_EQ(objects_dequeue[2]->id, 0);
    EXPECT_EQ(objects_dequeue[3]->id, -1);
    EXPECT_EQ(objects_dequeue[4]->id, -1);
  }
  EXPECT_GE(instance.RemainedNum(), 4);
}

TEST(ObjectPoolTest, light_object_pool_constructor_test) {
  typedef LightObjectPool<Object, 10> TestObjectPool;
  auto& pool = TestObjectPool::Instance();
  EXPECT_EQ(pool.RemainedNum(), 10);
  EXPECT_EQ(pool.get_capacity(), 10);
}

TEST(ObjectPoolTest, light_object_pool_initializer_test) {
  {
    typedef LightObjectPool<Object, 10, TestObjectPoolInitializer>
        TestObjectPool;
    std::shared_ptr<Object> ptr = TestObjectPool::Instance().Get();
    EXPECT_EQ(ptr->id, 1);
    {
      std::vector<std::shared_ptr<Object>> object_vec;
      TestObjectPool::Instance().BatchGet(2, &object_vec);
      for (auto& ptr : object_vec) {
        EXPECT_EQ(ptr->id, 1);
      }
    }
    {
      std::list<std::shared_ptr<Object>> object_list;
      TestObjectPool::Instance().BatchGet(2, true, &object_list);
      TestObjectPool::Instance().BatchGet(2, false, &object_list);
      for (auto& ptr : object_list) {
        EXPECT_EQ(ptr->id, 1);
      }
    }
    {
      std::deque<std::shared_ptr<Object>> object_deque;
      TestObjectPool::Instance().BatchGet(2, true, &object_deque);
      TestObjectPool::Instance().BatchGet(2, false, &object_deque);
      for (auto& ptr : object_deque) {
        EXPECT_EQ(ptr->id, 1);
      }
    }
  }
  {
    typedef LightObjectPool<Object> TestObjectPool;
    std::shared_ptr<Object> ptr = TestObjectPool::Instance().Get();
    EXPECT_EQ(ptr->id, -1);
    {
      std::vector<std::shared_ptr<Object>> object_vec;
      TestObjectPool::Instance().BatchGet(2, &object_vec);
      for (auto& ptr : object_vec) {
        EXPECT_EQ(ptr->id, -1);
      }
    }
    {
      std::list<std::shared_ptr<Object>> object_list;
      TestObjectPool::Instance().BatchGet(2, true, &object_list);
      TestObjectPool::Instance().BatchGet(2, false, &object_list);
      for (auto& ptr : object_list) {
        EXPECT_EQ(ptr->id, -1);
      }
    }
    {
      std::deque<std::shared_ptr<Object>> object_deque;
      TestObjectPool::Instance().BatchGet(2, true, &object_deque);
      TestObjectPool::Instance().BatchGet(2, false, &object_deque);
      for (auto& ptr : object_deque) {
        EXPECT_EQ(ptr->id, -1);
      }
    }
  }
}

}  // namespace base
}  // namespace perception
}  // namespace apollo
