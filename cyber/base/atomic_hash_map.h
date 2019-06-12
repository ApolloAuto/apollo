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

#ifndef CYBER_BASE_ATOMIC_HASH_MAP_H_
#define CYBER_BASE_ATOMIC_HASH_MAP_H_

#include <stdint.h>
#include <atomic>
#include <type_traits>
#include <utility>

namespace apollo {
namespace cyber {
namespace base {
/**
 * @brief A implementation of lock-free fixed size hash map
 *
 * @tparam K Type of key, must be integral
 * @tparam V Type of value
 * @tparam 128 Size of hash table
 * @tparam 0 Type traits, use for checking types of key & value
 */
template <typename K, typename V, std::size_t TableSize = 128,
          typename std::enable_if<std::is_integral<K>::value &&
                                      (TableSize & (TableSize - 1)) == 0,
                                  int>::type = 0>
class AtomicHashMap {
 public:
  AtomicHashMap() : capacity_(TableSize), mode_num_(capacity_ - 1) {}
  AtomicHashMap(const AtomicHashMap& other) = delete;
  AtomicHashMap& operator=(const AtomicHashMap& other) = delete;

  bool Has(K key) {
    uint64_t index = key & mode_num_;
    return table_[index].Has(key);
  }

  /**
   * @brief Get value with the key
   *
   * @param key The key to be found
   * @param value If key exists, value point to the address of the value stored
   * in map
   * @return true If the key exists
   * @return false If the key does not exists
   *
   * @note This function is not thread safe. If one thread set the key with new
   * value, the address will be deleted. This function can avoid copy the value,
   * and it is faster in most cases. Use it carefully.
   */
  bool Get(K key, V** value) {
    uint64_t index = key & mode_num_;
    ValueNode* val = nullptr;
    bool res = table_[index].Get(key, &val);
    if (res) {
      *value = &(val->value);
      val->Release();
    }
    return res;
  }

  /**
   * @brief Get value with the key
   *
   * @param key The key to be found
   * @param value If key exists, value pointer to a copy of the value stored in
   * map
   * @return true If the key exists
   * @return false If the key does not exists
   */
  bool Get(K key, V* value) {
    uint64_t index = key & mode_num_;
    ValueNode* val = nullptr;
    bool res = table_[index].Get(key, &val);
    if (res) {
      *value = val->value;
      val->Release();
    }
    return res;
  }

  void Set(K key) {
    uint64_t index = key & mode_num_;
    table_[index].Insert(key);
  }

  void Set(K key, const V& value) {
    uint64_t index = key & mode_num_;
    table_[index].Insert(key, value);
  }

  void Set(K key, V&& value) {
    uint64_t index = key & mode_num_;
    table_[index].Insert(key, std::forward<V>(value));
  }

 private:
  struct ValueNode {
    ValueNode() {}
    explicit ValueNode(const V& value) : value(value) {}
    explicit ValueNode(V&& value) : value(std::forward<V>(value)) {}
    V value;
    std::atomic<uint32_t> ref_count = {1};
    void Release() {
      if (ref_count.fetch_sub(1) == 1) {
        delete this;
      }
    }
  };
  struct Entry {
    Entry() {}
    explicit Entry(K key) : key(key) {
      value_ptr.store(new ValueNode(), std::memory_order_release);
    }
    Entry(K key, const V& value) : key(key) {
      value_ptr.store(new ValueNode(value), std::memory_order_release);
    }
    Entry(K key, V&& value) : key(key) {
      value_ptr.store(new V(std::forward<V>(value)), std::memory_order_release);
    }
    ~Entry() { delete value_ptr.load(std::memory_order_acquire); }

    K key = 0;
    std::atomic<ValueNode*> value_ptr = {nullptr};
    std::atomic<Entry*> next = {nullptr};
  };

  class Bucket {
   public:
    Bucket() : head_(new Entry()) {}
    ~Bucket() {
      Entry* ite = head_;
      while (ite) {
        auto tmp = ite->next.load(std::memory_order_acquire);
        delete ite;
        ite = tmp;
      }
    }

    bool Has(K key) {
      Entry* m_target = head_->next.load(std::memory_order_acquire);
      while (Entry* target = m_target) {
        if (target->key < key) {
          m_target = target->next.load(std::memory_order_acquire);
          continue;
        } else {
          return target->key == key;
        }
      }
      return false;
    }

    bool Find(K key, Entry** prev_ptr, Entry** target_ptr) {
      Entry* prev = head_;
      Entry* m_target = head_->next.load(std::memory_order_acquire);
      while (Entry* target = m_target) {
        if (target->key == key) {
          *prev_ptr = prev;
          *target_ptr = target;
          return true;
        } else if (target->key > key) {
          *prev_ptr = prev;
          *target_ptr = target;
          return false;
        } else {
          prev = target;
          m_target = target->next.load(std::memory_order_acquire);
        }
      }
      *prev_ptr = prev;
      *target_ptr = nullptr;
      return false;
    }

    void Insert(K key, const V& value) {
      Entry* prev = nullptr;
      Entry* target = nullptr;
      Entry* new_entry = nullptr;
      ValueNode* new_value = nullptr;
      while (true) {
        if (Find(key, &prev, &target)) {
          // key exists, update value
          if (!new_value) {
            new_value = new ValueNode(value);
          }
          auto old_val_ptr = target->value_ptr.load(std::memory_order_acquire);
          while (!target->value_ptr.compare_exchange_weak(
              old_val_ptr, new_value, std::memory_order_acq_rel,
              std::memory_order_relaxed)) {
          }
          old_val_ptr->Release();
          if (new_entry) {
            delete new_entry;
            new_entry = nullptr;
          }
          return;
        } else {
          if (!new_entry) {
            new_entry = new Entry(key, value);
          }
          new_entry->next.store(target, std::memory_order_release);
          if (prev->next.compare_exchange_strong(target, new_entry,
                                                 std::memory_order_acq_rel,
                                                 std::memory_order_relaxed)) {
            // Insert success
            if (new_value) {
              delete new_value;
              new_value = nullptr;
            }
            return;
          }
          // another entry has been inserted, retry
        }
      }
    }

    void Insert(K key, V&& value) {
      Entry* prev = nullptr;
      Entry* target = nullptr;
      Entry* new_entry = nullptr;
      ValueNode* new_value = nullptr;
      while (true) {
        if (Find(key, &prev, &target)) {
          // key exists, update value
          if (!new_value) {
            new_value = new ValueNode(std::forward<V>(value));
          }
          auto old_val_ptr = target->value_ptr.load(std::memory_order_acquire);
          while (!target->value_ptr.compare_exchange_weak(
              old_val_ptr, new_value, std::memory_order_acq_rel,
              std::memory_order_relaxed)) {
          }
          old_val_ptr->Release();
          if (new_entry) {
            delete new_entry;
            new_entry = nullptr;
          }
          return;
        } else {
          if (!new_entry) {
            new_entry = new Entry(key, value);
          }
          new_entry->next.store(target, std::memory_order_release);
          if (prev->next.compare_exchange_strong(target, new_entry,
                                                 std::memory_order_acq_rel,
                                                 std::memory_order_relaxed)) {
            // Insert success
            if (new_value) {
              delete new_value;
              new_value = nullptr;
            }
            return;
          }
          // another entry has been inserted, retry
        }
      }
    }

    void Insert(K key) {
      Entry* prev = nullptr;
      Entry* target = nullptr;
      Entry* new_entry = nullptr;
      ValueNode* new_value = nullptr;
      while (true) {
        if (Find(key, &prev, &target)) {
          // key exists, update value
          if (!new_value) {
            new_value = new ValueNode();
          }
          auto old_val_ptr = target->value_ptr.load(std::memory_order_acquire);
          while (!target->value_ptr.compare_exchange_weak(
              old_val_ptr, new_value, std::memory_order_acq_rel,
              std::memory_order_relaxed)) {
          }
          old_val_ptr->Release();
          if (new_entry) {
            delete new_entry;
            new_entry = nullptr;
          }
          return;
        } else {
          if (!new_entry) {
            new_entry = new Entry(key);
          }
          new_entry->next.store(target, std::memory_order_release);
          if (prev->next.compare_exchange_strong(target, new_entry,
                                                 std::memory_order_acq_rel,
                                                 std::memory_order_relaxed)) {
            // Insert success
            if (new_value) {
              delete new_value;
              new_value = nullptr;
            }
            return;
          }
          // another entry has been inserted, retry
        }
      }
    }

    bool Get(K key, ValueNode** value) {
      Entry* prev = nullptr;
      Entry* target = nullptr;
      if (Find(key, &prev, &target)) {
        *value = target->value_ptr.load(std::memory_order_acquire);
        ValueNode* value_node = nullptr;
        do {
          if (value_node) {
            value_node->Release();
          }
          (*value)->ref_count.fetch_add(1, std::memory_order_release);
          value_node = *value;
        } while (!target->value_ptr.compare_exchange_weak(
            value_node, value_node, std::memory_order_acq_rel,
            std::memory_order_relaxed));

        return true;
      }
      return false;
    }

    Entry* head_;
  };

 private:
  Bucket table_[TableSize];
  uint64_t capacity_;
  uint64_t mode_num_;
};

}  // namespace base
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_BASE_ATOMIC_HASH_MAP_H_
