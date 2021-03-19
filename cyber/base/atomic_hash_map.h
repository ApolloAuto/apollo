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

#include <atomic>
#include <cstdint>
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
  AtomicHashMap(const AtomicHashMap &other) = delete;
  AtomicHashMap &operator=(const AtomicHashMap &other) = delete;

  bool Has(K key) {
    uint64_t index = key & mode_num_;
    return table_[index].Has(key);
  }

  bool Get(K key, V **value) {
    uint64_t index = key & mode_num_;
    return table_[index].Get(key, value);
  }

  bool Get(K key, V *value) {
    uint64_t index = key & mode_num_;
    V *val = nullptr;
    bool res = table_[index].Get(key, &val);
    if (res) {
      *value = *val;
    }
    return res;
  }

  void Set(K key) {
    uint64_t index = key & mode_num_;
    table_[index].Insert(key);
  }

  void Set(K key, const V &value) {
    uint64_t index = key & mode_num_;
    table_[index].Insert(key, value);
  }

  void Set(K key, V &&value) {
    uint64_t index = key & mode_num_;
    table_[index].Insert(key, std::forward<V>(value));
  }

 private:
  struct Entry {
    Entry() {}
    explicit Entry(K key) : key(key) {
      value_ptr.store(new V(), std::memory_order_release);
    }
    Entry(K key, const V &value) : key(key) {
      value_ptr.store(new V(value), std::memory_order_release);
    }
    Entry(K key, V &&value) : key(key) {
      value_ptr.store(new V(std::forward<V>(value)), std::memory_order_release);
    }
    ~Entry() { delete value_ptr.load(std::memory_order_acquire); }

    K key = 0;
    std::atomic<V *> value_ptr = {nullptr};
    std::atomic<Entry *> next = {nullptr};
  };

  class Bucket {
   public:
    Bucket() : head_(new Entry()) {}
    ~Bucket() {
      Entry *ite = head_;
      while (ite) {
        auto tmp = ite->next.load(std::memory_order_acquire);
        delete ite;
        ite = tmp;
      }
    }

    bool Has(K key) {
      Entry *m_target = head_->next.load(std::memory_order_acquire);
      while (Entry *target = m_target) {
        if (target->key < key) {
          m_target = target->next.load(std::memory_order_acquire);
          continue;
        } else {
          return target->key == key;
        }
      }
      return false;
    }

    bool Find(K key, Entry **prev_ptr, Entry **target_ptr) {
      Entry *prev = head_;
      Entry *m_target = head_->next.load(std::memory_order_acquire);
      while (Entry *target = m_target) {
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

    void Insert(K key, const V &value) {
      Entry *prev = nullptr;
      Entry *target = nullptr;
      Entry *new_entry = nullptr;
      V *new_value = nullptr;
      while (true) {
        if (Find(key, &prev, &target)) {
          // key exists, update value
          if (!new_value) {
            new_value = new V(value);
          }
          auto old_val_ptr = target->value_ptr.load(std::memory_order_acquire);
          if (target->value_ptr.compare_exchange_strong(
                  old_val_ptr, new_value, std::memory_order_acq_rel,
                  std::memory_order_relaxed)) {
            delete old_val_ptr;
            if (new_entry) {
              delete new_entry;
              new_entry = nullptr;
            }
            return;
          }
          continue;
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

    void Insert(K key, V &&value) {
      Entry *prev = nullptr;
      Entry *target = nullptr;
      Entry *new_entry = nullptr;
      V *new_value = nullptr;
      while (true) {
        if (Find(key, &prev, &target)) {
          // key exists, update value
          if (!new_value) {
            new_value = new V(std::forward<V>(value));
          }
          auto old_val_ptr = target->value_ptr.load(std::memory_order_acquire);
          if (target->value_ptr.compare_exchange_strong(
                  old_val_ptr, new_value, std::memory_order_acq_rel,
                  std::memory_order_relaxed)) {
            delete old_val_ptr;
            if (new_entry) {
              delete new_entry;
              new_entry = nullptr;
            }
            return;
          }
          continue;
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
      Entry *prev = nullptr;
      Entry *target = nullptr;
      Entry *new_entry = nullptr;
      V *new_value = nullptr;
      while (true) {
        if (Find(key, &prev, &target)) {
          // key exists, update value
          if (!new_value) {
            new_value = new V();
          }
          auto old_val_ptr = target->value_ptr.load(std::memory_order_acquire);
          if (target->value_ptr.compare_exchange_strong(
                  old_val_ptr, new_value, std::memory_order_acq_rel,
                  std::memory_order_relaxed)) {
            delete old_val_ptr;
            if (new_entry) {
              delete new_entry;
              new_entry = nullptr;
            }
            return;
          }
          continue;
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

    bool Get(K key, V **value) {
      Entry *prev = nullptr;
      Entry *target = nullptr;
      if (Find(key, &prev, &target)) {
        *value = target->value_ptr.load(std::memory_order_acquire);
        return true;
      }
      return false;
    }

    Entry *head_;
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
