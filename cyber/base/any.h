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

#ifndef CYBER_BASE_ANY_H_
#define CYBER_BASE_ANY_H_

#include <memory>
#include <typeinfo>
#include <utility>

namespace apollo {
namespace cyber {
namespace data {

class BadAnyCast : public std::bad_cast {
 public:
  const char* what() const noexcept override { return "AnyCast Exception"; }
};

class Any final {
 public:
  Any();
  Any(const Any& any);
  Any(Any&& any);
  template <typename ValueType>
  explicit Any(const ValueType& value);
  ~Any();

  Any& operator=(const Any& rhs);

  bool Empty() const;
  template <typename ValueType>
  const ValueType* GetValue() const;
  template <typename ValueType>
  ValueType* GetValue();

 private:
  using StackValue = typename std::aligned_storage<2 * sizeof(void*)>::type;
  template <typename T>
  struct IsSharedPtr : std::false_type {};
  template <typename T>
  struct IsSharedPtr<std::shared_ptr<T>> : std::true_type {};

  template <typename ValueType, typename T>
  typename std::enable_if<IsSharedPtr<T>::value>::type Construct(
      const ValueType& value) {
    storage_ = new (&stack_) Storage<ValueType>(value);
  }

  template <typename ValueType, typename T>
  typename std::enable_if<!IsSharedPtr<T>::value>::type Construct(
      const ValueType& value) {
    storage_ = new Storage<ValueType>(value);
  }

  class AnyStorage {
   public:
    virtual ~AnyStorage() {}
    virtual AnyStorage* Clone(void*) const = 0;
    virtual void Destroy() = 0;
    virtual bool IsValid(const std::type_info& type) const = 0;
  };

  template <typename ValueType>
  class Storage : public AnyStorage {
   public:
    explicit Storage(ValueType&& value) : value_(std::move(value)) {}
    explicit Storage(const ValueType& value) : value_(value) {}
    virtual ~Storage() {}

    Storage* Clone(void* stack) const override {
      if (IsSharedPtr<std::decay<ValueType>>::value) {
        return new (stack) Storage<ValueType>(*this);
      } else {
        return new Storage<ValueType>(*this);
      }
    }

    // TODO(hewei03): Template specialization
    void Destroy() override {
      reinterpret_cast<ValueType*>(this)->~ValueType();
    }

    bool IsValid(const std::type_info& type) const override {
      return typeid(ValueType) == type;
    }

    ValueType value_;
  };

  // TODO(hewei03): It is a waste of memory.
  AnyStorage* storage_;
  StackValue stack_;
};

inline Any::Any() : storage_(nullptr) {}

inline Any::Any(const Any& any) {
  if (any.Empty()) {
    storage_ = nullptr;
  } else {
    storage_ = any.storage_->Clone(&stack_);
  }
}

inline Any::Any(Any&& any) {
  if (!any.Empty()) {
    storage_ = any.storage_;
    any.storage_ = nullptr;
  } else {
    storage_ = nullptr;
  }
}

template <typename ValueType>
inline Any::Any(const ValueType& value) {
  using T = typename std::decay<ValueType>::type;
  Construct<ValueType, T>(value);
}

inline Any::~Any() {
  if (reinterpret_cast<void*>(storage_) == &stack_) {
    storage_->Destroy();
    storage_ = nullptr;
  } else {
    delete storage_;
  }
}

inline Any& Any::operator=(const Any& rhs) {
  if (this != &rhs && !rhs.Empty()) {
    AnyStorage* tmp = rhs.storage_->Clone(&stack_);
    if (reinterpret_cast<void*>(storage_) == &stack_) {
      storage_->Destroy();
      storage_ = nullptr;
    } else {
      delete storage_;
    }
    storage_ = tmp;
  }
  return *this;
}

inline bool Any::Empty() const { return storage_ == nullptr; }

template <typename ValueType>
inline const ValueType* Any::GetValue() const {
  if (storage_ && storage_->IsValid(typeid(ValueType))) {
    return &(dynamic_cast<Storage<ValueType>*>(storage_)->value_);
  } else {
    return nullptr;
  }
}

template <typename ValueType>
inline ValueType* Any::GetValue() {
  if (storage_ && storage_->IsValid(typeid(ValueType))) {
    return &(dynamic_cast<Storage<ValueType>*>(storage_)->value_);
  } else {
    return nullptr;
  }
}

// public interface
template <typename ValueType>
inline const ValueType* AnyCast(const Any* operand) {
  if (operand == nullptr) {
    return nullptr;
  } else {
    return operand->template GetValue<ValueType>();
  }
}

template <typename ValueType>
inline ValueType* AnyCast(Any* operand) {
  if (operand == nullptr) {
    return nullptr;
  } else {
    return operand->template GetValue<ValueType>();
  }
}

// TODO(hewei03): Throw exception
template <typename ValueType>
inline ValueType AnyCast(Any&& operand) {
  auto p = operand.template GetValue<ValueType>();
  if (p != nullptr) {
    return *p;
  } else {
    throw BadAnyCast();
  }
}

template <typename ValueType>
inline ValueType AnyCast(const Any& operand) {
  auto p = operand.template GetValue<ValueType>();
  if (p != nullptr) {
    return *p;
  } else {
    throw BadAnyCast();
  }
}

}  // namespace data
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_BASE_ANY_H_
