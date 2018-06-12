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

// Registerer is a factory register mechanism that make class registration at
// compile time and allow generating an object by giving the name. Example:
//
//     class BaseClass {  // base class
//       ...
//     };
//     REGISTER_REGISTERER(BaseClass);
//     #define REGISTER_BASECLASS(name) REGISTER_CLASS(BaseClass, name)
//
//     class Sub1 : public BaseClass {
//       ...
//     };
//     REGISTER_BASE(Sub1);
//     class Sub2 : public BaseClass {
//       ...
//     };
//     REGISTER_BASE(Sub2);
//
// Note that REGISTER_BASE(sub1) should be put in cc file instead of h file,
// to avoid multi-declaration error when compile.
//
// Then you could get a new object of the sub class by:
//    Base *obj = BaseClassRegisterer::GetInstanceByName("Sub1");
//
// This is convenient when you need decide the class at runtime or by flag:
//    string name = "Sub1";
//    if (...)
//      name = "Sub2";
//    Base *obj = BaseClassRegisterer::GetInstanceByName(name);
//
// If there should be only one instance in the program by desgin,
// GetUniqInstance could be used:
//    Base *obj = BaseClassRegisterer::GetUniqInstance();
// If multi sub classes are registered, this method will cause a CHECK fail.
//

#ifndef MODULES_PERCEPTION_LIB_BASE_REGISTERER_H_
#define MODULES_PERCEPTION_LIB_BASE_REGISTERER_H_

#include <string>
#include <unordered_map>
#include <vector>

#include "gtest/gtest_prod.h"

#include "modules/common/log.h"
#include "modules/common/macro.h"

namespace apollo {
namespace perception {

// idea from boost any but make it more simple and don't use type_info.
class Any {
 public:
  Any() : content_(nullptr) {}

  template <typename ValueType>
  explicit Any(const ValueType &value)
      : content_(new Holder<ValueType>(value)) {}

  Any(const Any &other)
      : content_(other.content_ ? other.content_->clone() : nullptr) {}

  ~Any() { delete content_; }

  template <typename ValueType>
  ValueType *AnyCast() {
    return content_ ? &(static_cast<Holder<ValueType> *>(content_)->held_)
                    : nullptr;
  }

 private:
  class PlaceHolder {
   public:
    virtual ~PlaceHolder() {}
    virtual PlaceHolder *clone() const = 0;
  };

  template <typename ValueType>
  class Holder : public PlaceHolder {
   public:
    explicit Holder(const ValueType &value) : held_(value) {}
    virtual ~Holder() {}
    virtual PlaceHolder *clone() const { return new Holder(held_); }

    ValueType held_;
  };

  PlaceHolder *content_;

  FRIEND_TEST(ObjectFactoryTest, test_ObjectFactory);
};

class ObjectFactory {
 public:
  ObjectFactory() {}
  virtual ~ObjectFactory() {}
  virtual Any NewInstance() { return Any(); }

 private:
  DISALLOW_COPY_AND_ASSIGN(ObjectFactory);
};

typedef std::unordered_map<std::string, ObjectFactory *> FactoryMap;
typedef std::unordered_map<std::string, FactoryMap> BaseClassMap;
BaseClassMap &GlobalFactoryMap();

bool GetRegisteredClasses(
    const std::string &base_class_name,
    std::vector<std::string> *registered_derived_classes_names);

}  // namespace perception
}  // namespace apollo

#define REGISTER_REGISTERER(base_class)                               \
  class base_class##Registerer {                                      \
    typedef perception::Any Any;                                      \
    typedef perception::FactoryMap FactoryMap;                        \
                                                                      \
   public:                                                            \
    static base_class *GetInstanceByName(const ::std::string &name) { \
      FactoryMap &map = perception::GlobalFactoryMap()[#base_class];  \
      FactoryMap::iterator iter = map.find(name);                     \
      if (iter == map.end()) {                                        \
        for (auto c : map) {                                          \
          AERROR << "Instance:" << c.first;                           \
        }                                                             \
        AERROR << "Get instance " << name << " failed.";              \
        return nullptr;                                               \
      }                                                               \
      Any object = iter->second->NewInstance();                       \
      return *(object.AnyCast<base_class *>());                       \
    }                                                                 \
    static std::vector<base_class *> GetAllInstances() {              \
      std::vector<base_class *> instances;                            \
      FactoryMap &map = perception::GlobalFactoryMap()[#base_class];  \
      instances.reserve(map.size());                                  \
      for (auto item : map) {                                         \
        Any object = item.second->NewInstance();                      \
        instances.push_back(*(object.AnyCast<base_class *>()));       \
      }                                                               \
      return instances;                                               \
    }                                                                 \
    static const ::std::string GetUniqInstanceName() {                \
      FactoryMap &map = perception::GlobalFactoryMap()[#base_class];  \
      CHECK_EQ(map.size(), 1) << map.size();                          \
      return map.begin()->first;                                      \
    }                                                                 \
    static base_class *GetUniqInstance() {                            \
      FactoryMap &map = perception::GlobalFactoryMap()[#base_class];  \
      CHECK_EQ(map.size(), 1) << map.size();                          \
      Any object = map.begin()->second->NewInstance();                \
      return *(object.AnyCast<base_class *>());                       \
    }                                                                 \
    static bool IsValid(const ::std::string &name) {                  \
      FactoryMap &map = perception::GlobalFactoryMap()[#base_class];  \
      return map.find(name) != map.end();                             \
    }                                                                 \
  };

#define REGISTER_CLASS(clazz, name)                                           \
  class ObjectFactory##name : public apollo::perception::ObjectFactory {      \
   public:                                                                    \
    virtual ~ObjectFactory##name() {}                                         \
    virtual perception::Any NewInstance() {                                   \
      return perception::Any(new name());                                     \
    }                                                                         \
  };                                                                          \
  inline void RegisterFactory##name() {                                       \
    perception::FactoryMap &map = perception::GlobalFactoryMap()[#clazz];     \
    if (map.find(#name) == map.end()) map[#name] = new ObjectFactory##name(); \
  }

#endif  // MODULES_PERCEPTION_LIB_BASE_REGISTERER_H_
