/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include <iostream>
#include "cyber/class_loader/class_loader.h"
#include "cyber/class_loader/test/base.h"

class Apple : public Base {
 public:
  virtual void DoSomething() { std::cout << "I am Apple" << std::endl; }
};

class Pear : public Base {
 public:
  virtual void DoSomething() { std::cout << "I am Pear!!!" << std::endl; }
};

class Banana : public Base {
 public:
  virtual void DoSomething() { std::cout << "I am Banana" << std::endl; }
};

class Peach : public Base {
 public:
  virtual void DoSomething() { std::cout << "I am Peach!!!" << std::endl; }
};

CLASS_LOADER_REGISTER_CLASS(Apple, Base);
CLASS_LOADER_REGISTER_CLASS(Pear, Base);
CLASS_LOADER_REGISTER_CLASS(Banana, Base);
CLASS_LOADER_REGISTER_CLASS(Peach, Base);
