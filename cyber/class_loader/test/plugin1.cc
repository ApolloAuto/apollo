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

class Circle : public Base {
 public:
  virtual void DoSomething() { std::cout << "I am Circle" << std::endl; }
};

class Rect : public Base {
 public:
  virtual void DoSomething() { std::cout << "I am Rect" << std::endl; }
  ~Rect() {}
};

class Triangle : public Base {
 public:
  virtual void DoSomething() { std::cout << "I am Triangle" << std::endl; }
};

class Star : public Base {
 public:
  virtual void DoSomething() { std::cout << "I am Star" << std::endl; }
};

CLASS_LOADER_REGISTER_CLASS(Circle, Base);
CLASS_LOADER_REGISTER_CLASS(Rect, Base);
CLASS_LOADER_REGISTER_CLASS(Triangle, Base);
CLASS_LOADER_REGISTER_CLASS(Star, Base);
