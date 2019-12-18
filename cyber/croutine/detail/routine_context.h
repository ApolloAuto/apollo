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

#ifndef CYBER_CROUTINE_ROUTINE_CONTEXT_H_
#define CYBER_CROUTINE_ROUTINE_CONTEXT_H_

#include <cstdlib>
#include <cstring>
#include <iostream>

#include "cyber/common/log.h"

extern "C" {
extern void ctx_swap(void**, void**) asm("ctx_swap");
};

namespace apollo {
namespace cyber {
namespace croutine {

constexpr size_t STACK_SIZE = 2 * 1024 * 1024;
#if defined __aarch64__
constexpr size_t REGISTERS_SIZE = 160;
#else
constexpr size_t REGISTERS_SIZE = 56;
#endif

typedef void (*func)(void*);
struct RoutineContext {
  char stack[STACK_SIZE];
  char* sp = nullptr;
#if defined __aarch64__
} __attribute__((aligned(16)));
#else
};
#endif

void MakeContext(const func& f1, const void* arg, RoutineContext* ctx);

inline void SwapContext(char** src_sp, char** dest_sp) {
  ctx_swap(reinterpret_cast<void**>(src_sp), reinterpret_cast<void**>(dest_sp));
}

}  // namespace croutine
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_CROUTINE_ROUTINE_CONTEXT_H_
