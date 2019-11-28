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

#pragma once

#include "absl/memory/memory.h"
#include "absl/strings/string_view.h"
#include "absl/types/optional.h"
#include "absl/utility/utility.h"

namespace std {
// Drop-in replacement for code compliant with future C++ versions.
// Remove the borrowed items upon upgrading.

#if __cplusplus == 201103L

// Borrow from C++ 17.
using absl::optional;
using absl::string_view;

// Borrow from C++ 14.
using absl::make_integer_sequence;
using absl::make_unique;

#endif

}  // namespace std
