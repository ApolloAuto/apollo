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

#pragma once

namespace apollo {
namespace routing {

class NodeSRange {
 public:
  static bool IsEnoughForChangeLane(double start_s, double end_s);
  static bool IsEnoughForChangeLane(double length);

 public:
  NodeSRange() = default;
  NodeSRange(double s1, double s2);
  virtual ~NodeSRange() = default;

  bool operator<(const NodeSRange& other) const;
  bool IsValid() const;
  double StartS() const;
  double EndS() const;
  bool IsEnoughForChangeLane() const;
  double Length() const;

  void SetStartS(double start_s);
  void SetEndS(double end_s);
  bool MergeRangeOverlap(const NodeSRange& other);

 private:
  double start_s_ = 0.0;
  double end_s_ = 0.0;
};

}  // namespace routing
}  // namespace apollo
