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

#include <string>
#include <vector>

#include "modules/data/tools/smart_recorder/proto/smart_recorder_triggers.pb.h"
#include "modules/data/tools/smart_recorder/record_processor.h"

namespace apollo {
namespace data {

/**
 * @class PostRecordProcessor
 * @brief Post processor against recorded tasks that have been completed
 */
class PostRecordProcessor : public RecordProcessor {
 public:
  PostRecordProcessor(const std::string& source_record_dir,
                      const std::string& restored_output_dir)
      : RecordProcessor(source_record_dir, restored_output_dir) {}
  bool Init(const SmartRecordTrigger& trigger_conf) override;
  bool Process() override;
  std::string GetDefaultOutputFile() const override;
  virtual ~PostRecordProcessor() = default;

 private:
  void LoadSourceRecords();

  std::vector<std::string> source_record_files_;
};

}  // namespace data
}  // namespace apollo
