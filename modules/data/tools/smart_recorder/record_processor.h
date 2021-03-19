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

#include <memory>
#include <string>
#include <vector>

#include "cyber/record/record_message.h"
#include "cyber/record/record_writer.h"

#include "modules/data/tools/smart_recorder/proto/smart_recorder_triggers.pb.h"
#include "modules/data/tools/smart_recorder/trigger_base.h"

namespace apollo {
namespace data {

/**
 * @class RecordProcessor
 * @brief Process messages and apply the rules based on configured triggers
 */
class RecordProcessor {
 public:
  RecordProcessor(const std::string& source_record_dir,
                  const std::string& restored_output_dir);
  virtual bool Init(const SmartRecordTrigger& trigger_conf);
  virtual bool Process() = 0;
  virtual std::string GetDefaultOutputFile() const = 0;
  virtual ~RecordProcessor() { writer_->Close(); }

 protected:
  bool InitTriggers(const SmartRecordTrigger& trigger_conf);
  bool ShouldRestore(const cyber::record::RecordMessage& msg) const;

  const std::string source_record_dir_;
  const std::string restored_output_dir_;
  std::vector<std::unique_ptr<TriggerBase>> triggers_;
  std::unique_ptr<cyber::record::RecordWriter> writer_ = nullptr;
};

}  // namespace data
}  // namespace apollo
