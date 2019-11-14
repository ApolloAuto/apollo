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

#include <memory>

#include "gflags/gflags.h"

#include "absl/strings/str_cat.h"
#include "cyber/common/file.h"
#include "cyber/common/log.h"

#include "modules/data/tools/smart_recorder/post_record_processor.h"
#include "modules/data/tools/smart_recorder/realtime_record_processor.h"
#include "modules/data/tools/smart_recorder/smart_recorder_gflags.h"

using apollo::cyber::common::GetProtoFromFile;
using apollo::data::PostRecordProcessor;
using apollo::data::RealtimeRecordProcessor;
using apollo::data::RecordProcessor;
using apollo::data::SmartRecordTrigger;

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  if (FLAGS_restored_output_dir.empty()) {
    FLAGS_restored_output_dir =
        absl::StrCat(FLAGS_source_records_dir, "_restored");
  }
  AINFO << "input dir: " << FLAGS_source_records_dir
        << ". output dir: " << FLAGS_restored_output_dir
        << ". config file: " << FLAGS_smart_recorder_config_filename
        << ". program name: " << argv[0];
  SmartRecordTrigger trigger_conf;
  CHECK(GetProtoFromFile(FLAGS_smart_recorder_config_filename, &trigger_conf))
      << "Failed to load triggers config file "
      << FLAGS_smart_recorder_config_filename;
  auto processor = std::unique_ptr<RecordProcessor>(new RealtimeRecordProcessor(
      FLAGS_source_records_dir, FLAGS_restored_output_dir));
  if (!FLAGS_real_time_trigger) {
    processor = std::unique_ptr<RecordProcessor>(new PostRecordProcessor(
        FLAGS_source_records_dir, FLAGS_restored_output_dir));
  }
  if (!processor->Init(trigger_conf)) {
    AERROR << "failed to init record processor";
    return -1;
  }
  if (!processor->Process()) {
    AERROR << "failed to process records";
    return -1;
  }
  return 0;
}
