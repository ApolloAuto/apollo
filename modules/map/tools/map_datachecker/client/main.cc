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
#include "cyber/cyber.h"
#include "modules/map/tools/map_datachecker/client/client.h"
#include "modules/map/tools/map_datachecker/client/client_gflags.h"

int main(int argc, char** argv) {
  fprintf(stderr, "parsing command lines\n");
  google::ParseCommandLineFlags(&argc, &argv, true);
  fprintf(stderr, "parsing command lines done\n");

  fprintf(stderr, "init logger\n");
  if (apollo::cyber::Init(argv[0])) {
    AINFO << "init logger succeed";
  } else {
    fprintf(stderr, "init logger failed\n");
  }

  google::SetStderrLogging(FLAGS_minloglevel);

  AINFO << "Starting Client";
  apollo::hdmap::Client client;
  if (client.run() != 0) {
    AFATAL << "Start Client Failed!";
    return -1;
  }
  return 0;
}