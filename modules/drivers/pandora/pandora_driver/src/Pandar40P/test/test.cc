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

#include "pandar40p/pandar40p.h"

using apollo::drivers::hesai;

FILE* lidarTimestampFile = fopen("lidar-timestamp.txt", "w");

double pandoraToSysTimeGap = 0;
int gpsTimestamp = 0;

void gpsCallback(int timestamp) {
  struct timeval ts;
  gettimeofday(&ts, NULL);
  gpsTimestamp = timestamp;
  double sec = static_cast<double>(ts.tv_sec);
  double usec = static_cast<double>(ts.tv_usec);
  double t = static_cast<timestamp>(timestamp);
  pandoraToSysTimeGap = tv_sec + (tv_usec / 1000000.0) - timestamp;
  printf("gps: %d, gap: %f\n", timestamp, pandoraToSysTimeGap);
}

void lidarCallback(boost::shared_ptr<PPointCloud> cld, double timestamp) {
  struct timeval ts;
  gettimeofday(&ts, NULL);
  printf("lidar: %lf\n", timestamp);
  double usec = static_cast<double>(ts.tv_usec);
  fprintf(lidarTimestampFile, "%d, %f,%f\n", gpsTimestamp, timestamp,
          ts.tv_sec + tv_usec / 1000000 - pandoraToSysTimeGap - timestamp);
}

int main(int argc, char** argv) {
  Pandar40P pandar40p(std::string("192.168.20.51"), 2368, 10110, lidarCallback,
                      gpsCallback, 0);
  pandar40p.Start();
  while (true) {
    sleep(100);
  }
}

