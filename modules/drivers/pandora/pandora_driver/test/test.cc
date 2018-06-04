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

#include "pandora/pandora.h"

using apollo::drivers::hesai::Pandora;

FILE* lidarTimestampFile = fopen("lidar-timestamp.txt", "w");

double pandoraToSysTimeGap = 0;
int gpsTimestamp = 0;

void gpsCallback(int timestamp) {
  struct timeval ts;
  gettimeofday(&ts, NULL);
  gpsTimestamp = timestamp;
  double tv_sec = static_cast<double>(ts.tv_sec);
  double tv_usec = static_cast<double>(ts.tv_usec);
  double t = static_cast<double>(timestamp);
  pandoraToSysTimeGap = tv_sec + (tv_usec / 1000000.0) - timestamp;
  printf("gps: %d, gap: %f\n", timestamp, pandoraToSysTimeGap);
}

void lidarCallback(boost::shared_ptr<PPointCloud> cld, double timestamp) {
  struct timeval ts;
  gettimeofday(&ts, NULL);
  printf("lidar: %lf with frame id : %s \n", timestamp,
         cld->header.frame_id.c_str());
  // fprintf(lidarTimestampFile, "%d, %f,%f\n", gpsTimestamp, timestamp,
  //         ts.tv_sec + (double)ts.tv_usec / 1000000 - pandoraToSysTimeGap -
  //             timestamp);
}

void cameraCallback(boost::shared_ptr<cv::Mat> matp, double timestamp,
                    int picid, bool distortion) {
  printf("callback %d %d timestamp %lf \n", picid, distortion, timestamp);
}

int main(int argc, char** argv) {
  Pandora pandora(std::string("192.168.20.51"), 2368, 10110, lidarCallback,
                  gpsCallback, 13500, 9870, cameraCallback, 1, 0,
                  std::string("hesai40"));
  pandora.Start();
  while (true) {
    sleep(100);
  }
}

