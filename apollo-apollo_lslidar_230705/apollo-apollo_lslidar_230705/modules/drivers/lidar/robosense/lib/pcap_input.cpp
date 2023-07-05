/******************************************************************************
 * Copyright 2021 The Apollo Authors. All Rights Reserved.
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

#include "modules/drivers/lidar/robosense/lib/pcap_input.h"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/file.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include <string>

#include "cyber/cyber.h"

namespace apollo {
namespace drivers {
namespace robosense {

PcapInput::PcapInput(double packet_rate, const std::string& filename,
                     bool read_once, bool read_fast, double repeat_delay)
    : Input(), packet_rate_(packet_rate) {
  filename_ = filename;
  fp_ = NULL;
  pcap_ = NULL;
  empty_ = true;
  read_once_ = read_once;
  read_fast_ = read_fast;
  repeat_delay_ = repeat_delay;
}

void PcapInput::init() {
  if (pcap_ != NULL) {
    return;
  }

  if (read_once_) {
    AINFO << "Read input file only once.";
  }

  if (read_fast_) {
    AINFO << "Read input file as quickly as possible.";
  }

  if (repeat_delay_ > 0.0) {
    AINFO << "Delay %.3f seconds before repeating input file." << repeat_delay_;
  }

  if ((pcap_ = pcap_open_offline(filename_.c_str(), errbuf_)) == NULL) {
    AERROR << " Error opening suteng socket dump file.";
  }
}

/** destructor */
PcapInput::~PcapInput(void) { pcap_close(pcap_); }
// static uint64_t last_pkt_stamp = 0;
/** @brief Get one suteng packet. */
int PcapInput::get_firing_data_packet(
    apollo::drivers::suteng::SutengPacket* pkt, int time_zone,
    uint64_t start_time_) {
  struct pcap_pkthdr* header;
  const u_char* pkt_data;

  while (true) {
    int res = 0;

    if (pcap_ == nullptr) {
      return -1;
    }

    if ((res = pcap_next_ex(pcap_, &header, &pkt_data)) >= 0) {
      if (header->len != FIRING_DATA_PACKET_SIZE + ETHERNET_HEADER_SIZE) {
        continue;
      }

      // Keep the reader from blowing through the file.
      if (read_fast_ == false) {
        sleep(0.3);
      }
      usleep(1000);
      pkt->set_data(pkt_data + ETHERNET_HEADER_SIZE, FIRING_DATA_PACKET_SIZE);
      tm pkt_time;
      memset(&pkt_time, 0, sizeof(pkt_time));
      pkt_time.tm_year = static_cast<int>(pkt->data().c_str()[20] + 100);
      pkt_time.tm_mon = static_cast<int>(pkt->data().c_str()[21] - 1);
      pkt_time.tm_mday = static_cast<int>(pkt->data().c_str()[22]);
      pkt_time.tm_hour = static_cast<int>(pkt->data().c_str()[23] + time_zone);
      pkt_time.tm_min = static_cast<int>(pkt->data().c_str()[24]);
      pkt_time.tm_sec = static_cast<int>(pkt->data().c_str()[25]);

      uint64_t timestamp_sec = static_cast<uint64_t>((mktime(&pkt_time)) * 1e9);
      uint64_t timestamp_nsec = static_cast<uint64_t>(
          (1000 * (256 * pkt->data().c_str()[26] + pkt->data().c_str()[27]) +
           (256 * pkt->data().c_str()[28] + pkt->data().c_str()[29])) *
              1e3 +
          timestamp_sec);  // ns
      pkt->set_stamp(timestamp_nsec);
      if (!flags) {
        AINFO << "pcap robo first PPS-GPS-timestamp: [" << timestamp_nsec
              << "]";
        flags = true;
      }

      empty_ = false;
      return 0;  // success
    }

    if (empty_) {  // no data in file?
      AINFO << "Error %d reading suteng packet: %s" << res
            << pcap_geterr(pcap_);
      return -1;
    }

    if (read_once_) {
      AINFO << "end of file reached -- done reading.";
      return PCAP_FILE_END;
    }

    if (repeat_delay_ > 0.0) {
      AINFO << "end of file reached -- delaying %.3f seconds." << repeat_delay_;
      usleep(static_cast<int>(repeat_delay_ * 1000000.0));
    }

    pcap_close(pcap_);
    pcap_ = pcap_open_offline(filename_.c_str(), errbuf_);
    empty_ = true;  // maybe the file disappeared?
  }                 // loop back and try again
}

int PcapInput::get_positioning_data_packtet(const NMEATimePtr& nmea_time) {
  struct pcap_pkthdr* header;
  const u_char* pkt_data;

  while (true) {
    int res = 0;

    if (pcap_ == nullptr) {
      return -1;
    }

    if ((res = pcap_next_ex(pcap_, &header, &pkt_data)) >= 0) {
      if (header->len != POSITIONING_DATA_PACKET_SIZE + ETHERNET_HEADER_SIZE) {
        continue;
      }

      uint8_t bytes[POSITIONING_DATA_PACKET_SIZE];

      memcpy(bytes, pkt_data + ETHERNET_HEADER_SIZE,
             POSITIONING_DATA_PACKET_SIZE);
      // read successful, exract nmea time
      if (exract_nmea_time_from_packet(nmea_time, bytes + 303)) {
        empty_ = false;
        return 0;  // success
      } else {
        empty_ = false;
        return -1;
      }
    }

    if (empty_) {  // no data in file?
      AINFO << "Error %d reading suteng packet: %s" << res
            << pcap_geterr(pcap_);
      return -1;
    }

    if (read_once_) {
      AINFO << "end of file reached -- done reading.";
      return PCAP_FILE_END;
    }

    if (repeat_delay_ > 0.0) {
      AINFO << "end of file reached -- delaying %.3f seconds." << repeat_delay_;
      usleep(static_cast<int>(repeat_delay_ * 1000000.0));
    }
    // I can't figure out how to rewind the file, because it
    // starts with some kind of header.  So, close the file
    // and reopen it with pcap.
    pcap_close(pcap_);
    pcap_ = pcap_open_offline(filename_.c_str(), errbuf_);
    empty_ = true;  // maybe the file disappeared?
  }                 // loop back and try again
}

}  // namespace robosense
}  // namespace drivers
}  // namespace apollo
