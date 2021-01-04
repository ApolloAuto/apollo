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

#include "modules/drivers/lidar/lidar_robosense/include/lib/pcap_input.h"

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

namespace autobot {
namespace drivers {
namespace robosense {

PcapInput::PcapInput(double packet_rate, const std::string& filename,
                     bool read_once, bool read_fast, double repeat_delay)
    : Input(), _packet_rate(packet_rate) {
  _filename = filename;
  _fp = NULL;
  _pcap = NULL;
  _empty = true;
  _read_once = read_once;
  _read_fast = read_fast;
  _repeat_delay = repeat_delay;
}

void PcapInput::init() {
  if (_pcap != NULL) {
    return;
  }

  if (_read_once) {
    AINFO << "Read input file only once.";
  }

  if (_read_fast) {
    AINFO << "Read input file as quickly as possible.";
  }

  if (_repeat_delay > 0.0) {
    AINFO << "Delay %.3f seconds before repeating input file." << _repeat_delay;
  }

  if ((_pcap = pcap_open_offline(_filename.c_str(), _errbuf)) == NULL) {
    AERROR << " Error opening suteng socket dump file.";
  }
}

/** destructor */
PcapInput::~PcapInput(void) { pcap_close(_pcap); }
// static uint64_t last_pkt_stamp = 0;
/** @brief Get one suteng packet. */
int PcapInput::get_firing_data_packet(
    adu::common::sensor::suteng::SutengPacket* pkt, int time_zone,
    uint64_t _start_time) {
  struct pcap_pkthdr* header;
  const u_char* pkt_data;

  while (true) {
    int res = 0;

    if (_pcap == nullptr) {
      return -1;
    }

    if ((res = pcap_next_ex(_pcap, &header, &pkt_data)) >= 0) {
      if (header->len != FIRING_DATA_PACKET_SIZE + ETHERNET_HEADER_SIZE) {
        continue;
      }

      // Keep the reader from blowing through the file.
      if (_read_fast == false) {
        sleep(0.3);
      }
      usleep(1000);
      pkt->set_data(pkt_data + ETHERNET_HEADER_SIZE, FIRING_DATA_PACKET_SIZE);

      // uint64_t _temp_stamp = apollo::cyber::Time().Now().ToNanosecond();
      // uint64_t _pkt_start_diff = _temp_stamp -  _start_time;
      // // AINFO<<"pkt _start_time:["<<_start_time<<"]";
      // // AINFO<<"_pkt_start_diff:"<<apollo::cyber::Time(_pkt_start_diff -
      // last_pkt_stamp).ToSecond()*1000<<" ms";
      // // last_pkt_stamp = _pkt_start_diff;

      // pkt->set_stamp(_pkt_start_diff);

      // NMEATimePtr nmea_time(new NMEATime);
      // exract_nmea_time_from_packet(nmea_time, reinterpret_cast<const uint8_t
      // *>(&(pkt->data().c_str()[20])));
      //------------------------------------------
      tm pkt_time;
      memset(&pkt_time, 0, sizeof(pkt_time));
      pkt_time.tm_year = static_cast<int>(pkt->data().c_str()[20] + 100);
      pkt_time.tm_mon = static_cast<int>(pkt->data().c_str()[21] - 1);
      pkt_time.tm_mday = static_cast<int>(pkt->data().c_str()[22]);
      pkt_time.tm_hour = static_cast<int>(pkt->data().c_str()[23] + time_zone);
      pkt_time.tm_min = static_cast<int>(pkt->data().c_str()[24]);
      pkt_time.tm_sec = static_cast<int>(pkt->data().c_str()[25]);

      uint64_t timestamp_sec = static_cast<uint64_t>((mktime(&pkt_time)) * 1e9);
      uint64_t timestamp_nsec =
          static_cast<uint64_t>((
              1000 * (256 * pkt->data().c_str()[26] + pkt->data().c_str()[27]) +
              (256 * pkt->data().c_str()[28] + pkt->data().c_str()[29])) *
              1e3 +
          timestamp_sec);  // ns
      pkt->set_stamp(timestamp_nsec);
      //==========================================
      // tm pkt_time;
      // memset(&pkt_time, 0, sizeof(pkt_time));
      // pkt_time.tm_year = nmea_time->year + 100;
      // pkt_time.tm_mon = nmea_time->mon - 1;
      // pkt_time.tm_mday = nmea_time->day;
      // pkt_time.tm_hour = nmea_time->hour + time_zone;
      // pkt_time.tm_min = nmea_time->min;
      // pkt_time.tm_sec = 0;

      // uint64_t timestamp_sec = static_cast<uint64_t>(mktime(&pkt_time))*1e9;
      // uint64_t timestamp_nsec = static_cast<uint64_t>(nmea_time->sec*1e6 +
      // nmea_time->msec*1e3 + nmea_time->usec)*1e3
      //                           + timestamp_sec; //ns
      // pkt->set_stamp(timestamp_nsec);

      // // AINFO<<"_pkt_stamp_diff:"<<apollo::cyber::Time(timestamp_nsec -
      // last_pkt_stamp).ToSecond()*1000<<" ms";
      // // last_pkt_stamp = timestamp_nsec;

      if (!flags) {
        AINFO << "pcap robo first PPS-GPS-timestamp: [" << timestamp_nsec
              << "]";
        // AINFO << "first PPS-GPS-time: [" <<
        // nmea_time->year<<"/"<<nmea_time->mon<<"/"<<nmea_time->day<<"-"
        //      <<nmea_time->hour<<"/"<< nmea_time->min<<"/"<<
        //      nmea_time->sec<<"-" << nmea_time->msec
        //      <<"/"<< nmea_time->usec<< "]";
        flags = true;
      }
      // AINFO << "PPS-GPS-time: [" <<
      // nmea_time->year<<"/"<<nmea_time->mon<<"/"<<nmea_time->day<<"-"
      //        <<nmea_time->hour<<"/"<< nmea_time->min<<"/"<<
      //        nmea_time->sec<<"-" << nmea_time->msec
      //        <<"/"<< nmea_time->usec<< "]";

      _empty = false;
      return 0;  // success
    }

    if (_empty) {  // no data in file?
      AINFO << "Error %d reading suteng packet: %s" << res
            << pcap_geterr(_pcap);
      return -1;
    }

    if (_read_once) {
      AINFO << "end of file reached -- done reading.";
      return PCAP_FILE_END;
    }

    if (_repeat_delay > 0.0) {
      AINFO << "end of file reached -- delaying %.3f seconds." << _repeat_delay;
      usleep(static_cast<int>(_repeat_delay * 1000000.0));
    }

    pcap_close(_pcap);
    _pcap = pcap_open_offline(_filename.c_str(), _errbuf);
    _empty = true;  // maybe the file disappeared?
  }                 // loop back and try again
}

int PcapInput::get_positioning_data_packtet(const NMEATimePtr& nmea_time) {
  struct pcap_pkthdr* header;
  const u_char* pkt_data;

  while (true) {
    int res = 0;

    if (_pcap == nullptr) {
      return -1;
    }

    if ((res = pcap_next_ex(_pcap, &header, &pkt_data)) >= 0) {
      if (header->len != POSITIONING_DATA_PACKET_SIZE + ETHERNET_HEADER_SIZE) {
        continue;
      }

      // Keep the reader from blowing through the file.
      if (_read_fast == false) {
        // _packet_rate.sleep();
      }
      uint8_t bytes[POSITIONING_DATA_PACKET_SIZE];

      memcpy(bytes, pkt_data + ETHERNET_HEADER_SIZE,
             POSITIONING_DATA_PACKET_SIZE);
      // read successful, exract nmea time
      if (exract_nmea_time_from_packet(nmea_time, bytes + 303)) {
        _empty = false;
        return 0;  // success
      } else {
        _empty = false;
        return -1;
      }
    }

    if (_empty) {  // no data in file?
      AINFO << "Error %d reading suteng packet: %s" << res
            << pcap_geterr(_pcap);
      return -1;
    }

    if (_read_once) {
      AINFO << "end of file reached -- done reading.";
      return PCAP_FILE_END;
    }

    if (_repeat_delay > 0.0) {
      AINFO << "end of file reached -- delaying %.3f seconds." << _repeat_delay;
      usleep(static_cast<int>(_repeat_delay * 1000000.0));
    }
    // I can't figure out how to rewind the file, because it
    // starts with some kind of header.  So, close the file
    // and reopen it with pcap.
    pcap_close(_pcap);
    _pcap = pcap_open_offline(_filename.c_str(), _errbuf);
    _empty = true;  // maybe the file disappeared?
  }                 // loop back and try again
}

}  // namespace robosense
}  // namespace drivers
}  // namespace autobot
