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
#pragma once

namespace apollo {
namespace localization {
namespace msf {

// array_size(a) returns the number of elements in a.
template <class T, size_t N>
constexpr size_t ArraySize(T (&)[N]) {
  return N;
}

// Leap seconds change every a few years. See below for details.
// http://www.leapsecond.com/java/gpsclock.htm
// http://maia.usno.navy.mil/ser7/tai-utc.dat
//
// UNIX time counts seconds since 1970-1-1, without leap seconds.
// GPS time counts seconds since 1980-1-6, with leap seconds.
// When a leap second is inserted, UNIX time is ambiguous, as shown below.
//    UNIX date and time      UNIX epoch     GPS epoch
//    2008-12-31 23:59:59.0   1230767999.0   914803213.0
//    2008-12-31 23:59:59.5   1230767999.5   914803213.5
//    2008-12-31 23:59:60.0   1230768000.0   914803214.0
//    2008-12-31 23:59:60.5   1230768000.5   914803214.5
//    2009-01-01 00:00:00.0   1230768000.0   914803215.0
//    2009-01-01 00:00:00.5   1230768000.5   914803215.5

// A table of when a leap second is inserted and cumulative leap seconds.
static constexpr int32_t LEAP_SECONDS[][2] = {
    // UNIX time, leap seconds
    // Add future leap seconds here.
    {1483228800, 18},  // 2017-01-01
    {1435708800, 17},  // 2015-07-01
    {1341100800, 16},  // 2012-07-01
    {1230768000, 15},  // 2009-01-01
    {1136073600, 14},  // 2006-01-01
                       // We do not have any data before 2016, do we?
};

// This is number of seconds that UNIX is ahead of GPS, without leap seconds.
constexpr int32_t UNIX_GPS_DIFF = 315964800;

constexpr int64_t ONE_MILLION = 1000000L;

constexpr int64_t ONE_BILLION = 1000000000L;

template <typename T>
T UnixToGpsSeconds(T unix_seconds) {
  for (size_t i = 0; i < ArraySize(LEAP_SECONDS); ++i) {
    if (unix_seconds >= LEAP_SECONDS[i][0]) {
      return unix_seconds - (UNIX_GPS_DIFF - LEAP_SECONDS[i][1]);
    }
  }
  return static_cast<T>(0);
}

inline int64_t UnixToGpsMicroSeconds(int64_t unix_microseconds) {
  return UnixToGpsSeconds(unix_microseconds / ONE_MILLION) * ONE_MILLION +
         unix_microseconds % ONE_MILLION;
}

inline int64_t UnixToGpsNanoSeconds(int64_t unix_nanoseconds) {
  return UnixToGpsSeconds(unix_nanoseconds / ONE_BILLION) * ONE_BILLION +
         unix_nanoseconds % ONE_BILLION;
}

template <typename T>
T GpsToUnixSeconds(T gps_seconds) {
  for (size_t i = 0; i < ArraySize(LEAP_SECONDS); ++i) {
    T result = gps_seconds + (UNIX_GPS_DIFF - LEAP_SECONDS[i][1]);
    if (result >= LEAP_SECONDS[i][0]) {
      return result;
    }
  }
  return static_cast<T>(0);
}

inline int64_t GpsToUnixMicroSeconds(int64_t gps_microseconds) {
  return GpsToUnixSeconds(gps_microseconds / ONE_MILLION) * ONE_MILLION +
         gps_microseconds % ONE_MILLION;
}

inline int64_t GpsToUnixNanoSeconds(int64_t gps_nanoseconds) {
  return GpsToUnixSeconds(gps_nanoseconds / ONE_BILLION) * ONE_BILLION +
         gps_nanoseconds % ONE_BILLION;
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
