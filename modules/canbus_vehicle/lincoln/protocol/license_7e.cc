/* Copyright 2017 The Apollo Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/
#include "modules/canbus_vehicle/lincoln/protocol/license_7e.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace lincoln {
namespace {

using ::apollo::drivers::canbus::Byte;

template <class T>
inline T ByteTo(const Byte& byte) {
  return static_cast<T>(byte.get_byte(0, 8));
}

inline std::string ByteToString(const Byte& byte) {
  return std::string(1, ByteTo<char>(byte));
}

}  // namespace

const int32_t License7e::ID = 0x7E;

License7e::License7e()
    : vin_part0_(""),
      vin_part1_(""),
      vin_part2_(""),
      vin_part0_flag_(false),
      vin_part1_flag_(false),
      vin_part2_flag_(false),
      parse_success_(false) {}

void License7e::Parse(const std::uint8_t* bytes, int length,
                      Lincoln* chassis_detail) const {
  if (!parse_success_) {
    switch (mux(bytes, length)) {
      case 0x83:
        vin_part0_ = vin00(bytes, length) + vin01(bytes, length) +
                     vin02(bytes, length) + vin03(bytes, length) +
                     vin04(bytes, length) + vin05(bytes, length);
        vin_part0_flag_ = true;
        break;
      case 0x84:
        vin_part1_ = vin06(bytes, length) + vin07(bytes, length) +
                     vin08(bytes, length) + vin09(bytes, length) +
                     vin10(bytes, length) + vin11(bytes, length);
        vin_part1_flag_ = true;
        break;
      case 0x85:
        vin_part2_ = vin12(bytes, length) + vin13(bytes, length) +
                     vin14(bytes, length) + vin15(bytes, length) +
                     vin16(bytes, length);
        vin_part2_flag_ = true;
        break;
    }

    if (vin_part0_flag_ && vin_part1_flag_ && vin_part2_flag_) {
      parse_success_ = true;
      chassis_detail->mutable_vehicle_id()->set_vin(
          (vin_part0_ + vin_part1_ + vin_part2_));
    }
  }
}

// config detail: {'name': 'mux', 'offset': 0.0, 'precision': 1.0, 'len': 8,
// 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]', 'bit':
// 0, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
int License7e::mux(const std::uint8_t* bytes, int length) const {
  Byte frame(bytes + 0);
  return ByteTo<int>(frame);
}

// config detail: {'name': 'ready', 'offset': 0.0, 'precision': 1.0, 'len': 1,
// 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]', 'bit':
// 8, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
bool License7e::is_ready(const std::uint8_t* bytes, int length) const {
  Byte frame(bytes + 1);
  return frame.is_bit_1(0);
}

// config detail: {'name': 'trial', 'offset': 0.0, 'precision': 1.0, 'len': 1,
// 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]', 'bit':
// 9, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
bool License7e::is_trial(const std::uint8_t* bytes, int length) const {
  Byte frame(bytes + 1);
  return frame.is_bit_1(1);
}

// config detail: {'name': 'expired', 'offset': 0.0, 'precision': 1.0, 'len': 1,
// 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]', 'bit':
// 10, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
bool License7e::is_expired(const std::uint8_t* bytes, int length) const {
  Byte frame(bytes + 1);
  return frame.is_bit_1(2);
}

// config detail: {'name': 'feat_base_enabled', 'offset': 0.0, 'precision': 1.0,
// 'len': 1, 'f_type': 'valid', 'is_signed_var': False, 'physical_range':
// '[0|0]', 'bit': 16, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
bool License7e::is_feat_base_enabled(const std::uint8_t* bytes,
                                     int length) const {
  Byte frame(bytes + 2);
  return frame.is_bit_1(0);
}

// config detail: {'name': 'date0', 'offset': 0.0, 'precision': 1.0, 'len': 8,
// 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]', 'bit':
// 16, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
int License7e::date0(const std::uint8_t* bytes, int length) const {
  Byte frame(bytes + 2);
  return ByteTo<int>(frame);
}

// config detail: {'name': 'date6', 'offset': 0.0, 'precision': 1.0, 'len': 8,
// 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]', 'bit':
// 16, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
int License7e::date6(const std::uint8_t* bytes, int length) const {
  Byte frame(bytes + 2);
  return ByteTo<int>(frame);
}

// config detail: {'name': 'mac0', 'offset': 0.0, 'precision': 1.0, 'len': 8,
// 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]', 'bit':
// 16, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
int License7e::mac0(const std::uint8_t* bytes, int length) const {
  Byte frame(bytes + 2);
  return ByteTo<int>(frame);
}

// config detail: {'name': 'vin00', 'offset': 0.0, 'precision': 1.0, 'len': 8,
// 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]', 'bit':
// 16, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
std::string License7e::vin00(const std::uint8_t* bytes, int length) const {
  Byte frame(bytes + 2);
  return ByteToString(frame);
}

// config detail: {'name': 'vin06', 'offset': 0.0, 'precision': 1.0, 'len': 8,
// 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]', 'bit':
// 16, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
std::string License7e::vin06(const std::uint8_t* bytes, int length) const {
  Byte frame(bytes + 2);
  return ByteToString(frame);
}

// config detail: {'name': 'vin12', 'offset': 0.0, 'precision': 1.0, 'len': 8,
// 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]', 'bit':
// 16, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}t
std::string License7e::vin12(const std::uint8_t* bytes, int length) const {
  Byte frame(bytes + 2);
  return ByteToString(frame);
}

// config detail: {'name': 'feat_base_trial', 'offset': 0.0, 'precision': 1.0,
// 'len': 1, 'f_type': 'valid', 'is_signed_var': False, 'physical_range':
// '[0|0]', 'bit': 17, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
bool License7e::is_feat_base_trial(const std::uint8_t* bytes,
                                   int length) const {
  Byte frame(bytes + 2);
  return frame.is_bit_1(1);
}

// config detail: {'name': 'date1', 'offset': 0.0, 'precision': 1.0, 'len': 8,
// 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]', 'bit':
// 24, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
int License7e::date1(const std::uint8_t* bytes, int length) const {
  Byte frame(bytes + 3);
  return ByteTo<int>(frame);
}

// config detail: {'name': 'date7', 'offset': 0.0, 'precision': 1.0, 'len': 8,
// 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]', 'bit':
// 24, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
int License7e::date7(const std::uint8_t* bytes, int length) const {
  Byte frame(bytes + 3);
  return ByteTo<int>(frame);
}

// config detail: {'name': 'mac1', 'offset': 0.0, 'precision': 1.0, 'len': 8,
// 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]', 'bit':
// 24, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
int License7e::mac1(const std::uint8_t* bytes, int length) const {
  Byte frame(bytes + 3);
  return ByteTo<int>(frame);
}

// config detail: {'name': 'vin01', 'offset': 0.0, 'precision': 1.0, 'len': 8,
// 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]', 'bit':
// 24, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
std::string License7e::vin01(const std::uint8_t* bytes, int length) const {
  Byte frame(bytes + 3);
  return ByteToString(frame);
}

// config detail: {'name': 'vin07', 'offset': 0.0, 'precision': 1.0, 'len': 8,
// 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]', 'bit':
// 24, 'type': 'int', 'order': 'intel', 'physical_unit': '""'
std::string License7e::vin07(const std::uint8_t* bytes, int32_t length) const {
  Byte frame(bytes + 3);
  return ByteToString(frame);
}

// config detail: {'name': 'vin13', 'offset': 0.0, 'precision': 1.0, 'len': 8,
// 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]', 'bit':
// 24, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
std::string License7e::vin13(const std::uint8_t* bytes, int length) const {
  Byte frame(bytes + 3);
  return ByteToString(frame);
}

// config detail: {'name': 'date2', 'offset': 0.0, 'precision': 1.0, 'len': 8,
// 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]', 'bit':
// 32, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
int License7e::date2(const std::uint8_t* bytes, int length) const {
  Byte frame(bytes + 4);
  return ByteTo<int>(frame);
}

// config detail: {'name': 'date8', 'offset': 0.0, 'precision': 1.0, 'len': 8,
// 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]', 'bit':
// 32, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
int License7e::date8(const std::uint8_t* bytes, int length) const {
  Byte frame(bytes + 4);
  return ByteTo<int>(frame);
}

// config detail: {'name': 'mac2', 'offset': 0.0, 'precision': 1.0, 'len': 8,
// 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]', 'bit':
// 32, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
int License7e::mac2(const std::uint8_t* bytes, int length) const {
  Byte frame(bytes + 4);
  return ByteTo<int>(frame);
}

// config detail: {'name': 'vin02', 'offset': 0.0, 'precision': 1.0, 'len': 8,
// 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]', 'bit':
// 32, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
std::string License7e::vin02(const std::uint8_t* bytes, int length) const {
  Byte frame(bytes + 4);
  return ByteToString(frame);
}

// config detail: {'name': 'vin08', 'offset': 0.0, 'precision': 1.0, 'len': 8,
// 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]', 'bit':
// 32, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
std::string License7e::vin08(const std::uint8_t* bytes, int length) const {
  Byte frame(bytes + 4);
  return ByteToString(frame);
}

// config detail: {'name': 'vin14', 'offset': 0.0, 'precision': 1.0, 'len': 8,
// 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]', 'bit':
// 32, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
std::string License7e::vin14(const std::uint8_t* bytes, int length) const {
  Byte frame(bytes + 4);
  return ByteToString(frame);
}

// config detail: {'name': 'feat_base_trials_used', 'offset': 0.0,
// 'precision': 1.0, 'len': 16, 'f_type': 'value', 'is_signed_var': False,
// 'physical_range': '[0|0]', 'bit': 32, 'type': 'int', 'order': 'intel',
// 'physical_unit': '""'}
int License7e::feat_base_trials_used(const std::uint8_t* bytes,
                                     int length) const {
  Byte t0(bytes + 5);
  int x = t0.get_byte(0, 8);
  Byte t1(bytes + 4);
  int t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;
  return x;
}

// config detail: {'name': 'date3', 'offset': 0.0, 'precision': 1.0, 'len': 8,
// 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]', 'bit':
// 40, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
int License7e::date3(const std::uint8_t* bytes, int length) const {
  Byte frame(bytes + 5);
  return ByteTo<int>(frame);
}

// config detail: {'name': 'date9', 'offset': 0.0, 'precision': 1.0, 'len': 8,
// 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]', 'bit':
// 40, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
int License7e::date9(const std::uint8_t* bytes, int length) const {
  Byte frame(bytes + 5);
  return ByteTo<int>(frame);
}

// config detail: {'name': 'mac3', 'offset': 0.0, 'precision': 1.0, 'len': 8,
// 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]', 'bit':
// 40, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
int License7e::mac3(const std::uint8_t* bytes, int length) const {
  Byte frame(bytes + 5);
  return ByteTo<int>(frame);
}

// config detail: {'name': 'vin03', 'offset': 0.0, 'precision': 1.0, 'len': 8,
// 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]', 'bit':
// 40, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
std::string License7e::vin03(const std::uint8_t* bytes, int length) const {
  Byte frame(bytes + 5);
  return ByteToString(frame);
}

// config detail: {'name': 'vin09', 'offset': 0.0, 'precision': 1.0, 'len': 8,
// 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]', 'bit':
// 40, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
std::string License7e::vin09(const std::uint8_t* bytes, int length) const {
  Byte frame(bytes + 5);
  return ByteToString(frame);
}

// config detail: {'name': 'vin15', 'offset': 0.0, 'precision': 1.0, 'len': 8,
// 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]', 'bit':
// 40, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
std::string License7e::vin15(const std::uint8_t* bytes, int length) const {
  Byte frame(bytes + 5);
  return ByteToString(frame);
}

// config detail: {'name': 'date4', 'offset': 0.0, 'precision': 1.0, 'len': 8,
// 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]', 'bit':
// 48, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
int License7e::date4(const std::uint8_t* bytes, int length) const {
  Byte frame(bytes + 6);
  return ByteTo<int>(frame);
}

// config detail: {'name': 'mac4', 'offset': 0.0, 'precision': 1.0, 'len': 8,
// 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]', 'bit':
// 48, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
int License7e::mac4(const std::uint8_t* bytes, int length) const {
  Byte frame(bytes + 6);
  return ByteTo<int>(frame);
}

// config detail: {'name': 'vin04', 'offset': 0.0, 'precision': 1.0, 'len': 8,
// 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]', 'bit':
// 48, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
std::string License7e::vin04(const std::uint8_t* bytes, int length) const {
  Byte frame(bytes + 6);
  return ByteToString(frame);
}

// config detail: {'name': 'vin10', 'offset': 0.0, 'precision': 1.0, 'len': 8,
// 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]', 'bit':
// 48, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
std::string License7e::vin10(const std::uint8_t* bytes, int length) const {
  Byte frame(bytes + 6);
  return ByteToString(frame);
}

// config detail: {'name': 'vin16', 'offset': 0.0, 'precision': 1.0, 'len': 8,
// 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]', 'bit':
// 48, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
std::string License7e::vin16(const std::uint8_t* bytes, int length) const {
  Byte frame(bytes + 6);
  return ByteToString(frame);
}

// config detail: {'name': 'feat_base_trials_remaining', 'offset': 0.0,
// 'precision': 1.0, 'len': 16, 'f_type': 'value', 'is_signed_var': False,
// 'physical_range': '[0|0]', 'bit': 48, 'type': 'int', 'order': 'intel',
// 'physical_unit': '""'}
int License7e::feat_base_trials_remaining(const std::uint8_t* bytes,
                                          int length) const {
  Byte t0(bytes + 7);
  int x = t0.get_byte(0, 8);
  Byte t1(bytes + 6);
  int t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;
  return x;
}

// config detail: {'name': 'date5', 'offset': 0.0, 'precision': 1.0, 'len': 8,
// 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]', 'bit':
// 56, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
int License7e::date5(const std::uint8_t* bytes, int length) const {
  Byte frame(bytes + 7);
  return ByteTo<int>(frame);
}

// config detail: {'name': 'mac5', 'offset': 0.0, 'precision': 1.0, 'len': 8,
// 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]', 'bit':
// 56, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
int License7e::mac5(const std::uint8_t* bytes, int length) const {
  Byte frame(bytes + 7);
  return ByteTo<int>(frame);
}

// config detail: {'name': 'vin05', 'offset': 0.0, 'precision': 1.0, 'len': 8,
// 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]', 'bit':
// 56, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
std::string License7e::vin05(const std::uint8_t* bytes, int length) const {
  Byte frame(bytes + 7);
  return ByteToString(frame);
}

// config detail: {'name': 'vin11', 'offset': 0.0, 'precision': 1.0, 'len': 8,
// 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]', 'bit':
// 56, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
std::string License7e::vin11(const std::uint8_t* bytes, int length) const {
  Byte frame(bytes + 7);
  return ByteToString(frame);
}

}  // namespace lincoln
}  // namespace canbus
}  // namespace apollo
