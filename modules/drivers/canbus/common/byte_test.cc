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

#include "modules/drivers/canbus/common/byte.h"

#include <string>

#include "gtest/gtest.h"

namespace apollo {
namespace drivers {
namespace canbus {

TEST(ByteTest, CopyConstructor) {
  unsigned char byte_value = 0xFF;
  Byte value(&byte_value);
  Byte another_value(value);
  EXPECT_EQ(another_value.to_hex_string(), value.to_hex_string());
  EXPECT_EQ(another_value.to_binary_string(), value.to_binary_string());
}

TEST(ByteTest, SetBit) {
  unsigned char byte_value = 0xFF;
  Byte value(&byte_value);
  value.set_bit_0(1);
  EXPECT_EQ(0xFD, value.get_byte());
  value.set_bit_0(7);
  EXPECT_EQ(0x7D, value.get_byte());
  value.set_bit_1(7);
  EXPECT_EQ(0xFD, value.get_byte());
  value.set_value(0x77);
  value.set_bit_1(0);
  EXPECT_EQ(0x77, value.get_byte());

  EXPECT_TRUE(value.is_bit_1(0));
  EXPECT_TRUE(value.is_bit_1(1));
  EXPECT_FALSE(value.is_bit_1(3));
  EXPECT_TRUE(value.is_bit_1(6));
  EXPECT_FALSE(value.is_bit_1(7));
}

TEST(ByteTest, SetValue) {
  unsigned char byte_value = 0x1A;
  Byte value(&byte_value);
  value.set_value(0x06, 3, 3);
  EXPECT_EQ(0x32, value.get_byte());
  value.set_value(0x1A);
  value.set_value(0x06, 0, 8);
  EXPECT_EQ(0x06, value.get_byte());
  value.set_value(0x1A);
  value.set_value(0x06, 0, 10);
  EXPECT_EQ(0x06, value.get_byte());
  value.set_value(0x1A);
  value.set_value(0x06, 1, 7);
  EXPECT_EQ(0x0C, value.get_byte());
  value.set_value(0x1A);
  value.set_value(0x07, 1, 1);
  EXPECT_EQ(0x1A, value.get_byte());
  value.set_value(0x1A);
  value.set_value(0x07, -1, 1);
  EXPECT_EQ(0x1A, value.get_byte());
}

TEST(ByteTest, GetValue) {
  unsigned char byte_value = 0x1A;
  Byte value(&byte_value);
  EXPECT_EQ(0x05, value.get_byte(1, 3));
  EXPECT_EQ(0x01, value.get_byte(1, 1));
  EXPECT_EQ(0x00, value.get_byte(8, 1));
  EXPECT_EQ(0x00, value.get_byte(-1, 1));
  EXPECT_EQ(0x1A, value.get_byte(0, 10));
}

TEST(ByteTest, SetGetHighLowBit) {
  unsigned char byte_value = 0x37;
  Byte value(&byte_value);
  value.set_value_high_4_bits(0x0B);
  EXPECT_EQ(0x0B, value.get_byte_high_4_bits());
  EXPECT_EQ(0x07, value.get_byte_low_4_bits());
  value.set_value_low_4_bits(0x0B);
  EXPECT_EQ(0x0B, value.get_byte_high_4_bits());
  EXPECT_EQ(0x0B, value.get_byte_low_4_bits());
}

TEST(ByteTest, ByteToString) {
  unsigned char value = 0x34;
  EXPECT_EQ("34", Byte::byte_to_hex(value));
  EXPECT_EQ("00110100", Byte::byte_to_binary(value));
  uint32_t int_value = 0xE13A;
  EXPECT_EQ("E13A", Byte::byte_to_hex(int_value));
}

}  // namespace canbus
}  // namespace drivers
}  // namespace apollo
