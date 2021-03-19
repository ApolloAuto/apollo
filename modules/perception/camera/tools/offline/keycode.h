/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#pragma once

namespace apollo {
namespace perception {

enum KeyCode {
  // Arabic numbers
  KEY_0 = 48,
  KEY_1 = 49,
  KEY_2 = 50,
  KEY_3 = 51,
  KEY_4 = 52,
  KEY_5 = 53,
  KEY_6 = 54,
  KEY_7 = 55,
  KEY_8 = 56,
  KEY_9 = 57,

  // Alphabet upper case
  KEY_UPPER_A = 65,
  KEY_UPPER_B = 66,
  KEY_UPPER_C = 67,
  KEY_UPPER_D = 68,
  KEY_UPPER_E = 69,
  KEY_UPPER_F = 70,
  KEY_UPPER_G = 71,
  KEY_UPPER_H = 72,
  KEY_UPPER_I = 73,
  KEY_UPPER_J = 74,
  KEY_UPPER_K = 75,
  KEY_UPPER_L = 76,
  KEY_UPPER_M = 77,
  KEY_UPPER_N = 78,
  KEY_UPPER_O = 79,
  KEY_UPPER_P = 80,
  KEY_UPPER_Q = 81,
  KEY_UPPER_R = 82,
  KEY_UPPER_S = 83,
  KEY_UPPER_T = 84,
  KEY_UPPER_U = 85,
  KEY_UPPER_V = 86,
  KEY_UPPER_W = 87,
  KEY_UPPER_X = 88,
  KEY_UPPER_Y = 89,
  KEY_UPPER_Z = 90,

  // Alphabet lower case
  KEY_LOWER_A = 97,
  KEY_LOWER_B = 98,
  KEY_LOWER_C = 99,
  KEY_LOWER_D = 100,
  KEY_LOWER_E = 101,
  KEY_LOWER_F = 102,
  KEY_LOWER_G = 103,
  KEY_LOWER_H = 104,
  KEY_LOWER_I = 105,
  KEY_LOWER_J = 106,
  KEY_LOWER_K = 107,
  KEY_LOWER_L = 108,
  KEY_LOWER_M = 109,
  KEY_LOWER_N = 110,
  KEY_LOWER_O = 111,
  KEY_LOWER_P = 112,
  KEY_LOWER_Q = 113,
  KEY_LOWER_R = 114,
  KEY_LOWER_S = 115,
  KEY_LOWER_T = 116,
  KEY_LOWER_U = 117,
  KEY_LOWER_V = 118,
  KEY_LOWER_W = 119,
  KEY_LOWER_X = 120,
  KEY_LOWER_Y = 121,
  KEY_LOWER_Z = 122,

  // Arrows
  KEY_LEFT = 65361,
  KEY_UP = 65362,
  KEY_RIGHT = 65363,
  KEY_DOWN = 65364,
  KEY_SHIFT_LEFT = 130897,
  KEY_SHIFT_RIGHT = 130899,

  // Combination with Shift and Control keys
  KEY_CTRL_S = 262259,
  KEY_ALT_C = 524387,

  // Num Lock is on
  // Arrows
  KEY_LEFT_NUM_LOCK_ON = 1113937,
  KEY_UP_NUM_LOCK_ON = 1113938,
  KEY_RIGHT_NUM_LOCK_ON = 1113939,
  KEY_DOWN_NUM_LOCK_ON = 1113940,
  KEY_SHIFT_LEFT_NUM_LOCK_ON = 1179475,
  KEY_SHIFT_RIGHT_NUM_LOCK_ON = 1179473,

  // Combination with Shift and Control keys
  KEY_CTRL_S_NUM_LOCK_ON = 1310835,
  KEY_ALT_C_NUM_LOCK_ON = 1572963,
};  // enum KeyCode
}  // namespace perception
}  // namespace apollo
