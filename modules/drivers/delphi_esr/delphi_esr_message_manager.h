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

/**
 * @file sensor_message_manager.h
 * @brief The class of SensorMessageManager
 */
#ifndef MODULES_DRIVERS_DELPHI_ESR_DELPHI_ESR_MESSAGE_MANAGER_H_
#define MODULES_DRIVERS_DELPHI_ESR_DELPHI_ESR_MESSAGE_MANAGER_H_

#include "modules/drivers/proto/delphi_esr.pb.h"
#include "modules/drivers/sensor_message_manager.h"

#include "modules/drivers/delphi_esr/protocol/acm_inst_req_7e0.h"
#include "modules/drivers/delphi_esr/protocol/acm_inst_resp_7e4.h"
#include "modules/drivers/delphi_esr/protocol/esr_sim1_5c0.h"
#include "modules/drivers/delphi_esr/protocol/esr_status1_4e0.h"
#include "modules/drivers/delphi_esr/protocol/esr_status2_4e1.h"
#include "modules/drivers/delphi_esr/protocol/esr_status3_4e2.h"
#include "modules/drivers/delphi_esr/protocol/esr_status4_4e3.h"
#include "modules/drivers/delphi_esr/protocol/esr_status5_5e4.h"
#include "modules/drivers/delphi_esr/protocol/esr_status6_5e5.h"
#include "modules/drivers/delphi_esr/protocol/esr_status7_5e6.h"
#include "modules/drivers/delphi_esr/protocol/esr_status8_5e7.h"
#include "modules/drivers/delphi_esr/protocol/esr_status9_5e8.h"
#include "modules/drivers/delphi_esr/protocol/esr_track01_500.h"
#include "modules/drivers/delphi_esr/protocol/esr_track02_501.h"
#include "modules/drivers/delphi_esr/protocol/esr_track03_502.h"
#include "modules/drivers/delphi_esr/protocol/esr_track04_503.h"
#include "modules/drivers/delphi_esr/protocol/esr_track05_504.h"
#include "modules/drivers/delphi_esr/protocol/esr_track06_505.h"
#include "modules/drivers/delphi_esr/protocol/esr_track07_506.h"
#include "modules/drivers/delphi_esr/protocol/esr_track08_507.h"
#include "modules/drivers/delphi_esr/protocol/esr_track09_508.h"
#include "modules/drivers/delphi_esr/protocol/esr_track10_509.h"
#include "modules/drivers/delphi_esr/protocol/esr_track11_50a.h"
#include "modules/drivers/delphi_esr/protocol/esr_track12_50b.h"
#include "modules/drivers/delphi_esr/protocol/esr_track13_50c.h"
#include "modules/drivers/delphi_esr/protocol/esr_track14_50d.h"
#include "modules/drivers/delphi_esr/protocol/esr_track15_50e.h"
#include "modules/drivers/delphi_esr/protocol/esr_track16_50f.h"
#include "modules/drivers/delphi_esr/protocol/esr_track17_510.h"
#include "modules/drivers/delphi_esr/protocol/esr_track18_511.h"
#include "modules/drivers/delphi_esr/protocol/esr_track19_512.h"
#include "modules/drivers/delphi_esr/protocol/esr_track20_513.h"
#include "modules/drivers/delphi_esr/protocol/esr_track21_514.h"
#include "modules/drivers/delphi_esr/protocol/esr_track22_515.h"
#include "modules/drivers/delphi_esr/protocol/esr_track23_516.h"
#include "modules/drivers/delphi_esr/protocol/esr_track24_517.h"
#include "modules/drivers/delphi_esr/protocol/esr_track25_518.h"
#include "modules/drivers/delphi_esr/protocol/esr_track26_519.h"
#include "modules/drivers/delphi_esr/protocol/esr_track27_51a.h"
#include "modules/drivers/delphi_esr/protocol/esr_track28_51b.h"
#include "modules/drivers/delphi_esr/protocol/esr_track29_51c.h"
#include "modules/drivers/delphi_esr/protocol/esr_track30_51d.h"
#include "modules/drivers/delphi_esr/protocol/esr_track31_51e.h"
#include "modules/drivers/delphi_esr/protocol/esr_track32_51f.h"
#include "modules/drivers/delphi_esr/protocol/esr_track33_520.h"
#include "modules/drivers/delphi_esr/protocol/esr_track34_521.h"
#include "modules/drivers/delphi_esr/protocol/esr_track35_522.h"
#include "modules/drivers/delphi_esr/protocol/esr_track36_523.h"
#include "modules/drivers/delphi_esr/protocol/esr_track37_524.h"
#include "modules/drivers/delphi_esr/protocol/esr_track38_525.h"
#include "modules/drivers/delphi_esr/protocol/esr_track39_526.h"
#include "modules/drivers/delphi_esr/protocol/esr_track40_527.h"
#include "modules/drivers/delphi_esr/protocol/esr_track41_528.h"
#include "modules/drivers/delphi_esr/protocol/esr_track42_529.h"
#include "modules/drivers/delphi_esr/protocol/esr_track43_52a.h"
#include "modules/drivers/delphi_esr/protocol/esr_track44_52b.h"
#include "modules/drivers/delphi_esr/protocol/esr_track45_52c.h"
#include "modules/drivers/delphi_esr/protocol/esr_track46_52d.h"
#include "modules/drivers/delphi_esr/protocol/esr_track47_52e.h"
#include "modules/drivers/delphi_esr/protocol/esr_track48_52f.h"
#include "modules/drivers/delphi_esr/protocol/esr_track49_530.h"
#include "modules/drivers/delphi_esr/protocol/esr_track50_531.h"
#include "modules/drivers/delphi_esr/protocol/esr_track51_532.h"
#include "modules/drivers/delphi_esr/protocol/esr_track52_533.h"
#include "modules/drivers/delphi_esr/protocol/esr_track53_534.h"
#include "modules/drivers/delphi_esr/protocol/esr_track54_535.h"
#include "modules/drivers/delphi_esr/protocol/esr_track55_536.h"
#include "modules/drivers/delphi_esr/protocol/esr_track56_537.h"
#include "modules/drivers/delphi_esr/protocol/esr_track57_538.h"
#include "modules/drivers/delphi_esr/protocol/esr_track58_539.h"
#include "modules/drivers/delphi_esr/protocol/esr_track59_53a.h"
#include "modules/drivers/delphi_esr/protocol/esr_track60_53b.h"
#include "modules/drivers/delphi_esr/protocol/esr_track61_53c.h"
#include "modules/drivers/delphi_esr/protocol/esr_track62_53d.h"
#include "modules/drivers/delphi_esr/protocol/esr_track63_53e.h"
#include "modules/drivers/delphi_esr/protocol/esr_track64_53f.h"
#include "modules/drivers/delphi_esr/protocol/esr_trackmotionpower_540.h"
#include "modules/drivers/delphi_esr/protocol/esr_valid1_5d0.h"
#include "modules/drivers/delphi_esr/protocol/esr_valid2_5d1.h"
#include "modules/drivers/delphi_esr/protocol/vehicle1_4f0.h"
#include "modules/drivers/delphi_esr/protocol/vehicle2_4f1.h"
#include "modules/drivers/delphi_esr/protocol/vehicle3_5f2.h"
#include "modules/drivers/delphi_esr/protocol/vehicle4_5f3.h"
#include "modules/drivers/delphi_esr/protocol/vehicle5_5f4.h"
#include "modules/drivers/delphi_esr/protocol/vehicle6_5f5.h"


namespace apollo {
namespace drivers {

template <>
SensorMessageManager<DelphiESR>::SensorMessageManager() {
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Acminstreq7e0, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Acminstresp7e4, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrsim15c0, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrstatus14e0, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrstatus24e1, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrstatus34e2, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrstatus44e3, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrstatus55e4, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrstatus65e5, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrstatus75e6, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrstatus85e7, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrstatus95e8, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrack01500, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrack02501, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrack03502, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrack04503, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrack05504, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrack06505, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrack07506, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrack08507, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrack09508, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrack10509, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrack1150a, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrack1250b, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrack1350c, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrack1450d, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrack1550e, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrack1650f, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrack17510, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrack18511, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrack19512, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrack20513, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrack21514, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrack22515, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrack23516, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrack24517, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrack25518, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrack26519, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrack2751a, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrack2851b, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrack2951c, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrack3051d, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrack3151e, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrack3251f, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrack33520, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrack34521, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrack35522, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrack36523, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrack37524, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrack38525, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrack39526, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrack40527, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrack41528, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrack42529, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrack4352a, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrack4452b, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrack4552c, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrack4652d, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrack4752e, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrack4852f, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrack49530, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrack50531, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrack51532, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrack52533, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrack53534, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrack54535, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrack55536, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrack56537, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrack57538, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrack58539, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrack5953a, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrack6053b, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrack6153c, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrack6253d, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrack6353e, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrack6453f, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrackmotionpower540, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrvalid15d0, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrvalid25d1, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Vehicle14f0, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Vehicle24f1, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Vehicle35f2, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Vehicle45f3, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Vehicle55f4, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Vehicle65f5, true>();
}

}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_DELPHI_ESR_DELPHI_ESR_MESSAGE_MANAGER_H_
