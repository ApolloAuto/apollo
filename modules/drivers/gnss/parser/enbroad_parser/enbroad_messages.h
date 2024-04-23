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

// This defines enums and structures for parsing ENS binary messages. Please
// refer to ENS's
// documents for details about these messages.


#pragma once

#include <cstdint>
#include <limits>

#include "modules/drivers/gnss/proto/config.pb.h"

namespace apollo {
namespace drivers {
namespace gnss {
namespace enbroad {

enum ENAVSTANDARDFLAGSTATUS
{
	E_NAV_STANDARD_NO_PROCCSS=0,			//0:未标定完成 
	E_NAV_STANDARD_PROCCSSING=1,			//1：标定中
	E_NAV_STANDARD_PROCCSSED=2,				//2：标定完成
	E_NAV_STANDARD_TOTAL=3,					//总共状态数目
};

enum ENAVSTATUS
{
    E_NAV_STATUS_START =0,       	        //开始导航加载导航参数
    E_NAV_STATUS_ROUTH_ALIGN=1,		    	//SINS初对准
    E_NAV_STATUS_SINS_KF_INITIAL=2,      	//SINS、KF初始化
    E_NAV_STATUS_SYSTEM_STANDARD=3,      	//系统标定
    E_NAV_STATUS_IN_NAV=4,                 	//正常导航
    E_NAV_STATUS_STOP=5,                   	//停止导航
    E_NAV_STATUS_TOTAL=6,					//总共状态数目
};

//rtk状态
enum EGPSRTKSTATUS
{
    E_GPS_RTK_INVALID=0, 						//0: 无定位
	E_GPS_RTK_SPP=1,							//1：单点定位  		
    E_GPS_RTK_DGPS=2,							//2：差分解
    E_GPS_RTK_FIXED=4,							//4：RTK固定解
    E_GPS_RTK_FLOAT=5,							//5：RTK浮动解
    E_GPS_RTK_TOTAL=6,
    
};

enum EPOLLDATATYPE 
{
    E_POLL_DEV_TEMP =0,       				
    E_POLL_GNSS_STATE=20,  					
    E_POLL_IMU_STATE=40,						
    E_POLL_CAN_STATE=60,						
    E_POLL_INS_STATE=80,					
    E_POLL_GNSS2_STATE=100,  				
    E_POLL_DATA_TOTAL=120,
};
	

enum MessageId : uint16_t {
  BIN_NAV_DATA 	= 0x01AA,
  BIN_SINS_DATA = 0x02AA,
  BIN_IMU_DATA 	= 0x03AA,
  BIN_GNSS_DATA = 0x04AA,
};

// Every binary message has 32-bit CRC performed on all data including the
// header.
//constexpr uint16_t CRC_LENGTH = 4;

#pragma pack(push, 1)  // Turn off struct padding.

enum class DatumId : uint32_t {
  // We only use WGS-84.
  WGS84 = 61,
};


//帧头
enum SyncHeadByte : uint8_t {
  SYNC_HEAD_0 = 0xA5,
  SYNC_HEAD_1 = 0x5A,
  SYNC_HEAD_2 = 0x5A,
};
//帧尾
enum SyncTailByte : uint8_t {
  SYNC_TAIL_0 = 0xA5,
  SYNC_Tail_1 = 0x5A,
  SYNC_Tail_2 = 0x5A,
};

//Frame Head
struct  FrameHeader{
  SyncHeadByte 	sync_head[3];//sync Head
  uint8_t 		src_id;//src id
  uint8_t 		dst_id;//dst id
  MessageId 	message_id;//message id
  uint8_t 		rsp;  //RSP
  uint16_t		SN;	  //session NO.
  uint16_t		reserved;//RESERVED		
  uint16_t		message_length;  // Message length
};

static_assert(sizeof(FrameHeader) == 14, "Incorrect size of FrameHeader");
struct  NAV_DATA_TypeDef{

  uint32_t gps_week;       // GPS Week number.
  uint32_t gps_millisecs;  // Milliseconds of week.
  uint16_t pitch;
  uint16_t roll;
  uint16_t head;
  uint16_t gyroX;
  uint16_t gyroY;
  uint16_t gyroZ;
  uint16_t accX;
  uint16_t accY;
  uint16_t accZ;
  uint16_t magnetX;
  uint16_t magnetY;
  uint16_t magentZ;
  long long lat;
  long long lon;
  uint32_t  alt;
  uint16_t	ve;
  uint16_t	vn;
  uint16_t	vu;
  uint8_t	poll_type;
  uint16_t	poll_frame1;
  uint16_t	poll_frame2;
  uint16_t	poll_frame3;
};
static_assert(sizeof(NAV_DATA_TypeDef) == 65, "Incorrect size of NAV_DATA_TypeDef");

struct  NAV_SINS_TypeDef{

  uint32_t gps_week;       // GPS Week number.
  uint32_t gps_millisecs;  // Milliseconds of week.
  
  double pitch;
  double roll;
  double heading;
  double  ve; 			//m/s
  double  vn;
  double  vu;
  double  latitude;			//degree
  double  longitude;
  double  altitude;
  
};
static_assert(sizeof(NAV_SINS_TypeDef) == 80, "Incorrect size of NAV_SINS_TypeDef");

struct	NAV_IMU_TypeDef{

  uint32_t gps_week;	   // GPS Week number.
  uint32_t gps_millisecs;  // Milliseconds of week.
  
  uint8_t gyroFlag;			//角速度计状态0：异常，1：正常
  double  gyroX;			//degree/s
  double  gyroY;
  double  gyroZ;

  uint8_t accFlag;			//0：异常，1：正常
  double  accX;				//m/s2
  double  accY;
  double  accZ;

  uint8_t magnetFlag;			//0：异常，1：正常
  double  magnetX;				//mGauss
  double  magnetY;
  double  magnetZ;
};
static_assert(sizeof(NAV_IMU_TypeDef) == 83, "Incorrect size of NAV_IMU_TypeDef");

struct	NAV_GNSS_TypeDef{

  uint32_t gps_week;	   // GPS Week number.
  uint32_t gps_millisecs;  // Milliseconds of week.
  
  uint8_t rtkStatus; 		//
  double  latitude;			//degree
  double  longitude;
  double  altitude;

  uint8_t headingStatus;
  double heading;

  uint8_t velStatus;
  double  ve; 			//m/s
  double  vn;
  double  vu;
};
static_assert(sizeof(NAV_GNSS_TypeDef) == 67, "Incorrect size of NAV_GNSS_TypeDef");


#pragma pack(pop)


}  // namespace enbroad
}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
