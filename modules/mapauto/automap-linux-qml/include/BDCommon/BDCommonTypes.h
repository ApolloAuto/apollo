/**
 * @file BDCommonTypes.h
 * @author Baidu MapAuto Team
 *
 * Copyright (c) 2017  Baidu MapAuto Company,,
 * All Rights Reserved.
 *
 * Use and copying of this software and preparation of derivative works
 * based upon this software are permitted. Any copy of this software or
 * of any derivative work must include the above copyright notice, this
 * paragraph and the one after it.  Any distribution of this software or
 * derivative works must comply with all aplicable laws.
 *
 * This software is made available AS IS, and COPYRIGHT OWNERS DISCLAIMS
 * ALL WARRANTIES, EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE, AND NOTWITHSTANDING ANY OTHER PROVISION CONTAINED HEREIN, ANY
 * LIABILITY FOR DAMAGES RESULTING FROM THE SOFTWARE OR ITS USE IS
 * EXPRESSLY DISCLAIMED, WHETHER ARISING IN CONTRACT, TORT (INCLUDING
 * NEGLIGENCE) OR STRICT LIABILITY, EVEN IF COPYRIGHT OWNERS ARE ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGES.
 */

#ifndef BAIDU_MAPAUTO_BDCOMMONTYPES_H
#define BAIDU_MAPAUTO_BDCOMMONTYPES_H

#include <string>
#include <list>
#include <vector>
#include <map>
#include <iostream>
#include <memory>
#include <utility>
namespace baidu {
namespace mapauto {

#define BD_MAX_PROVINCE_NUM            (34)
	
#define BD_MAX_TIMEINFO_LENGTH         (16)
#define BD_MAX_APKVERSION_LENGTH       (16)
#define BD_MAX_APKINFO_LENGTH          (2048)
#define BD_MAX_DATA_FILE_MD5_LENGTH    (33)            
#define BD_MAX_DATA_VERSION_LENGTH     (16)             
#define BD_MAX_URL_LENGTH              (128)   
#define BD_DISTRICT_NAME_MAX_LENGTH    (32)

/**
* @brief Defined HMC unique ID
*/
typedef int BDUID;
/**
* @brief Defined HMC Byte
*/
typedef unsigned char BDByte;
/**
 * @brief Defined HMC char
 */
typedef char BDChar;
/**
* @brief Defined HMC char
*/
typedef wchar_t BDWChar;
/**
* @brief Defined HMC Bool
*/
typedef bool BDBool;
/**
* @brief Defined HMC Int16
*/
typedef short BDInt16;
/**
* @brief Defined HMC Int32
*/
typedef int BDInt32;
/**
* @brief Defined HMC Int64
*/
typedef long long int BDInt64;
/**
* @brief Defined HMC UInt16
*/
typedef unsigned short BDUInt16;
/**
* @brief Defined HMC UInt32
*/
typedef unsigned int BDUInt32;
/**
* @brief Defined HMC UInt64
*/
typedef unsigned long long int BDUInt64;
/**
 * @brief Defined HMC BDFloat
 */
typedef float BDFloat;
/**
 * @brief Defined HMC double
 */
typedef double BDDouble;

/**
* @brief The ReturnType enum
*/
enum class BDResult {
    INVALID = -1,       /**< The Input is invalid */
    OK,                 /**< OK */
    ERROR,              /**< Something is wrong */
    OUT_OF_RANGE,       /**< Input range is wrong */
    CONNECTION_FAIL,    /**< fail to connect */
    NO_RESPONSE,        /**< No response */
    UNAVAILABLE,        /**< Unavailable */
    NULLPOINTER,        /**< Null Pointer */
    //TBD MORE
    MAX,                /**< End of Enum value */
};

/**
 * @brief This enum class illustrates the available var types that use in the BDVariant class
 * @author Baidu MapAuto Team
 */
enum class BDVarTypes {
    NONE,               /**< HVarType is not exist */
    V_STRING,           /**< String type */
    V_HINT32,           /**< BDInt32 type */
    V_HFLOAT,           /**< BDFloat type */
    V_VECTOR_STRING,    /**< Array of String */
    V_VECTOR_HINT32,    /**< Array of BDInt32 */
    V_VECTOR_HFLOAT,    /**< Array of BDFloat */
    V_HVARIANT,         /**< BDVariant Type */
    MAX
};

/**
 * @brief The Distance Unit enum
 */
enum class BDDistanceUnit {
    KILOMETER,          /**< Kilometer (metric) distance unit */
    MILE,               /**< Mile (imperial and us) distance unit */
    MAX,
};

/**
* @brief The LanguageType enum\n
* More language will be supported.
*/
enum class BDLanguageType {
    KOREAN,                     /**< Korean Language type */
    ENGLISH,                    /**< English Language type */
    CHINESE,                    /**< Chinese Language type */
    FRENCH,                     /**< French Language type */
    MAX,                        /**< End of Enum value */
};

typedef struct _BDNewApkInfo {
    BDChar updateTime[BD_MAX_TIMEINFO_LENGTH];
	BDChar apkVersion[BD_MAX_APKVERSION_LENGTH];
	BDChar apkInfo[BD_MAX_APKINFO_LENGTH];
	BDChar url[BD_MAX_URL_LENGTH];
    BDChar MD5[BD_MAX_DATA_FILE_MD5_LENGTH];
	BDUInt32 timeLength;
	BDUInt32 versionLength;
	BDUInt32 apkSize;
	BDUInt32 infoLength;
    BDUInt32 md5Length;
    BDUInt32 urlLength;
    BDInt32  apkVersionCode;
} BDNewApkInfo;

typedef struct _BDOfflineDataInfo {
    BDChar   provinceName[BD_DISTRICT_NAME_MAX_LENGTH];  
    BDInt32  provinceNameLength;                         
    BDInt32  provinceId;
    BDInt32  engineStatus;                                     /**< Engine status*/
    BDUInt32 totalDownloadSize;                                
    BDUInt32 downloadedSize;
    BDInt32  downloadProgress;
    BDUInt32 totalUpdateSize;
    BDUInt32 updatedSize;
    BDInt32  updateProgress;
    BDInt32  downloadProgressBy10;                            /**< Download progress with divided by 10*/
    BDInt32  updateProgressBy10;                              /**< Engine return update progress with divided by 10*/
    BDInt32  taskUiStatus;                                    /**< Download status for UI*/
    BDBool   hasNewerVersion = false;                         
    BDBool   isRequesting = false;
    BDBool   isSuspendByNetChange = false;
    BDBool   isSuspendByPhoneChange = false;
    BDBool   hasNeccessaryData = false;
} BDOfflineDataInfo;


enum  class BDOfflineDataStatus {
    BD_DATA_STATUS_UNDOWNLOAD = 0,           /**<  Not download */
    BD_DATA_STATUS_DOWNLOADING = 1,          /**<  Downloading */
    BD_DATA_STATUS_DOWNLOADED = 2,           /**<  Downloaded */
    BD_DATA_STATUS_NEED_UPDATE = 3,          /**<  Need update */
    BD_DATA_STATUS_UPDATING = 4,             /**<  Updating   */
    BD_DATA_STATUS_ALL = 5,                  /**<  Get all status */
};

} // mapauto
} //baidu

/**
 * Non-primitive data types that use the types defined above should be
 * included in the bottom part of this file as follow in order to avoid
 * conflictions during build time
 */

#include <BDKeyTypes.h>
#include <BDVariant.h>
#include <BDRectangle.h>
#include <BDPoint.h>
#include <BDColor.h>
#include <BDGeoCoord.h>

#endif  // BAIDU_MAPAUTO_BDCOMMONTYPES_H
