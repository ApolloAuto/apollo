/**
 * @file BDMapMatchingInfo.h
 * @author zhengyu02@baidu.com
 *
 * Copyright (c) 2017  Baidu MapAuto Company,
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
 * PURPOSE, AND NOTWITHSTANDING ANY OTHER PROVISION CONTAINED BDEREIN, ANY
 * LIABILITY FOR DAMAGES RESULTING FROM THE SOFTWARE OR ITS USE IS
 * EXPRESSLY DISCLAIMED, WHETHER ARISING IN CONTRACT, TORT (INCLUDING
 * NEGLIGENCE) OR STRICT LIABILITY, EVEN IF COPYRIGHT OWNERS ARE ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGES.
 */
 
#ifndef BAIDU_MAPAUTO_NAVI_MAPMATCHING_HMAPMATCHINGINFO_H
#define BAIDU_MAPAUTO_NAVI_MAPMATCHING_HMAPMATCHINGINFO_H

#include <string>

#include <BDCommonTypes.h>

namespace baidu {
namespace mapauto {
namespace navi {
namespace mapmatching {

/**
 * @brief The BDMapMatchingType enum
 * @details Dedicated type for mapmatching result
 * @see _MM_State_Enum in mapmatch_def.h
 */
enum class BDMapMatchingType {
    // common status
    MM_INVALID,     // invalid
    MM_YAW,         // yaw
    MM_FORCE_YAW,   // forced to yaw
    MM_ON_ROUTE,    // 在路线上
    MM_TR_YAW,      // 限行偏航
    // special status
};

/**
 * @brief The HMapMatchingStatus enum
 * @details Matching Point Routing Status, BOTAI Project used at first
 */
enum class BDMapMatchingStatus {
    MM_NORMAL = 0x00000000,     // 正常道路
    MM_RING = 0x00000001,       // 环岛
    MM_MAIN_SLAVE = 0x00000002, // 主辅路切换
    MM_VIADUCT = 0x00000004,    // 高架桥上 
    MM_UNDERGROUND = 0x00000020,// 地下停车场
    MM_PARK = 0x00000040,       // 停车场
    MM_RAMP = 0x00000800,       // 匝道
    MM_TUNNEL = 0x00008000,     // 隧道
    MM_BUS = 0x00020000,        // 公交专用道
}; 
class BDMapMatchingInfo {
public:
    BDMapMatchingInfo();
    BDMapMatchingInfo(
            const BDInt32& longitude,
            const BDInt32& latitude,
            const BDFloat& angle,
            const BDFloat& altitude,
            const BDFloat& speed,
            const BDMapMatchingType& type,
            const BDUInt32& status,
            const BDInt32& distance,
            const BDInt32& link_level,
            const BDDouble& link_length,
            const std::string& link_name,
			const BDUInt32& sequence);

    ~BDMapMatchingInfo();

    BDMapMatchingInfo(const BDMapMatchingInfo& object);

    /**
     * @brief This function is to override the assignment operator
     * @param[in] The source object to be assigned
     */
    BDMapMatchingInfo& operator=(const BDMapMatchingInfo& object);

    /**
     * @brief This function is to override == operator
     * @param[in] object right side handler
     * @retval TRUE if two objects's are same
     * @retval FALSE if two objects' are different
     */
    BDBool operator==(const BDMapMatchingInfo& object) const;

public:
    BDInt32 getLongitude() const;
    BDInt32 getLatitude() const;
    BDFloat getAngle() const;
    BDFloat getAltitude() const;
    BDFloat getSpeed() const;  
    BDMapMatchingType getMatchingType() const;
    BDUInt32 getMatchingStatus() const;
    BDInt32 getAddDistance() const;
    BDInt32 getMatchingLinkLevel() const;
    BDDouble getMatchingLinkLength() const;
    std::string getMatchingLinkName() const;
    BDUInt32 getSequence() const;

private:
    class BDMapMatchingInfoImpl;
    BDMapMatchingInfoImpl* m_pImpl;    
};


class BDInertialInfo {
public:
    BDInertialInfo();
    BDInertialInfo(
            const BDInt32& longitude,
            const BDInt32& latitude,
            const BDFloat& angle,
            const BDFloat& altitude,
            const BDFloat& speed);

    ~BDInertialInfo();

    BDInertialInfo(const BDInertialInfo& object);

    /**
     * @brief This function is to override the assignment operator
     * @param[in] The source object to be assigned
     */
    BDInertialInfo& operator=(const BDInertialInfo& object);

    /**
     * @brief This function is to override == operator
     * @param[in] object right side handler
     * @retval TRUE if two objects's are same
     * @retval FALSE if two objects' are different
     */
    BDBool operator==(const BDInertialInfo& object) const;

public:
    BDInt32 getLongitude() const;
    BDInt32 getLatitude() const;
    BDFloat getAngle() const;
    BDFloat getAltitude() const;
    BDFloat getSpeed() const;  

private:
    class BDInertialInfoImpl;
    BDInertialInfoImpl* m_pImpl;    
};


} // namespace mapmatching
} // namespace navi
} // namespace mapauto
} // baidu

#endif // BAIDU_MAPAUTO_NAVI_MAPMATCHING_HMPAMATCHINGINFO_H
