/**
 * @file BDLocationInfo.h
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
 
#ifndef BAIDU_MAPAUTO_NAVI_LOCATION_HLOCATIONINFO_H
#define BAIDU_MAPAUTO_NAVI_LOCATION_HLOCATIONINFO_H

#include <BDCommonTypes.h>

namespace baidu {
namespace mapauto {
namespace navi {
namespace location {


class BDLocationInfo {
public:
    BDLocationInfo();
    BDLocationInfo(
            const BDInt32& longitude,
            const BDInt32& latitude,
            const BDFloat& angle,
            const BDFloat& altitude,
            const BDFloat& speed,
            const BDFloat& accuracy,
            const BDInt32& satellite_number,
            const BDInt32& wifi_flag,
            const BDInt32& tick_count);

    ~BDLocationInfo();

    BDLocationInfo(const BDLocationInfo& object);

    /**
     * @brief This function is to override the assignment operator
     * @param[in] The source object to be assigned
     */
    BDLocationInfo& operator=(const BDLocationInfo& object);

    /**
     * @brief This function is to override == operator
     * @param[in] object right side handler
     * @retval TRUE if two objects's are same
     * @retval FALSE if two objects' are different
     */
    BDBool operator==(const BDLocationInfo& object) const;

public:
    BDInt32 getLongitude() const;
    BDInt32 getLatitude() const;
    BDFloat getAngle() const;
    BDFloat getAltitude() const;
    BDFloat getSpeed() const;  
    BDFloat getAccuracy() const;
    BDInt32 getSatelliteNumber() const;
    BDInt32 getWifiFlag() const;
    BDInt32 getTickCount() const;

private:
    class BDLocationInfoImpl;
    BDLocationInfoImpl* m_pImpl;    
};

} // namespace location
} // namespace navi
} // namespace mapauto
} // baidu

#endif // BAIDU_MAPAUTO_NAVI_LOCATION_HLOCATIONINFO_H
