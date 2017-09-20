/**
 * @file BDJunctionViewInfo.h
 * @author Infotainment Software Development Team
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
 * PURPOSE, AND NOTWITHSTANDING ANY OTHER PROVISION CONTAINED HEREIN, ANY
 * LIABILITY FOR DAMAGES RESULTING FROM THE SOFTWARE OR ITS USE IS
 * EXPRESSLY DISCLAIMED, WHETHER ARISING IN CONTRACT, TORT (INCLUDING
 * NEGLIGENCE) OR STRICT LIABILITY, EVEN IF COPYRIGHT OWNERS ARE ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGES.
 */

#ifndef BAIDU_MAPAUTO_NAVI_GUIDE_BDJUNCTIONVIEWINFO_H
#define BAIDU_MAPAUTO_NAVI_GUIDE_BDJUNCTIONVIEWINFO_H

#include <BDCommonTypes.h>

namespace baidu{
namespace mapauto {
namespace navi {
namespace guide {

/**
 * @brief This is the data class that holds the junction view information for IHRouteGuideViewListener.onJunctionViewUpdated
 * @author Sun Fengyan (sunfengyan@baidu.com)
 * @sa IHRouteGuideViewListener.onJunctionViewUpdated
 */
class BDJunctionViewInfo {
public:
    /**
     * @brief The constructor which should be initiated in order to use this class
     */  	
    BDJunctionViewInfo();

    BDJunctionViewInfo(const common::BDGeoCoord& geoCoord,
                      const BDUInt32& remainDistance,
                      const BDUInt32& remainDistanceRatio,
                      const BDUInt32& id);

	/**
     * @brief The destructor must be called when the lane information is not needed anymore
     */
    ~BDJunctionViewInfo();

    /**
     * @brief This function is to override the assignment operator
     * @param[in] The source object to be assigned
     */
    BDJunctionViewInfo& operator=(const BDJunctionViewInfo& object);

	/**
     * @brief Get geo coord 
     * @retval BDGeoCoord geo coord of element
     */	
    common::BDGeoCoord& getGeoCoord() const;

	/**
     * @brief Get remain distance
     * @retval BDUInt32 remain distance of element
     */	
    BDUInt32 getRemainDistance() const;

	/**
     * @brief Get remain distance ratio
     * @retval remain distance ratio of element
     */	
    BDUInt32 getRemainDistanceRatio();

	/**
	 * @brief Get the id of junction view
	 * @retval id of element
	 */
    BDUInt32 getId();

private:
    class BDJunctionViewInfoImpl;
    BDJunctionViewInfoImpl* m_pImpl;
};

} // namespace guide
} // namespace navi
} // namespace mapauto
} // baidu
#endif // BAIDU_MAPAUTO_NAVI_GUIDE_BDJUNCTIONVIEWINFO_H
