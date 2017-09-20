/**
 * @file BDRoadCondition.h
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

#ifndef BAIDU_MAPAUTO_NAVI_GUIDE_BDROADCONDITION_H
#define BAIDU_MAPAUTO_NAVI_GUIDE_BDROADCONDITION_H

#include <BDCommonTypes.h>
#include <BDRouteGuideTypes.h>

namespace baidu{
namespace mapauto {
namespace navi {
namespace guide {

//type of updating road condition
typedef enum _BDRGUpdateRoadConditionEnum
{
    INVAILD,   /** invalid */
    ALL,   /** all road info, not recommended */
    ONLYRC,   /** only road condition */
}BDRGUpdateRoadConditionEnum;


//type of road condition
typedef enum _BDRGRoadConditionType
{
    INVAILDTYPE            = 0x00000000,   /** invalid */
    STRAIGHTWAY         = 0x00000001,   /** good */
    SLOW                        = 0x00000002,   /** slowly */
    OBSTRUCTION         = 0x00000003,   /** bad */
    VERYOBSTRUCTION = 0x00000004,   /** very bad */
}BDRGRoadConditionType;


/**
 * @brief This is the data class that holds the roadcondition information
 * @author Sun Fengyan (sunfengyan@baidu.com)
 * @details In conventional navigation, engine will provide the road condition.
 */
class BDRoadCondition {
public:
    /**
     * @brief The constructor which should be initiated in order to use this class
     */
    BDRoadCondition();


    /**
     * @brief The constructor which has every parameters
     */
    BDRoadCondition(const uint32_t& unEndShapeIdx, const BDRGRoadConditionType& enRoadCondition,
                   const uint32_t& unEndAddDist, const uint32_t& unEndTravelTime);

    /**
     * @brief The destructor must be called when the roadcondition information is not needed anymore
     */
    ~BDRoadCondition();

    /**
     * @brief copy constructor
     */
    BDRoadCondition (const BDRoadCondition& other);

    /**
     * @brief This function is to override the assignment operator
     * @param[in] other The source object to be assigned
     */
    BDRoadCondition& operator=(const BDRoadCondition& other);

    /**
     * @brief This function is used to get road index
     * @return BDUInt32
     */
    BDUInt32 getUnEndShapeIdx() const;

    /**
     * @brief This function is used to set road index
     * @return void
     */
    void setUnEndShapeIdx(BDUInt32 unEndShapeIdx);

    /**
     * @brief This function is used to get type of road condition
     * @return BDRGRoadConditionType
     */
    BDRGRoadConditionType getEnRoadCondition() const;

    /**
     * @brief This function is used to set type of road condition
     * @return void
     */
    void setEnRoadCondition(BDRGRoadConditionType enRoadCondition);

    /**
     * @brief This function is used to get distance from begin
     * @return BDUInt32
     */
    BDUInt32 getUnEndAddDist() const;

    /**
     * @brief This function is used to set distance from begin
     * @return void
     */
    void setUnEndAddDist(BDUInt32 unEndAddDist);

    /**
     * @brief This function is used to get time for pass this road
     * @return BDUInt32
     */
    BDUInt32 getUnEndTravelTime() const;

    /**
     * @brief This function is used to set time for pass this road
     * @return void
     */
    void setUnEndTravelTime(BDUInt32 unEndTravelTime);

private:
    class Impl;
    Impl* m_pImpl;
};

} // namespace guide
} // namespace navi
} // namespace mapauto
} // namespace baidu

#endif //BAIDU_MAPAUTO_NAVI_GUIDE_BDROADCONDITION_H
