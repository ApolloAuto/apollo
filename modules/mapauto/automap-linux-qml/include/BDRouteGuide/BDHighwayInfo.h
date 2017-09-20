/**
 * @file BDHighwayInfo.h
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

#ifndef BAIDU_MAPAUTO_NAVI_GUIDE_BDHIGHWAYNFO_H
#define BAIDU_MAPAUTO_NAVI_GUIDE_BDHIGHWAYNFO_H

#include <BDCommonTypes.h>

namespace baidu{
namespace mapauto {
namespace navi {
namespace guide {

/**
 * @brief This is the data class that holds the highway information
 * @author Sun Fengyan (sunfengyan@baidu.com)
 * @details In conventional navigation, engine will provide the distance and name of tollgate and serive area and exit.
 */
class BDHighwayInfo {
public:
	/**
     * @brief The constructor which should be initiated in order to use this class
     */
    BDHighwayInfo();


    /**
     * @brief The constructor which has every parameters
     */
    BDHighwayInfo(const std::string& usExitHighwayDirectName, const std::string& usExitHighwayNextRoadName,
                 const std::string& usExitHighwayID, const std::string& usHighwayTollGateName,
                 const std::string& usHighwaySAName, const std::string& usHighwayNextSAName,
                 const std::string& usCurHighwayRoadName, const BDUInt32& unExitRemainDist,
                 const BDUInt32& unNextGPRemainDist, const BDUInt32& unTollGateRemainDist,
                 const BDUInt32& unSARemainDist, const BDUInt32& unNextSARemainDist, const BDBool& bHideInfo);

    /**
     * @brief The destructor must be called when the highway information is not needed anymore
     */
    ~BDHighwayInfo();

    /**
     * @brief copy constructor
     */
    BDHighwayInfo (const BDHighwayInfo& other);

    /**
     * @brief This function is to override the assignment operator
     * @param[in] other The source object to be assigned
     */
    BDHighwayInfo& operator=(const BDHighwayInfo& other);

    /**
     * @brief This function is used to get road name when exit highway
     * @return std::string
     */
    std::string getExitHighwayDirectName() const;

    /**
     * @brief This function is used to set road name when exit highway
     * @param[in] ExitHighwayDirectName
     * @return void
     */
    void setExitHighwayDirectName(std::string ExitHighwayDirectName);

    /**
     * @brief This function is used to get name of next road when exit highway
     * @return std::string
     */
    std::string getExitHighwayNextRoadName() const;

    /**
     * @brief This function is used to set name of next road when exit highway
     * @param[in] ExitHighwayNextRoadName
     * @return void
     */
    void setExitHighwayNextRoadName(std::string ExitHighwayNextRoadName);

    /**
     * @brief This function is used to get id of exiting highway
     * @return std::string
     */
    std::string getExitHighwayID() const;

    /**
     * @brief This function is used to set id of exiting highway
     * @param[in] ExitHighwayID
     * @return void
     */
    void setExitHighwayID(std::string ExitHighwayID);

    /**
     * @brief This function is used to get name of nearest toll gate
     * @return std::string
     */
    std::string getHighwayTollGateName() const;

    /**
     * @brief This function is used to set name of nearest toll gate
     * @param[in] HighwayTollGateName
     * @return void
     */
    void setHighwayTollGateName(std::string HighwayTollGateName);

    /**
     * @brief This function is used to get name of nearest service area
     * @return std::string
     */
    std::string getHighwaySAName() const;

    /**
     * @brief This function is used to set name of nearest service area
     * @param[in] HighwaySAName
     * @return void
     */
    void setHighwaySAName(std::string HighwaySAName);

    /**
     * @brief This function is used to get name of next nearest service area
     * @return std::string
     */
    std::string getHighwayNextSAName() const;

    /**
     * @brief This function is used to set name of next nearest service area
     * @param[in] HighwayNextSAName
     * @return void
     */
    void setHighwayNextSAName(std::string HighwayNextSAName);

    /**
     * @brief This function is used to get name of current highway road
     * @return std::string
     */
    std::string getCurHighwayRoadName() const;

    /**
     * @brief This function is used to set name of current highway road
     * @param[in] CurHighwayRoadName
     * @return void
     */
    void setCurHighwayRoadName(std::string CurHighwayRoadName);

    /**
     * @brief This function is used to get remain distance of exit
     * @return BDUInt32
     */
    BDUInt32 getExitRemainDist() const;

    /**
     * @brief This function is used to set remain distance of exit
     * @param[in] ExitRemainDist
     * @return void
     */
    void setExitRemainDist(BDUInt32& ExitRemainDist);

    /**
     * @brief This function is used to get remain distance of next guide point
     * @return BDUInt32
     */
    BDUInt32 getNextGPRemainDist() const;

    /**
     * @brief This function is used to set remain distance of next guide point
     * @param[in] NextGPRemainDist
     * @return void
     */
    void setNextGPRemainDist(BDUInt32& NextGPRemainDist);

    /**
     * @brief This function is used to get remain distance of toll gate
     * @return BDUInt32
     */
    BDUInt32 getTollGateRemainDist() const;

    /**
     * @brief This function is used to set remain distance of toll gate
     * @param[in] TollGateRemainDist
     * @return void
     */
    void setTollGateRemainDist(BDUInt32& TollGateRemainDist);

    /**
     * @brief This function is used to get remain distance of service area
     * @return BDUInt32
     */
    BDUInt32 getSARemainDist() const;

    /**
     * @brief This function is used to set remain distance of service area
     * @param[in] SARemainDist
     * @return void
     */
    void setSARemainDist(BDUInt32& SARemainDist);

    /**
     * @brief This function is used to get remain distance of next service area
     * @return BDUInt32
     */
    BDUInt32 getNextSARemainDist() const;

    /**
     * @brief This function is used to set remain distance of next service area
     * @param[in] NextSARemainDist
     * @return void
     */
    void setNextSARemainDist(BDUInt32& NextSARemainDist);

    /**
     * @brief This function is used to get status of hide highway info
     * @return BDBool
     */
    BDBool getBHideInfo() const;

    /**
     * @brief This function is used to set status of hide highway info
     * @param[in] BHideInfo
     * @return void
     */
    void setBHideInfo(BDBool& BHideInfo);

private:
	class Impl;
	Impl* m_pImpl;
};

} // namespace guide
} // namespace navi
} // namespace mapauto
} // namespace baidu

#endif // BAIDU_MAPAUTO_NAVI_GUIDE_BDHighwayInfo_H
