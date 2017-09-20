/**
 * @file BDLocation.h
 * @author zhengyu02@baidu.com
 *
 * Copyright (c) 2017+  Baidu MapAuto Company,
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

#ifndef BAIDU_MAPAUTO_NAVI_LOCATION_HLOCATION_H
#define BAIDU_MAPAUTO_NAVI_LOCATION_HLOCATION_H

#include <BDCommonTypes.h>
#include <BDLocationInfo.h>


#include <BNAObserver/IAdapterMsgObserver.h>

namespace baidu {
namespace mapauto {
namespace navi {
namespace location {

/**
 * @brief IBDRouteGuideListener class
 * @details This interface class should be implemented in a inherited class and registered to Other Module instance
 */
class IBDLocationListener {
public:
    /**
     * @brief The user might implement the destructor
     */
    virtual ~IBDLocationListener();

    /**
     * @brief This function will be invoked when location info changed.
     * @details Any field of LocationInfo changed, the function will be invoked.
     * @param[out] LocationInfo
     */
    virtual void onLocationInfoUpdated(const BDLocationInfo& LocationInfo) = 0;

};


/**
 * @brief The BDLocation class
 * @details This class provides MapMatched positioning information
 */
class BDLocation : public baidu::navi_adapter::IAdapterMsgObserver {
public:
    /**
     * @brief This method is to get instance of BDLocation
     * @details This class is singleton therefore in order to use member function, this function MUST be called at the first time.
     */
    static BDLocation* getInstance();

    /**
     * @brief This method is to set event listener instance to get events
     * @param[in] pListener The pointer of IBDLocationListener instance
     */
    void setEventListener(IBDLocationListener* pListener);
    void unsetEventListener();

    /**
     * @brief This function is to get the latest location info.
     * @details This function will not wait for the new location Info and just returns the latest data.
     * @param[out] LocationInfo
     */
    BDResult getLocationInfo(BDLocationInfo& LocationInfo) const;

    /**
     * @brief
     * @details
     * @param
     */
    BDResult setNmeaSimulator(
            std::string& trackFile, BDInt32 interval, BDFloat speed);

    BDResult setNaviTrackSimulator(
            std::string& trackFile, BDInt32 interval, BDFloat speed);

    BDResult setRouteSimulator();

    BDResult unsetSimulator();

    BDResult start();
    BDResult stop();
    BDResult pause();
    BDResult resume();

    /**
     * @brief Set speed value in Simulator Model
     * @details This function should called only on Simulator model
     */
    BDResult setSimulatorSpeed(BDFloat speed);

    /**
     * @brief Set position in Simulator Model, driver->moveto();
     * @details This function should called only on Simulator model
     */
    BDResult setSimulatorPosition(BDInt32 longitude, BDInt32 latitude);

private:
    /**
     * @function Update
     * @brief CVMsg default callback
     */
    bool Update(int unMsgID, int unArg1, void *nArg2);

private:
    BDLocation();
    ~BDLocation();

private:
    /**
     * @brief Disable copy constructor
     */
    BDLocation(const BDLocation &);
    /**
     * @brief Disable assign operator
     */
    const BDLocation &operator = (const BDLocation &);

private:
    class BDLocationImpl;
    BDLocationImpl* m_pImpl;
};

} // namespace location
} // namespace navi
} // namespace mapauto
} // namespace baidu

#endif // BAIDU_MAPAUTO_NAVI_LOCATION_HLOCATIONINFO_H
