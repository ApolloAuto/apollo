/**
 * @file BDMapMatching.h
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

#ifndef BAIDU_MAPAUTO_NAVI_MAPMATCHING_HMAPMATCHING_H
#define BAIDU_MAPAUTO_NAVI_MAPMATCHING_HMAPMATCHING_H

#include <BDCommonTypes.h>
#include <BDMapMatchingInfo.h>

#include <BNAObserver/IAdapterMsgObserver.h>
#include <BNAInterface/navi_auto_adapter_if.h>

namespace baidu {
namespace mapauto {
namespace navi {
namespace mapmatching {

/**
 * @brief IBDRouteGuideListener class
 * @details This interface class should be implemented in a inherited class and registered to Other Module instance
 */
class IBDMapMatchingListener {
public:
    /**
     * @brief The user might implement the destructor
     */
    virtual ~IBDMapMatchingListener();

    /**
     * @brief This function will be invoked when mapmatching info changed.
     * @details Any field of MapMatchingInfo changed, the function will be invoked.
     * @param[out] MapMathcingInfo
     */
    virtual void onMapMatchingInfoUpdated(const BDMapMatchingInfo& MapMatchingInfo) = 0;

    /**
     * @brief 
     */
    virtual void onInertialInfoUpdated(const BDInertialInfo& InertialInfo) = 0;
};


/**
 * @brief The BDMapMatching class
 * @details This class provides MapMatched positioning information
 */
class BDMapMatching : public baidu::navi_adapter::IAdapterMsgObserver {
public:
    /**
     * @brief This method is to get instance of BDMapMatching
     * @details This class is singleton therefore in order to use member function, this function MUST be called at the first time.
     */
    static BDMapMatching* getInstance();

    /**
     * @brief This method is to set event listener instance to get events
     * @param[in] pListener The pointer of IBDMapMatchingListener instance
     */
    void setEventListener(IBDMapMatchingListener* pListener);
    void unsetEventListener();

    /**
     * @brief This function is to get the latest mapmatched info.
     * @details This function will not wait for the new mapmatched Info and just returns the latest data.
     * @param[out] MapMatchingInfo
     */
    BDResult getMapMatchingInfo(BDMapMatchingInfo& matchingInfo) const;

    /**
     * @brief This function is to get the latest inertial position info.
     * @param[out] InertialInfo
     */
    BDResult getInertialInfo(BDInertialInfo& inertialInfo) const;

private:
    /**
     * @function Update
     * @brief CVMsg default callback
     */
    bool Update(int unMsgID, int unArg1, void *nArg2);

    BDMapMatchingType FormatMatchingType(
            baidu::navi_adapter::BNAMapMatchResult& match_result) const;
    BDUInt32 FormatMatchingStatus(
            baidu::navi_adapter::BNAMapMatchResult& match_result) const;
    BDResult FormatMatchingLinkName(
            baidu::navi_adapter::BNAMapMatchResult& match_result,
            std::string& matchingLinkName) const;


private:
    BDMapMatching();
    ~BDMapMatching();

private:
    /**
     * @brief Disable copy constructor
     */
    BDMapMatching(const BDMapMatching &);
    /**
     * @brief Disable assign operator
     */
    const BDMapMatching &operator = (const BDMapMatching &);

private:
    class BDMapMatchingImpl;
    BDMapMatchingImpl* m_pImpl;
};

} // namespace mapmatching
} // namespace navi
} // namespace mapauto
} // namespace baidu

#endif // BAIDU_MAPAUTO_NAVI_MAPMATCHING_HMAPMATCHING_H
