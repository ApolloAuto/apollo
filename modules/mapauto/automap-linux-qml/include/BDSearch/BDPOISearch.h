/**
 * @file BDPOISearch.h
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

#ifndef BAIDU_MAPAUTO_NAVI_SEARCH_BDPOISEARCH_H
#define BAIDU_MAPAUTO_NAVI_SEARCH_BDPOISEARCH_H

#include <BDNaviTypes.h>
#include <BDPOISearchFilter.h>
#include <BDPOIInfo.h>
#include <BNAObserver/IAdapterMsgObserver.h>

namespace baidu {
namespace mapauto {
namespace navi {
namespace search {

class BDPOISearch;

struct BDPOISugInfo
{
    std::string name;
    std::string address;
};

//namespace baidu {
//namespace navi_adapter {

//        class IAdapterMsgObserver;

//}
//}
/**
 * @brief Callback interface class of BDPOISearch
 * @details inherit this class and implement callback function
 * @author Hansen (bianheshan@baidu.com)
 * @todo just few functions are declared now
 */
class IBDPOISearchListener {
public:
    virtual ~IBDPOISearchListener();
    /**
     * @brief callback function of BDPOISearch::search()
     * @details Client can set object as listener of multiple BDPOISearch instance
     *  So callback function will be called with pointer of BDPOISearch instance
     * @param[in] pSearch to identify callee instance
     * @param[in] latitude of position of POI. HLonLat class
     * @retval NONE
     * @sa BDPOISearch::setEventListener
     */
    virtual void onSearchResultUpdate(BDPOISearch* pSearch, const BDInt32& status, const BDInt32& count) = 0;

    /**
     * @brief callback function of BDPOISearch::searchSug()
     * @details Client can set object as listener of multiple BDPOISearch instance
     *  So callback function will be called with pointer of BDPOISearch instance
     * @param[in] pSearch to identify callee instance
     * @retval NONE
     * @sa BDPOISearch::setEventListener
     */
    virtual void onSearchSugResultUpdate(BDPOISearch* pSearch) = 0;
};


/**
 * @brief BDPOISearch.
 * @details It can be reusable. Change property of BDPOISearchFilter and call search() will get different results \n
 * but it has only one result list interanlly, \n
 * so when you call search() again after search finished \n
 * previous result list will be gone.
 * If you want to do multiple search simulatenously, create more BDPOISearch instance.
 * @author Hansen (bianheshan@baidu.com)
 * @todo just few functions are declared now
 */
class BDPOISearch : public baidu::navi_adapter::IAdapterMsgObserver {
public:
    BDPOISearch();
    ~BDPOISearch();

    /**
     * @brief set SearchFilter to search POI
     * @details must be called before search()
     * @param[in] filter all of filter parameter should be set explicitly, BDPOISearchFilter
     * @retval NONE
     * @sa BDPOISearchFilter
     */
    void setSearchFilter(const BDPOISearchFilter& filter);

    /**
     * @brief just return SearchFilter
     * @param[out] pFilter pointer of filter. default value is NULL
     */
    void getSearchFilter(BDPOISearchFilter &pFilter) const;

    /**
     * @brief set callback listener
     * @details when search() is finished. callback function of listener will be called with results.
     * @param[in] pListener instance of IBDPOISearchListener class
     * @sa IBDPOISearchListener::onSearchResultUpdate
     */
    void setEventListener(IBDPOISearchListener* pListener);

    /**
     * @brief do search, searchfilter should be set properly
     * @details search asynchronous and notfiy finish via callback
     *  if call search() again before previous search operation is not finished, it returns H_ERROR.
     * @retval H_OK
     * @retval H_ERROR
     */
    BDResult search();

    /**
     * @brief do search sug, by keyword
     * @details
     * @param[in] keyword for search
     * @retval H_OK
     * @retval H_ERROR
     */
    BDResult searchSug(std::string keyword);

    /**
     * @brief when routeplan is finished, you can use this funtion to searching pois near by route.
     * @details result only include name/address/regionCode/uid/position.
     * @param[in] keyword for search by route.
     * @param[in&out] max count of results in a page.
     * @param[in] page number.
     * @param[out] search results list.
     * @param[in] max distance between result and route.
     * @param[out] is last page.
     * @retval H_OK
     * @retval H_ERROR
     */
    BDResult searchByRoute(std::string keyword,
                          unsigned int& count,
                          unsigned int pagenum,
                          std::vector<BDPOIInfo>& results,
                          unsigned int distance,
                          bool& islastpage);

    /**
     * @brief check whether it is searching
     * @retval true searching
     * @retval false not searching now
     */
    BDBool isSearching();

    /**
     * @brief cancel the search operation()
     * @details if not searching, this function do nothing and return H_OK \n
     *  when cancel called(), search operation is finished and callback function will not be called
     * @return H_OK or H_ERROR, HReuslt
     */
    BDResult cancel();

    /**
     * @brief getResultItemCount, return value is only valid after search finished
     * @details it is same as count parameter at callback function
     * @retval [0~] count of search result
     */
    BDInt32 getResultItemCount() const;

    /**
     * @brief BDPOISearch::isLastPage
     * @return
     */
    bool isLastPage() const;

    /**
     * @brief Get results of the search operation. getResultItemList and getResultItem have same purpose, Should be called after search callback
     * @details There are so many search results. Client need to take count of the system resource and to call this function with proper parameter(offset, count) \n
     *  if nOffset+nCount > totalCount, it will only get available results.
     * @param[in] nOffset offset
     * @param[in] nCount the number of items to get
     * @param[out] pItemList list of BDPOIInfo
     * @sa BDPOIInfo, getResultItem, getResultItemCount
     */
    void getResultItemList(const BDUInt32& nOffset, const BDUInt32& nCount, std::vector<BDPOIInfo>& pItemList) const;

    /**
     * @brief Get results of the search operation. getResultItemList and getResultItem have same purpose, Should be called after search callback
     * @details It pick up only one result.
     * @param[in] nIdx index, if index > totalcount, pItem will be NULL
     * @param[out] pItem item
     * @retval BDResult
     * @sa getResultItemList, getResultItemCount
     */
    BDResult getResultItem(const BDUInt32& nIdx, BDPOIInfo& item) const;

    BDResult getSugResultItemList(std::vector<BDPOISugInfo>& sugResultList) const;

    void getTopDistrict();
private:
    /**
     * @brief Disable copy constructor
     */
    BDPOISearch(const BDPOISearch &);
    /**
     * @brief Disable assign operator
     */
    const BDPOISearch &operator = (const BDPOISearch &);

    class BDPOISearchImpl;

    BDPOISearchImpl* m_pImpl;

    bool Update(int unMsgID, int unArg1, void* nArg2);

    void getDistrictInfoByPoint(const baidu::mapauto::common::BDGeoCoord geo, int& districtId);
};

} // namespcae search
} // namespcae navi
} // namespcae mapauto
} // namespace baidu

#endif // BAIDU_MAPAUTO_NAVI_SEARCH_BDPOISEARCH_H
