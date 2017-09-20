#ifndef BAIDU_MAPAUTO_DATA_MANAGER_H
#define BAIDU_MAPAUTO_DATA_MANAGER_H

#include <BDCommonTypes.h>

namespace baidu {
namespace mapauto {
namespace navi {
namespace data {

class BDDataManager
{
public:
    static BDDataManager* getInstance();

    BDDataManager();

    ~BDDataManager();

    /**
     * @brief Get all item of offline data
     * @param list [out]Offline data's list
     * @param size [out]The count of offline data list
     * @retval true: Get sucess
     * @retval false: Get failed
     */
    BDBool getAllItemList(BDOfflineDataInfo *list, BDUInt32 &size);

    /**
     * @brief Get a list of offline data messages in a state
     * @param status [in]State：not download、downloading、downloaded、To be updated、updating、all
     * @param list [out]The list of offline data messages
     * @param size [out]Count of list
     * @retval true: Get sucess
     * @retval false: Get failed
     */
    BDBool getItemList(BDOfflineDataStatus status, BDOfflineDataInfo *list, BDUInt32 &size);

    /**
     * @brief  Check whether the data is updated
     * @param bAppUpdate [out] App need to be updated
     * @param apkInfo [out]Info of apk
     * @param bDataUpdate [out]data need to be updated
     * @param provinceId [out] Need to updated the province id list
     * @param count [out]The number of provice need updated
     * @retval true: Success
     * @retval false: Failed
     */
    BDBool checkNewVersion(BDBool &bAppUpdate, BDNewApkInfo &apkInfo, BDBool &bDataUpdate, BDInt32 *provinceId, BDUInt32 count);

    /**
     * @brief Start download request
     * @param provinceId [in] The id of provice
     */
    void startDownloadRequest(BDInt32 provinceId);

    /**
     * @brief Start download data
     * @param provinceId [in]The id of provice
     */
    void startDownloadData(int provinceId);

    /**
     * @brief Suspend download data
     * @param provinceId [in]The id of provice
     */
    void suspendDownloadData(int provinceId);

    /**
     * @brief Remove a province data
     * @param provinceId [in]The id of provice
     */
    void removeData(int provinceId);

    /**
     * @brief Start updata data
     * @param provinceId [in]The id of provice
     */
    void startUpdateData(int provinceId);

    /**
     * @brief Suspend update data 
     * @param provinceId [in]The id of provice
     */
    void suspendUpdateData(int provinceId);

    /**
     * @brief Suspend all ownload data
     */
    void suspendAllDownload();

    /**
     * @brief Cancel update data 
     * @param provinceId [in]The id of provice
     */
    void cancelUpdateData(int provinceId); 

protected:
    BDDataManager(const BDDataManager& r);

    BDDataManager& operator=(const BDDataManager& r);

private:
    static BDDataManager *_instance;
    class BDDataManagerImpl;
    BDDataManagerImpl* m_pImpl;
};

} // namespace data
} // namespace navi
} // namespace mapauto
} // namespace baidu

#endif // BAIDU_MAPAUTO_DATA_MANAGER_H
