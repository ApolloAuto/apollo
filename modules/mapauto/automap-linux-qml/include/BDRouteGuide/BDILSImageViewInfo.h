/**
 * @file BDILSImageViewInfo.h
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

#ifndef BAIDU_MAPAUTO_NAVI_GUIDE_BDILSIMAGEVIEWINFO_H
#define BAIDU_MAPAUTO_NAVI_GUIDE_BDILSIMAGEVIEWINFO_H

#include <BDCommonTypes.h>

namespace baidu {
namespace mapauto {
namespace navi {
namespace guide {

/**
 * @brief
 */
enum class BDILSImageType{
	RAW_RGB,
    RAW_RGBA,
	BITMAP,
	PNG,
	JPG,
	MAX
};

/**
 * @brief The BDILSImageViewInfo class
 * @detailes This class is used on IHRouteGuideViewListener.onILSImageViewUpdated().
 * This is a kind of data class that has property about ILS image information.
 * @author Sun Fengyan (sunfengyan@baidu.com)
 */
class BDILSImageViewInfo {
public:
    BDILSImageViewInfo();
    ~BDILSImageViewInfo();
	
    BDILSImageViewInfo(const BDUInt32& id, const BDILSImageType& type, const BDUInt32& width,
                      const BDUInt32& height, const BDUInt32& bitCount, const BDUInt32& size,
                      std::string bufferPath, const BDUInt32& remainDistance, const BDUInt32& remainDistanceRadio,
                      std::string backGoundMap, std::string arrowMap);

	/**
     * @brief This function is to probihit shallow copy
     * @details The override copy constructor
     * @param[in] The source object to be copied
     */
    BDILSImageViewInfo(const BDILSImageViewInfo& object);

    /**
     * @brief This function is to override the assignment operator
     * @param[in] The source object to be assigned
     */
    BDILSImageViewInfo& operator=(const BDILSImageViewInfo& object);
        
    /**
     * @brief Get Id
     * @retval BDUInt32 id of element
     */
    BDUInt32 getId() const;

    /**
     * @brief Set Id
     * @param[in] uint32_t  id of element
     */
    void setId(const uint32_t &id);

    /**
     * @brief Get image type
     * @retval BDILSImageType image type of element
     */
    BDILSImageType getType() const;

    /**
     * @brief Set image type
     * @param[in] uint32_t image type of element
     */
    void setType(const uint32_t &type);

    /**
     * @brief Get image width
     * @retval BDUInt32 image width of element
     */	
    BDUInt32 getWidth() const;

    /**
     * @brief Set image width
     * @param[in] uint32_t image width of element
     */
    void setWidth(const uint32_t &width);

    /**
     * @brief Get image height
     * @retval BDUInt32 image height of element
     */
    BDUInt32 getHeight() const;

    /**
     * @brief Set image height
     * @param[in] uint32_t image height of element
     */
    void setHeight(const uint32_t &height);

	/**
     * @brief Get bit counts per a pixel
     * @retval BDUInt32 image bit counts of element
     */	
    BDUInt32 getBitCount() const;

    /**
     * @brief Set bit counts per a pixel
     * @param[in] uint32_t image bit counts of element
     */
    void setBitCount(const uint32_t &bitCount);
	
	/**
     * @brief Get image buffer size
     * @retval BDUInt32 image buffer size of element
     */	
    BDUInt32 getSize() const;

    /**
     * @brief Set image buffer size
     * @param[in] uint32_t image buffer size of element
     */
    void setSize(const uint32_t &size);

	/**
     * @brief Get image buffer path
     * @retval std::string image buffer of element
     */	
    std::string getShmBufferPath() const;

    /**
     * @brief Set image buffer
     * @param[in] std::string image buffer of element
     */
    void setShmBufferPath(const std::string &shmBufferPath);

	/**
     * @brief Get remain distance
     * @retval BDUInt32 remain distance of element
     */	
    BDUInt32 getRemainDistance() const;

    /**
     * @brief Set remain distance
     * @param[in] uint32_t remain distance of element
     */
    void setRemainDistance(const uint32_t &remainDistance);

	/**
     * @brief Get remain distance ratio
     * @details ratio goes to 0 from 100
     * @retval remain distance ratio of element
     */	
    BDUInt32 getRemainDistanceRatio() const;

    /**
     * @brief Set remain distance ratio
     * @param[in] uint32_t distance ratio of element; goes to 0 from 100
     */
    void setRemainDistanceRatio(const uint32_t &remainDistanceRatio);

    /**
     * @brief Get backgound path
     * @retval std::string path of background map
     */
    std::string getBackGoundMap() const;

    /**
     * @brief Set backgound map path
     * @param[in] std::string path of background map
     */
    void setBackGoundMap(const std::string &backGoundMap);

    /**
     * @brief Get arrow path
     * @retval std::string path of arrow map
     */
    std::string getArrowMap() const;

    /**
     * @brief Set arrow map path
     * @param[in] std::string path of arrow map
     */
    void setArrowMap(const std::string &arrowMap);


//    /**
//     * @brief Get geo coord
//     * @retval BDGeoCoord geo coord of element
//     */
//    common::BDGeoCoord& getGeoCoord() const;

//    /**
//     * @brief Set geo coord
//     * @param[in] common::BDGeoCoord geo coord of element
//     */
//    void setGeoCoord(const common::BDGeoCoord &geoCoord);

private:
    class BDILSImageViewInfoImpl;
    BDILSImageViewInfoImpl* m_pImpl;
};

} // namespace guide
} // namespace navi
} // namespace mapauto
} // namespace baidu
#endif // BAIDU_MAPAUTO_NAVI_GUIDE_BDILSIMAGEVIEWINFO_H
