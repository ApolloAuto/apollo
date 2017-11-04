#ifndef MODULES_LOCALIZATION_MSF_LOCAL_MAP_BASE_MAP_BASE_MAP_MATRIX_H
#define MODULES_LOCALIZATION_MSF_LOCAL_MAP_BASE_MAP_BASE_MAP_MATRIX_H

#include <vector>
#include <assert.h>
#include <opencv2/opencv.hpp>
#include "modules/localization/msf/local_map/base_map/base_map_fwd.h"

namespace apollo {
namespace localization {
namespace msf {

/**@brief The data structure of the map cells in a map node. */
class BaseMapMatrix {
public:
    /**@brief The default constructor. */
    BaseMapMatrix();
    /**@brief The deconstructor. */
    virtual ~BaseMapMatrix();
    /**@brief The copy constructor. */
    BaseMapMatrix(const BaseMapMatrix& cell);
    /**@brief Initialize the map matrix. */
    virtual void init(const BaseMapConfig* config) = 0;
    /**@brief Reset map cells data. */
    virtual void reset(const BaseMapConfig* config) = 0;
    /**@brief Load the map cell from a binary chunk.
     * @param <return> The size read (the real size of object).
     */
    virtual unsigned int load_binary(unsigned char * buf) = 0;
    /**@brief Create the binary. Serialization of the object.
     * @param <buf, buf_size> The buffer and its size.
     * @param <return> The required or the used size of is returned.
     */
    virtual unsigned int create_binary(unsigned char * buf, unsigned int buf_size) const = 0;
    /**@brief Get the binary size of the object. */
    virtual unsigned int get_binary_size() const = 0;
    /**@brief get intensity image of node. */
    virtual void get_intensity_img(cv::Mat &intensity_img) const = 0;
};

} // namespace msf
} // namespace localization
} // namespace apollo

#endif // MODULES_LOCALIZATION_MSF_LOCAL_MAP_BASE_MAP_BASE_MAP_MATRIX_H