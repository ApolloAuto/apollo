#ifndef BAIDU_ADU_LOCALIZATION_LOSSY_MAP_2D_H
#define BAIDU_ADU_LOCALIZATION_LOSSY_MAP_2D_H

#include "modules/localization/msf/local_map/base_map/base_map.h"
#include "modules/localization/msf/local_map/lossy_map/lossy_map_config_2d.h"

namespace apollo {
namespace localization {    
namespace msf {

class LossyMap2D: public BaseMap {
public:
    explicit LossyMap2D(LossyMapConfig2D& config);
    ~LossyMap2D();

    /**@brief Preload map nodes for the next frame location calculation. 
     * It will forecasts the nodes by the direction of the car moving. 
     * Because the progress of loading will cost a long time (over 100ms), 
     * it must do this for a period of time in advance. 
     * After the index of nodes calculate finished, it will create loading tasks, 
     * but will not wait for the loading finished, eigen version. */
    virtual void preload_map_area(const Eigen::Vector3d& location, const Eigen::Vector3d& trans_diff, 
                    unsigned int resolution_id, unsigned int zone_id);
    /**@brief Load map nodes for the location calculate of this frame. 
     * If the forecasts are correct in last frame, these nodes will be all in cache, if not, 
     * then need to create loading tasks, and wait for the loading finish, 
     * in order to the nodes which the following calculate needed are all in the memory, eigen version. */
    virtual bool load_map_area(const Eigen::Vector3d& seed_pt3d, unsigned int resolution_id, 
            unsigned int zone_id, int filter_size_x, int filter_size_y);
};

} // namespace msf
} // namespace localization
} // namespace apollo

#endif  //BAIDU_ADU_LOCALIZATION_LOSSY_MAP_2D_H