#ifndef MODULES_LOCALIZATION_MSF_LOCAL_TOOL_VISUALIZATION_ENGINE_H
#define MODULES_LOCALIZATION_MSF_LOCAL_TOOL_VISUALIZATION_ENGINE_H

#include <Eigen/Geometry>
#include <string>
#include <vector>
#include <map>
#include <list>
#include <opencv2/opencv.hpp>
#include "modules/localization/msf/local_map/base_map/base_map_config.h"
namespace apollo {
namespace localization {
namespace msf {

struct MapImageKey {
    MapImageKey() : level(0), 
    zone_id(0), node_north_id(0), node_east_id(0) {}
    bool operator < (const MapImageKey& key) const;
    
    unsigned int level;
    int zone_id;
    unsigned int node_north_id;
    unsigned int node_east_id;
};

class MapImageCache {
public:
    typedef std::list<std::pair<MapImageKey, 
                cv::Mat>>::iterator ListIterator;

public:
    explicit MapImageCache(int capacity) : _capacity(capacity) {}
    bool Get(const MapImageKey &key, cv::Mat &image);
    void Set(const MapImageKey &key, cv::Mat &image);

private:
    unsigned int _capacity;
    std::map<MapImageKey, ListIterator> _map;
    std::list<std::pair<MapImageKey, cv::Mat>> _list;
};

class VisualizationEngine {
public:
    VisualizationEngine();
    ~VisualizationEngine();

public:
    bool Init(const std::string &map_folder, const BaseMapConfig &map_config,
                const unsigned int resolution_id, const int zone_id, 
                const Eigen::Affine3d &extrinsic);
    void Visualize(const Eigen::Affine3d &cur_pose, 
                const std::vector<Eigen::Vector3d> &cloud);
    
private:
    void Preprocess(const std::string &map_folder);
    void Draw();
    void DrawCar(const cv::Point &bias);
    void DrawCloud(const cv::Point &bias);
    // void DrawLegend();

    void UpdateLevel();
    void GenerateMutiResolutionImages(
            const std::vector<std::string> &src_files,
            const int base_path_length, 
            const std::string &dst_folder);
    void InitOtherParams(const int x_min, const int y_min,
                    const int x_max, const int y_max,
                    const int level, const std::string &path);
    bool InitOtherParams(const std::string &params_file);
    
    void CloudToMat(const Eigen::Affine3d &cur_pose,
                const Eigen::Affine3d &velodyne_extrinsic, 
                const std::vector<Eigen::Vector3d> &cloud,
                cv::Mat &cloud_img,
                cv::Mat &cloud_img_mask); 
    void CoordToImageKey(const Eigen::Vector2d& coord,
                            MapImageKey &key);
    
    cv::Point CoordToMapGridIndex(const Eigen::Vector2d& coord, 
                    const unsigned int resolution_id, const int stride);
    cv::Point MapGridIndexToNodeGridIndex(const cv::Point& p);
    
    bool LoadImageToCache(const MapImageKey& key);

    void SetViewCenter(const double center_x, const double center_y);
    void UpdateViewCenter(const double move_x, const double move_y);
    void SetScale(const double scale);
    void UpdateScale(const double factor);
    void ProcessKey(int key);

private:
    std::string _map_folder;
    BaseMapConfig _map_config;
    unsigned int _zone_id;
    unsigned int _resolution_id;

    std::string _image_visual_resolution_path;
    std::string _image_visual_leaf_path;

    MapImageCache _map_image_cache;
    cv::Point _lt_node_index;
    cv::Point _lt_node_grid_index;

    std::string _window_name;
    cv::Mat _image_window;
    cv::Mat _big_window;
    cv::Mat _subMat[3][3];

    Eigen::Vector2d _view_center;
    double _cur_scale;
    int _cur_stride;
    int _cur_level;
    int _max_level;
    int _max_stride;

    bool _is_init;
    bool _follow_car;
    bool _auto_play;

    // Set if show multi localization
    // bool _if_show_muti_localization;

    Eigen::Affine3d _car_pose;
    cv::Mat _cloud_img;
    cv::Mat _cloud_img_mask;
    Eigen::Vector2d _cloud_img_lt_coord;
    Eigen::Affine3d _velodyne_extrinsic;
};

} // namespace msf
} // namespace localization
} // namespace apollo

#endif // MODULES_LOCALIZATION_MSF_LOCAL_TOOL_VISUALIZATION_ENGINE_H
