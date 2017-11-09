#ifndef MODULES_LOCALIZATION_MSF_LOCAL_TOOL_OFFLINE_LOCAL_VISUALIZER_H
#define MODULES_LOCALIZATION_MSF_LOCAL_TOOL_OFFLINE_LOCAL_VISUALIZER_H

#include <string>
#include "modules/localization/msf/local_tool/local_visualization/visualization_engine.h"

namespace apollo {
namespace localization {
namespace msf {

class OfflineLocalVisualizer {
 public:
  OfflineLocalVisualizer();
  ~OfflineLocalVisualizer();

 public:
  bool Init(const std::string &map_folder, const std::string &pcd_folder,
            const std::string &gnss_loc_file, const std::string &lidar_loc_file,
            const std::string &fusion_loc_file,
            const std::string &extrinsic_file);

  void Visualize();

 private:
  bool LidarLocFileHandler();
  bool GnssLocFileHandler(const std::vector<double> &pcd_timestamps);
  bool FusionLocFileHandler(const std::vector<double> &pcd_timestamps);

  void PoseInterpolationByTime(
      const std::vector<Eigen::Affine3d> &in_poses,
      const std::vector<double> &in_timestamps,
      const std::vector<double> &ref_timestamps,
      std::map<unsigned int, Eigen::Affine3d> &out_poses);
  bool GetZoneIdFromMapFolder(const std::string &map_folder,
                              const unsigned int &resolution_id, int &zone_id);

 private:
  std::string _map_folder;
  std::string _pcd_folder;
  std::string _gnss_loc_file;
  std::string _lidar_loc_file;
  std::string _fusion_loc_file;
  std::string _extrinsic_file;

  std::vector<double> _pcd_timestamps;
  std::map<unsigned int, Eigen::Affine3d> _gnss_poses;
  std::map<unsigned int, Eigen::Affine3d> _lidar_poses;
  std::map<unsigned int, Eigen::Affine3d> _fusion_poses;

  BaseMapConfig _map_config;
  unsigned int _resolution_id;
  int _zone_id;

  Eigen::Affine3d _velodyne_extrinsic;

  VisualizationEngine _visual_engine;
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo

#endif  // MODULES_LOCALIZATION_MSF_LOCAL_TOOL_OFFLINE_LOCAL_VISUALIZER_H
