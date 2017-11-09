#include "modules/localization/msf/local_tool/local_visualization/offline_local_visualizer.h"
#include <boost/filesystem.hpp>
#include "modules/localization/msf/common/io/velodyne_utility.h"

namespace apollo {
namespace localization {
namespace msf {
OfflineLocalVisualizer::OfflineLocalVisualizer()
    : _map_config(), _resolution_id(0), _zone_id(0), _visual_engine() {
  _map_folder = "";
  _pcd_folder = "";
  _gnss_loc_file = "";
  _lidar_loc_file = "";
  _fusion_loc_file = "";
  _extrinsic_file = "";
}

OfflineLocalVisualizer::~OfflineLocalVisualizer() {}

bool OfflineLocalVisualizer::Init(const std::string &map_folder,
                                  const std::string &pcd_folder,
                                  const std::string &gnss_loc_file,
                                  const std::string &lidar_loc_file,
                                  const std::string &fusion_loc_file,
                                  const std::string &extrinsic_file) {
  _map_folder = map_folder;
  _pcd_folder = pcd_folder;
  _gnss_loc_file = gnss_loc_file;
  _lidar_loc_file = lidar_loc_file;
  _fusion_loc_file = fusion_loc_file;
  _extrinsic_file = extrinsic_file;

  _pcd_timestamps.clear();
  _gnss_poses.clear();
  _lidar_poses.clear();
  _fusion_poses.clear();

  std::string config_file = _map_folder + "/config.xml";
  _map_config._map_version = "lossy_full_alt";
  bool success = _map_config.load(config_file);
  if (!success) {
    std::cerr << "Load map config failed." << std::endl;
    return false;
  }
  std::cout << "Load map config succeed." << std::endl;

  success = velodyne::LoadExtrinsic(_extrinsic_file, _velodyne_extrinsic);
  if (!success) {
    std::cerr << "Load velodyne extrinsic failed." << std::endl;
    return false;
  }
  std::cout << "Load velodyne extrinsic succeed." << std::endl;

  success = LidarLocFileHandler();
  if (!success) {
    std::cerr << "Handle lidar localization file failed." << std::endl;
    return false;
  }
  std::cout << "Handle lidar localization file succeed." << std::endl;

  success = GnssLocFileHandler(_pcd_timestamps);
  if (!success) {
    std::cerr << "Handle gnss localization file failed." << std::endl;
    return false;
  }
  std::cout << "Handle gnss localization file succeed." << std::endl;

  success = FusionLocFileHandler(_pcd_timestamps);
  if (!success) {
    std::cerr << "Handle fusion localization file failed." << std::endl;
    return false;
  }
  std::cout << "Handle fusion localization file succeed." << std::endl;

  _resolution_id = 0;
  success = GetZoneIdFromMapFolder(_map_folder, _resolution_id, _zone_id);
  if (!success) {
    std::cerr << "Get zone id failed." << std::endl;
    return false;
  }
  std::cout << "Get zone id succeed." << std::endl;

  success = _visual_engine.Init(_map_folder, _map_config, _resolution_id,
                                _zone_id, _velodyne_extrinsic);
  if (!success) {
    std::cerr << "Visualization engine init failed." << std::endl;
    return false;
  }
  std::cout << "Visualization engine init succeed." << std::endl;

  return true;
}

void OfflineLocalVisualizer::Visualize() {
  for (unsigned int idx = 0; idx < _pcd_timestamps.size(); idx++) {
    auto found_iter = _lidar_poses.find(idx);
    if (found_iter == _lidar_poses.end()) {
      continue;
    }
    const Eigen::Affine3d &lidar_pose = found_iter->second;

    std::string pcd_file_path;
    std::ostringstream ss;
    ss << idx + 1;
    pcd_file_path = _pcd_folder + "/" + ss.str() + ".pcd";
    // std::cout << "pcd_file_path: " << pcd_file_path << std::endl;
    std::vector<Eigen::Vector3d> pt3ds;
    std::vector<unsigned char> intensities;
    apollo::localization::msf::velodyne::LoadPcds(
        pcd_file_path, idx, lidar_pose, pt3ds, intensities, false);
    _visual_engine.Visualize(lidar_pose, pt3ds);
  }
}

bool OfflineLocalVisualizer::LidarLocFileHandler() {
  std::vector<Eigen::Affine3d> poses;
  velodyne::LoadPcdPoses(_lidar_loc_file, poses, _pcd_timestamps);
  for (unsigned int idx = 0; idx < poses.size(); idx++) {
    _lidar_poses[idx] = poses[idx];
  }

  return true;
}

bool OfflineLocalVisualizer::GnssLocFileHandler(
    const std::vector<double> &pcd_timestamps) {
  std::vector<Eigen::Affine3d> poses;
  std::vector<double> timestamps;
  velodyne::LoadPcdPoses(_gnss_loc_file, poses, timestamps);

  PoseInterpolationByTime(poses, timestamps, pcd_timestamps, _gnss_poses);

  return true;
}

bool OfflineLocalVisualizer::FusionLocFileHandler(
    const std::vector<double> &pcd_timestamps) {
  std::vector<Eigen::Affine3d> poses;
  std::vector<double> timestamps;
  velodyne::LoadPcdPoses(_fusion_loc_file, poses, timestamps);

  PoseInterpolationByTime(poses, timestamps, pcd_timestamps, _fusion_poses);

  return true;
}

void OfflineLocalVisualizer::PoseInterpolationByTime(
    const std::vector<Eigen::Affine3d> &in_poses,
    const std::vector<double> &in_timestamps,
    const std::vector<double> &ref_timestamps,
    std::map<unsigned int, Eigen::Affine3d> &out_poses) {
  unsigned int index = 0;
  for (size_t i = 0; i < ref_timestamps.size(); i++) {
    double ref_timestamp = ref_timestamps[i];
    unsigned int ref_frame_id = i;
    // unsigned int matched_index = 0;
    while (in_timestamps[index] < ref_timestamp &&
           index < in_timestamps.size()) {
      ++index;
    }

    if (index < in_timestamps.size()) {
      if (index >= 1) {
        double cur_timestamp = in_timestamps[index];
        double pre_timestamp = in_timestamps[index - 1];
        assert(cur_timestamp != pre_timestamp);

        double t =
            (cur_timestamp - ref_timestamp) / (cur_timestamp - pre_timestamp);
        assert(t >= 0.0);
        assert(t <= 1.0);

        Eigen::Affine3d pre_pose = in_poses[index - 1];
        Eigen::Affine3d cur_pose = in_poses[index];
        Eigen::Quaterniond pre_quatd(pre_pose.linear());
        Eigen::Translation3d pre_transd(pre_pose.translation());
        Eigen::Quaterniond cur_quatd(cur_pose.linear());
        Eigen::Translation3d cur_transd(cur_pose.translation());

        Eigen::Quaterniond res_quatd = pre_quatd.slerp(t, cur_quatd);
        Eigen::Translation3d re_transd;
        re_transd.x() = pre_quatd.x() * (1 - t) + cur_transd.x() * t;
        re_transd.y() = pre_quatd.y() * (1 - t) + cur_transd.y() * t;
        re_transd.z() = pre_quatd.z() * (1 - t) + cur_transd.z() * t;

        out_poses[i] = re_transd * res_quatd;
        // if (fabs(cur_timestamp - ref_timestamp)
        //         < fabs(pre_timestamp - ref_timestamp)) {
        //         matched_index = index;
        // }
        // else {
        //         matched_index = index - 1;
        // }
      }
    } else {
      std::cerr << "[ERROR] No more poses. Exit now." << std::endl;
      break;
    }
    // std::cout << "Frame_id: " << i << std::endl;
  }
}

bool OfflineLocalVisualizer::GetZoneIdFromMapFolder(
    const std::string &map_folder, const unsigned int &resolution_id,
    int &zone_id) {
  char buf[256];
  snprintf(buf, 256, "/%03u", resolution_id);
  std::string folder_north = map_folder + "/map" + buf + "/north";
  std::string folder_south = map_folder + "/map" + buf + "/south";
  boost::filesystem::directory_iterator directory_end;
  boost::filesystem::directory_iterator iter_north(folder_north);
  if (iter_north == directory_end) {
    boost::filesystem::directory_iterator iter_south(folder_south);
    if (iter_south == directory_end) {
      return false;
    } else {
      std::string zone_id_full_path = (*iter_south).path().string();
      std::size_t pos = zone_id_full_path.find_last_of("/");
      std::string zone_id_str =
          zone_id_full_path.substr(pos + 1, zone_id_full_path.length());

      zone_id = -(std::stoi(zone_id_str));
      std::cout << "Find zone id: " << zone_id << std::endl;
      return true;
    }
  }
  std::string zone_id_full_path = (*iter_north).path().string();
  std::size_t pos = zone_id_full_path.find_last_of("/");
  std::string zone_id_str =
      zone_id_full_path.substr(pos + 1, zone_id_full_path.length());

  zone_id = (std::stoi(zone_id_str));
  std::cout << "Find zone id: " << zone_id << std::endl;
  return true;
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
