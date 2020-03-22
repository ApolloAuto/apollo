/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <vector>

#include "cyber/common/file.h"
#include "modules/localization/msf/common/io/velodyne_utility.h"
#include "modules/localization/msf/common/util/extract_ground_plane.h"
#include "modules/localization/msf/common/util/file_utility.h"
#include "modules/localization/msf/local_pyramid_map/pyramid_map/pyramid_map.h"
#include "modules/localization/msf/local_pyramid_map/pyramid_map/pyramid_map_pool.h"

const unsigned int CAR_SENSOR_LASER_NUMBER = 64;

using apollo::localization::msf::FeatureXYPlane;
using apollo::localization::msf::pyramid_map::MapNodeIndex;
using apollo::localization::msf::pyramid_map::PyramidMap;
using apollo::localization::msf::pyramid_map::PyramidMapConfig;
using apollo::localization::msf::pyramid_map::PyramidMapMatrix;
using apollo::localization::msf::pyramid_map::PyramidMapNode;
using apollo::localization::msf::pyramid_map::PyramidMapNodePool;
typedef apollo::localization::msf::FeatureXYPlane::PointT PclPointT;
typedef apollo::localization::msf::FeatureXYPlane::PointCloudT PclPointCloudT;
typedef apollo::localization::msf::FeatureXYPlane::PointCloudPtrT
    PclPointCloudPtrT;

bool ParseCommandLine(int argc, char* argv[],
                      boost::program_options::variables_map* vm) {
  boost::program_options::options_description desc("Allowd options");
  desc.add_options()("help", "product help message")(
      "use_plane_inliers_only",
      boost::program_options::value<bool>()->required(),
      "use plane inliers only")
      // ("use_plane_fitting_ransac",
      // boost::program_options::value<bool>()->required(),
      //  "use plane fitting ransac")
      ("pcd_folders",
       boost::program_options::value<std::vector<std::string>>()
           ->multitoken()
           ->composing()
           ->required(),
       "pcd folders(repeated)")(
          "pose_files",
          boost::program_options::value<std::vector<std::string>>()
              ->multitoken()
              ->composing()
              ->required(),
          "pose files(repeated)")(
          "map_folder",
          boost::program_options::value<std::string>()->required(),
          "map folder")(
          "zone_id", boost::program_options::value<int>()->required(),
          "zone id")("coordinate_type",
                     boost::program_options::value<std::string>()->required(),
                     "coordinate type: UTM or LTM")(
          "map_resolution_type",
          boost::program_options::value<std::string>()->required(),
          "map resolution type: single or multi")(
          "resolution",
          boost::program_options::value<float>()->default_value(0.125),
          "optional: resolution for single resolution generation, default: "
          "0.125");
  try {
    boost::program_options::store(
        boost::program_options::parse_command_line(argc, argv, desc), *vm);
    if (vm->count("help")) {
      AERROR << desc;
      return false;
    }
    boost::program_options::notify(*vm);
  } catch (std::exception& e) {
    AERROR << "Error: " << e.what() << " " << desc;
    return false;
  } catch (...) {
    AERROR << "Unknown error!";
    return false;
  }
  return true;
}

void VarianceOnline(double* mean, double* var, unsigned int* N, double x) {
  ++(*N);
  double value = (x - (*mean)) / (*N);
  double v1 = x - (*mean);
  (*mean) += value;
  double v2 = x - (*mean);
  (*var) = (((*N) - 1) * (*var) + v1 * v2) / (*N);
}

int main(int argc, char** argv) {
  FeatureXYPlane plane_extractor;

  boost::program_options::variables_map boost_args;
  if (!ParseCommandLine(argc, argv, &boost_args)) {
    AERROR << "Parse input command line failed.";
    return -1;
  }

  const std::vector<std::string> pcd_folder_paths =
      boost_args["pcd_folders"].as<std::vector<std::string>>();
  const std::vector<std::string> pose_files =
      boost_args["pose_files"].as<std::vector<std::string>>();
  if (pcd_folder_paths.size() != pose_files.size()) {
    AERROR << "The count of pcd folders is not equal pose files";
    return -1;
  }

  const std::string map_base_folder =
      boost_args["map_folder"].as<std::string>();
  bool use_plane_inliers_only = boost_args["use_plane_inliers_only"].as<bool>();
  // bool use_plane_fitting_ransac =
  // boost_args["use_plane_fitting_ransac"].as<bool>();
  const int zone_id = boost_args["zone_id"].as<int>();
  const std::string coordinate_type =
      boost_args["coordinate_type"].as<std::string>();
  if (strcasecmp(coordinate_type.c_str(), "UTM") != 0 &&
      strcasecmp(coordinate_type.c_str(), "LTM") != 0) {
    AERROR << "Coordinate type invalid. (UTM or LTM)";
    return -1;
  }
  const std::string map_resolution_type =
      boost_args["map_resolution_type"].as<std::string>();
  if (strcasecmp(map_resolution_type.c_str(), "single") != 0 &&
      strcasecmp(map_resolution_type.c_str(), "multi") != 0) {
    AERROR << "Map resolution type invalid. (single or multi)";
    return -1;
  }

  float single_resolution_map = boost_args["resolution"].as<float>();
  if (fabs(single_resolution_map - 0.03125) > 1e-8 &&
      fabs(single_resolution_map - 0.0625) > 1e-8 &&
      fabs(single_resolution_map - 0.125) < 1e-8 &&
      fabs(single_resolution_map - 0.25) < 1e-8 &&
      fabs(single_resolution_map - 0.5) < 1e-8 &&
      fabs(single_resolution_map - 1.0) < 1e-8 &&
      fabs(single_resolution_map - 2.0) < 1e-8 &&
      fabs(single_resolution_map - 4.0) < 1e-8 &&
      fabs(single_resolution_map - 8.0) < 1e-8 &&
      fabs(single_resolution_map - 16.0) < 1e-8) {
    AWARN << "Map resolution can only be: 0.03125, "
          << "0.0625, 0.125, 0.25, 0.5, 1.0, 2.0, "
          << "4.0, 8.0 or 16.0.";
  }

  const size_t num_trials = pcd_folder_paths.size();

  // load all poses
  AINFO << "Pcd folders are as follows:";
  for (size_t i = 0; i < num_trials; ++i) {
    AINFO << pcd_folder_paths[i];
  }
  std::vector<std::vector<Eigen::Affine3d>> ieout_poses(num_trials);
  std::vector<std::vector<double>> time_stamps(num_trials);
  std::vector<std::vector<unsigned int>> pcd_indices(num_trials);
  for (size_t i = 0; i < pose_files.size(); ++i) {
    apollo::localization::msf::velodyne::LoadPcdPoses(
        pose_files[i], &ieout_poses[i], &time_stamps[i], &pcd_indices[i]);
  }

  PyramidMapConfig conf("lossless_map");
  PyramidMap map(&conf);
  PyramidMapConfig& loss_less_config =
      static_cast<PyramidMapConfig&>(map.GetMapConfig());
  std::string map_folder_path = map_base_folder + "/lossless_map";
  apollo::cyber::common::EnsureDirectory(map_folder_path);
  map.SetMapFolderPath(map_folder_path);
  for (size_t i = 0; i < pcd_folder_paths.size(); ++i) {
    map.AddDataset(pcd_folder_paths[i]);
  }
  if (strcasecmp(map_resolution_type.c_str(), "single") == 0) {
    loss_less_config.SetSingleResolutions(single_resolution_map);
  } else {
    loss_less_config.SetMultiResolutions();
  }

  if (strcasecmp(coordinate_type.c_str(), "UTM") == 0) {
    loss_less_config.coordinate_type_ = "UTM";
  } else {
    loss_less_config.coordinate_type_ = "LTM";
    loss_less_config.map_range_ = apollo::localization::msf::Rect2D<double>(
        -1638400.0, -1638400.0, 1638400.0, 1638400.0);
  }

  // Output Config file
  char file_buf[1024];
  snprintf(file_buf, sizeof(file_buf), "%s/lossless_map/config.xml",
           map_base_folder.c_str());
  loss_less_config.Save(file_buf);

  snprintf(file_buf, sizeof(file_buf), "%s/lossless_map/config.txt",
           map_base_folder.c_str());
  FILE* file = fopen(file_buf, "a");

  if (file) {
    fprintf(file, "\n\nVeldoyne %uE\n", CAR_SENSOR_LASER_NUMBER);
    fprintf(file, "Map coordinate type: %s\n",
            loss_less_config.coordinate_type_.c_str());
    // if (loss_less_config.coordinate_type_ == "LTM") {
    //     fprintf(file, "Map origin longitude: %lf\n",
    //     loss_less_config._origin_longitude); fprintf(file, "Map origin
    //     latitude: %lf\n", loss_less_config._origin_latitude);
    // }
    fprintf(file, "Map compression: %d\n",
            loss_less_config.map_is_compression_);
    fprintf(file, "Map resolution: ");
    for (size_t i = 0; i < loss_less_config.map_resolutions_.size(); ++i) {
      fprintf(file, "%lf, ", loss_less_config.map_resolutions_[i]);
    }
    fprintf(file, "\nMap size: %lf %lf %lf %lf\n",
            loss_less_config.map_range_.GetMinX(),
            loss_less_config.map_range_.GetMinY(),
            loss_less_config.map_range_.GetMaxX(),
            loss_less_config.map_range_.GetMaxY());
    fprintf(file, "Map node size: %d x %d\n", loss_less_config.map_node_size_x_,
            loss_less_config.map_node_size_y_);
    fprintf(file, "Map row x col: \n");
    for (size_t i = 0; i < loss_less_config.map_resolutions_.size(); ++i) {
      fprintf(file, "%u x %u, ",
              MapNodeIndex::GetMapIndexRangeNorth(loss_less_config,
                                                  static_cast<unsigned int>(i)),
              MapNodeIndex::GetMapIndexRangeEast(loss_less_config,
                                                 static_cast<unsigned int>(i)));
    }
    fprintf(file, "Map image max intensity: %lf\n",
            loss_less_config.max_intensity_value_);
    fprintf(file, "Map image max var: %lf\n",
            loss_less_config.max_intensity_var_value_);
    fprintf(file, "PCD folders: \n");
    for (unsigned int trial = 0; trial < num_trials; ++trial) {
      fprintf(file, "%s\n", pcd_folder_paths[trial].c_str());
    }
    fclose(file);
  } else {
    AERROR << "Can't open file: " << file_buf;
  }

  PyramidMapNodePool lossless_map_node_pool(25, 8);
  lossless_map_node_pool.Initial(&loss_less_config);
  map.InitMapNodeCaches(12, 24);
  map.AttachMapNodePool(&lossless_map_node_pool);

  for (unsigned int trial = 0; trial < num_trials; ++trial) {
    for (unsigned int frame_idx = 0; frame_idx < ieout_poses[trial].size();
         ++frame_idx) {
      unsigned int trial_frame_idx = frame_idx;
      const std::vector<Eigen::Affine3d>& poses = ieout_poses[trial];
      apollo::localization::msf::velodyne::VelodyneFrame velodyne_frame;
      std::string pcd_file_path;
      std::ostringstream ss;
      ss << pcd_indices[trial][frame_idx];
      pcd_file_path = pcd_folder_paths[trial] + "/" + ss.str() + ".pcd";
      const Eigen::Affine3d& pcd_pose = poses[trial_frame_idx];
      apollo::localization::msf::velodyne::LoadPcds(
          pcd_file_path, trial_frame_idx, pcd_pose, &velodyne_frame, false);
      AINFO << "Loaded " << velodyne_frame.pt3ds.size()
            << "3D Points at Trial: " << trial << " Frame: " << trial_frame_idx
            << ".";

      unsigned int resolution_id = 0;
      unsigned int row = 0;
      unsigned int col = 0;
      for (size_t i = 0; i < velodyne_frame.pt3ds.size(); ++i) {
        Eigen::Vector3d& pt3d_local = velodyne_frame.pt3ds[i];
        unsigned char intensity = velodyne_frame.intensities[i];
        Eigen::Vector3d pt3d_global = velodyne_frame.pose * pt3d_local;
        MapNodeIndex map_node_index = MapNodeIndex::GetMapNodeIndex(
            loss_less_config, pt3d_global, resolution_id, zone_id);
        PyramidMapNode* map_node =
            dynamic_cast<PyramidMapNode*>(map.GetMapNodeSafe(map_node_index));
        map_node->GetCoordinate(pt3d_global, &col, &row);
        PyramidMapMatrix& map_matrix =
            dynamic_cast<PyramidMapMatrix&>(map_node->GetMapCellMatrix());
        map_matrix.SetIntensitySafe(intensity, row, col);
      }

      if (use_plane_inliers_only) {
        PclPointCloudPtrT pcl_pc = PclPointCloudPtrT(new PclPointCloudT);
        pcl_pc->resize(velodyne_frame.pt3ds.size());
        for (size_t i = 0; i < velodyne_frame.pt3ds.size(); ++i) {
          PclPointT& pt = pcl_pc->at(i);
          pt.x = static_cast<float>(velodyne_frame.pt3ds[i][0]);
          pt.y = static_cast<float>(velodyne_frame.pt3ds[i][1]);
          pt.z = static_cast<float>(velodyne_frame.pt3ds[i][2]);
          pt.intensity = static_cast<float>(velodyne_frame.intensities[i]);
        }

        plane_extractor.ExtractXYPlane(pcl_pc);
        PclPointCloudPtrT& plane_pc = plane_extractor.GetXYPlaneCloud();

        for (unsigned int k = 0; k < plane_pc->size(); ++k) {
          const PclPointT& plane_pt = plane_pc->at(k);
          Eigen::Vector3d pt3d_local_double;
          pt3d_local_double[0] = plane_pt.x;
          pt3d_local_double[1] = plane_pt.y;
          pt3d_local_double[2] = plane_pt.z;
          unsigned char intensity =
              static_cast<unsigned char>(plane_pt.intensity);
          Eigen::Vector3d pt3d_global = velodyne_frame.pose * pt3d_local_double;
          float ground_altitude = static_cast<float>(pt3d_global[2]);
          MapNodeIndex map_node_index = MapNodeIndex::GetMapNodeIndex(
              loss_less_config, pt3d_global, resolution_id, zone_id);
          PyramidMapNode* map_node =
              dynamic_cast<PyramidMapNode*>(map.GetMapNodeSafe(map_node_index));
          map_node->GetCoordinate(pt3d_global, &col, &row);
          PyramidMapMatrix& map_matrix =
              dynamic_cast<PyramidMapMatrix&>(map_node->GetMapCellMatrix());
          map_matrix.SetIntensitySafe(intensity, row, col);
          map_matrix.SetGroundAltitudeSafe(ground_altitude, row, col);
          map_node->SetIsChanged(true);
        }
      }
    }
  }

  // Compute the ground height offset
  double mean_height_diff = 0;
  double var_height_diff = 0;
  unsigned int count_height_diff = 0;
  for (unsigned int trial = 0; trial < num_trials; ++trial) {
    for (unsigned int i = 0; i < ieout_poses[trial].size(); ++i) {
      const Eigen::Affine3d& ieout_pose = ieout_poses[trial][i];
      const Eigen::Vector3d& pt3d = ieout_pose.translation();
      unsigned int resolution_id = 0;
      unsigned int row = 0;
      unsigned int col = 0;
      MapNodeIndex map_node_index = MapNodeIndex::GetMapNodeIndex(
          loss_less_config, pt3d, resolution_id, zone_id);
      PyramidMapNode* map_node =
          dynamic_cast<PyramidMapNode*>(map.GetMapNodeSafe(map_node_index));
      map_node->GetCoordinate(pt3d, &col, &row);
      PyramidMapMatrix& map_matrix =
          dynamic_cast<PyramidMapMatrix&>(map_node->GetMapCellMatrix());

      if (use_plane_inliers_only) {
        // Use the altitudes from layer 0 (layer 1 internally in the Map).
        std::vector<unsigned int> layer_counts;
        const unsigned int* ground_count =
            map_matrix.GetGroundCountSafe(row, col);
        if (!ground_count) {
          AERROR << "No ground layer, skip.";
          continue;
        }
        if (*ground_count > 0) {
          // std::vector<float> layer_alts;
          // map.GetAltSafe(pt3d, zone_id, resolution_id, &layer_alts);
          const float* ground_altitude =
              map_matrix.GetGroundAltitudeSafe(row, col);
          if (!ground_altitude) {
            AERROR << "No ground points, skip.";
            continue;
          }
          // float alt = layer_alts[layer_id];
          double height_diff = pt3d[2] - *ground_altitude;
          VarianceOnline(&mean_height_diff, &var_height_diff,
                         &count_height_diff, height_diff);
        }
      } else {
        // Use the altitudes from all layers
        const unsigned int* count = map_matrix.GetCountSafe(row, col);
        if (*count > 0) {
          const float* alt = map_matrix.GetAltitudeSafe(row, col);
          double height_diff = pt3d[2] - *alt;
          VarianceOnline(&mean_height_diff, &var_height_diff,
                         &count_height_diff, height_diff);
        }
      }
    }
  }

  map.GetMapConfig().map_ground_height_offset_ =
      static_cast<float>(mean_height_diff);
  std::string config_path = map.GetMapConfig().map_folder_path_ + "/config.xml";
  map.GetMapConfig().Save(config_path);
  ADEBUG << "Mean: " << mean_height_diff << ", Var: " << var_height_diff << ".";
  return 0;
}
