/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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
#include <fstream>
#include <functional>
#include <map>
#include <ostream>
#include <string>

#include "Eigen/Core"
#include "modules/common/util/file.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/lib/base/timer.h"
#include "modules/perception/obstacle/common/file_system_util.h"
#include "modules/perception/obstacle/common/pose_util.h"
#include "modules/perception/obstacle/onboard/obstacle_perception.h"
#include "pcl/io/pcd_io.h"

DECLARE_string(flagfile);
DEFINE_string(lidar_path, "./modules/perception/tool/export_sensor_data/lidar/",
              "lidar path");
DEFINE_string(radar_path, "./modules/perception/tool/export_sensor_data/radar/",
              "radar path");
DEFINE_bool(enable_global_offset, true,
            "enable global offset for benchmark evaluation");
DEFINE_string(main_sensor, "velodyne_64", "main publish sensor");

namespace apollo {
namespace perception {

struct SensorFile {
  SensorFile() {}
  SensorFile(std::string k, std::string p, double t = 0.0) {
    sensor_key = k, file_path = p, timestamp = t;
  }
  std::string sensor_key;
  std::string file_path;
  double timestamp;
  friend std::ostream& operator<<(std::ostream& out,
                                  const SensorFile& sensor_file) {
    out << "sensor_key: " << sensor_file.sensor_key
        << " file_path: " << sensor_file.file_path << std::setprecision(16)
        << " timestamp: " << sensor_file.timestamp;
    return out;
  }
};

struct SensorFilesSource {
  SensorFilesSource() {}
  SensorFilesSource(std::string folder, std::string ext) {
    folder_path = folder, extension = ext;
  }
  std::string folder_path;
  std::string extension;
};

bool LoadRadarProto(const std::string& filepath, ContiRadar* radar_obs_proto) {
  std::fstream input(filepath.c_str(), std::ios::in | std::ios::binary);
  if (!radar_obs_proto->ParseFromIstream(&input)) {
    AERROR << "Parsing error: " << filepath;
    return false;
  }
  input.close();
  return true;
}

bool LoadOdometry(const std::string& filepath, Eigen::Vector3f* velocity) {
  std::ifstream fin(filepath.c_str());
  if (!fin.is_open()) {
    AINFO << "File " << filepath << " is not exit";
    return false;
  }
  double timestamp = 0.0;
  int frame_id;
  fin >> frame_id >> timestamp >> (*velocity)(0) >> (*velocity)(1) >>
      (*velocity)(2);
  bool state = true;
  // if (!fin.good()) {
  //   state = false;
  //   AERROR << "Failed to read odometry: " << filepath;
  // }
  fin.close();
  return state;
}

class SequentialPerceptionTest {
 public:
  typedef std::function<bool(const std::string&,
                             std::shared_ptr<SensorRawFrame>*)>
      SensorFrameReconstructor;

  SequentialPerceptionTest() : init_offset_(false) {}

  ~SequentialPerceptionTest() {}

  bool Init() {
    if (!obstacle_perception_.Init()) {
      AERROR << "Failed to init ObstaclePerception";
      return false;
    }
    if (!FLAGS_enable_global_offset) {
      init_offset_ = true;
      global_offset_ = Eigen::Vector3d(0, 0, 0);
    } else {
      init_offset_ = false;
    }
    sensor_frame_reconstructor_["pcd"] = std::bind(
        &SequentialPerceptionTest::ReconstructPointcloudSensorRawFrame, this,
        std::placeholders::_1, std::placeholders::_2);
    sensor_frame_reconstructor_["radar"] =
        std::bind(&SequentialPerceptionTest::ReconstructRadarSensorRawFrame,
                  this, std::placeholders::_1, std::placeholders::_2);
    sensors_files_sources_ = {
        {"pcd", SensorFilesSource(FLAGS_lidar_path, "pcd")},
        {"radar", SensorFilesSource(FLAGS_radar_path, "radar")}};

    return true;
  }

  bool ReconstructSensorRawFrame(const std::string& file_path,
                                 std::shared_ptr<SensorRawFrame>* frame) {
    std::string type = file_path.substr(file_path.find_last_of(".") + 1);
    auto find_res = sensor_frame_reconstructor_.find(type);
    if (find_res == sensor_frame_reconstructor_.end()) {
      AERROR << "The file type: " << type << ", is not supported";
      return false;
    }
    return (find_res->second)(file_path, frame);
  }

  bool ReconstructPointcloudSensorRawFrame(
      const std::string& file_path, std::shared_ptr<SensorRawFrame>* frame) {
    std::string type = file_path.substr(file_path.find_last_of(".") + 1);
    if (type != "pcd") {
      AERROR
          << "reconstruct_pointcloud_sensor_raw_frame can only handle pcd file";
      AERROR << "Pass file: " << file_path;
      return false;
    }
    // static Eigen::Matrix4d pose_original;
    AINFO << "Process pcd";
    // read pose and timestamp
    Eigen::Matrix4d pose;
    int frame_id = 0;
    double timestamp = 0.0;
    std::string pose_filename =
        FLAGS_lidar_path + "/" +
        file_path.substr(0, file_path.find_last_of('.')) + ".pose";
    if (!ReadPoseFile(pose_filename, &pose, &frame_id, &timestamp)) {
      AERROR << "Pose file not exits: " << pose_filename;
      return false;
    }
    std::cout << "read pose file: " << std::endl;
    std::cout << pose << std::endl;
    if (!init_offset_) {
      global_offset_ = pose.col(3).head(3);
      obstacle_perception_.SetGlobalOffset(global_offset_);
      init_offset_ = true;
    }
    pose.col(3).head(3) -= global_offset_;

    // pose_original = pose;
    // read pcd
    pcl::PointCloud<pcl_util::PointXYZIT>::Ptr cloud_raw(
        new pcl::PointCloud<pcl_util::PointXYZIT>);
    std::string pcd_filename =
        FLAGS_lidar_path + "/" +
        file_path.substr(0, file_path.find_last_of('.')) + ".pcd";
    AINFO << "pcd file_name: " << pcd_filename;
    pcl::io::loadPCDFile<pcl_util::PointXYZIT>(pcd_filename, *cloud_raw);
    pcl_util::PointCloudPtr cloud(new pcl_util::PointCloud);
    for (size_t i = 0; i < cloud_raw->points.size(); ++i) {
      pcl_util::Point p;
      p.x = cloud_raw->points[i].x;
      p.y = cloud_raw->points[i].y;
      p.z = cloud_raw->points[i].z;
      p.intensity = cloud_raw->points[i].intensity;
      cloud->points.push_back(p);
    }

    // pose = pose * _velodyne2novatel_ex;
    frame->reset(new VelodyneRawFrame);
    VelodyneRawFrame* velodyne_frame =
        dynamic_cast<VelodyneRawFrame*>(frame->get());
    velodyne_frame->timestamp_ = timestamp;
    velodyne_frame->pose_ = pose;
    velodyne_frame->sensor_type_ = VELODYNE_64;
    velodyne_frame->cloud_ = cloud;
    AINFO << "velo frame sensor_type_ = " << (*frame)->sensor_type_;
    return true;
  }

  bool ReconstructRadarSensorRawFrame(const std::string& file_path,
                                      std::shared_ptr<SensorRawFrame>* frame) {
    std::string type = file_path.substr(file_path.find_last_of(".") + 1);
    if (type != "radar") {
      AERROR << "reconstruct_radar_sensor_raw_frame can only handle "
             << "radar file";
      AERROR << "Pass file: " << file_path;
      return false;
    }

    // read pose
    Eigen::Matrix4d pose;
    int frame_id = 0;
    double timestamp = 0.0;
    std::string pose_filename =
        FLAGS_radar_path + "/" +
        file_path.substr(0, file_path.find_last_of('.')) + ".pose";
    if (!ReadPoseFile(pose_filename, &pose, &frame_id, &timestamp)) {
      AERROR << "Pose file not exits: " << pose_filename;
      return false;
    }
    // read radar obstacle
    ContiRadar radar_obs_proto;
    std::string radar_filename =
        FLAGS_radar_path + "/" +
        file_path.substr(0, file_path.find_last_of('.')) + ".radar";
    if (!LoadRadarProto(radar_filename, &radar_obs_proto)) {
      AERROR << "Failed to load " << radar_filename;
      return false;
    }
    AINFO << "radar_obs_proto size: " << radar_obs_proto.contiobs().size();
    // read host vehicle velocity
    Eigen::Vector3f velocity;
    std::string odometry_filename =
        FLAGS_radar_path + "/" +
        file_path.substr(0, file_path.find_last_of('.')) + ".velocity";
    if (!LoadOdometry(odometry_filename, &velocity)) {
      AERROR << "Failed to load " << odometry_filename;
      return false;
    }

    if (!init_offset_) {
      global_offset_ = pose.col(3).head(3);
      obstacle_perception_.SetGlobalOffset(global_offset_);
      init_offset_ = true;
    }
    pose.col(3).head(3) -= global_offset_;

    frame->reset(new RadarRawFrame);
    RadarRawFrame* radar_frame = dynamic_cast<RadarRawFrame*>(frame->get());
    radar_frame->timestamp_ = timestamp;
    radar_frame->pose_ = pose;
    radar_frame->sensor_type_ = RADAR;
    radar_frame->raw_obstacles_ = radar_obs_proto;
    radar_frame->car_linear_speed_ = velocity;
    AINFO << "radar frame sensor_type_ = " << (*frame)->sensor_type_;
    return true;
  }

  void GetSequentialSensorsDataFiles(std::vector<SensorFile>* sensors_files) {
    PERF_BLOCK_START();
    std::map<std::string, std::vector<SensorFile>> sensors_files_lists;
    size_t total_sensor_files_num = 0;
    for (const auto& sensor_files_source : sensors_files_sources_) {
      std::string sensor_key = sensor_files_source.first;
      const SensorFilesSource& source = sensor_files_source.second;
      AINFO << "source folder_path: " << source.folder_path;
      if (!apollo::common::util::DirectoryExists(source.folder_path)) {
        AWARN << "No sensor files source: " << sensor_key;
        continue;
      }
      std::string ext = std::string(".") + source.extension;
      std::vector<std::string> files_list;
      GetFileNamesInFolderById(source.folder_path, ext, &files_list);
      // get time stamp
      std::vector<SensorFile>& sensor_files_list =
          sensors_files_lists[sensor_key];
      sensor_files_list.reserve(files_list.size());
      for (const auto& file_name : files_list) {
        std::string pose_file =
            source.folder_path + "/" +
            file_name.substr(0, file_name.find_last_of('.')) + ".pose";
        FILE* fin = fopen(pose_file.c_str(), "rt");
        if (fin == nullptr) {
          AWARN << "pose file: " << pose_file
                << " not found. sensor key: " << sensor_key;
          continue;
        }
        int seq = 0;
        double timestamp = 0.0;
        if (fscanf(fin, "%d %lf", &seq, &timestamp) != 2) {
          AERROR << "parse pose file: " << pose_file
                 << " failed. sensor key: " << sensor_key;
          fclose(fin);
          continue;
        }
        sensor_files_list.push_back(
            SensorFile(sensor_key, file_name, timestamp));
        fclose(fin);
      }
      total_sensor_files_num += sensor_files_list.size();
      AINFO << "sensor_key: " << sensor_key
            << " files num: " << sensor_files_list.size();
    }
    sensors_files->clear();
    sensors_files->reserve(total_sensor_files_num);
    for (const auto& sensor_files_list : sensors_files_lists) {
      sensors_files->insert(sensors_files->end(),
                            sensor_files_list.second.begin(),
                            sensor_files_list.second.end());
    }
    auto compare = [](const SensorFile& lhs, const SensorFile& rhs) {
      return lhs.timestamp < rhs.timestamp;
    };
    std::sort(sensors_files->begin(), sensors_files->end(), compare);
    PERF_BLOCK_END("get_sequential_sensors_data_files");
  }

  void Run(SensorRawFrame* frame) {
    std::vector<ObjectPtr> fused_objs;
    obstacle_perception_.Process(frame, &fused_objs);
  }

  const Eigen::Vector3d& GetGlobalOffset() { return global_offset_; }

 private:
  // reconstruct sensor raw frame function
  std::map<std::string, SensorFrameReconstructor> sensor_frame_reconstructor_;
  std::map<std::string, SensorFilesSource> sensors_files_sources_;

  ObstaclePerception obstacle_perception_;
  Eigen::Vector3d global_offset_;
  bool init_offset_;
};

}  // namespace perception
}  // namespace apollo

int main(int argc, char** argv) {
  FLAGS_flagfile =
      "./modules/perception/tool/offline_visualizer_tool/conf/"
      "offline_sequential_obstacle_perception_test.flag";
  google::ParseCommandLineFlags(&argc, &argv, true);

  apollo::perception::SequentialPerceptionTest test;
  test.Init();
  std::vector<apollo::perception::SensorFile> sensors_files;
  test.GetSequentialSensorsDataFiles(&sensors_files);
  AINFO << "=============================" << std::endl;
  for (size_t i = 0; i < sensors_files.size(); i++) {
    AINFO << sensors_files[i];
  }
  AINFO << "Total sensors files num: " << sensors_files.size();
  for (size_t i = 0; i < sensors_files.size(); ++i) {
    AINFO << "Process frame " << sensors_files[i];
    std::shared_ptr<apollo::perception::SensorRawFrame> raw_frame(
        new apollo::perception::SensorRawFrame);
    test.ReconstructSensorRawFrame(sensors_files[i].file_path, &raw_frame);
    test.Run(raw_frame.get());
  }
  return 0;
}
