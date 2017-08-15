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
#include <iomanip>
#include <sstream>
#include <vector>
#include <string>
#include <pcl/io/pcd_io.h>

#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/lib/pcl_util/pcl_types.h"
#include "modules/perception/obstacle/common/file_system_util.h"
#include "modules/perception/obstacle/onboard/lidar_process.h"
#include "modules/perception/obstacle/lidar/visualizer/opengl_visualizer/opengl_visualizer.h"
#include "modules/perception/obstacle/lidar/visualizer/opengl_visualizer/frame_content.h"

DECLARE_string(flagfile);
DECLARE_bool(enable_visualization);
DECLARE_string(config_manager_path);
DEFINE_string(pcd_path, "./pcd/", "pcd path");
DEFINE_string(pose_path, "./pose/", "pose path");
DEFINE_bool(save_obstacles, false, "save obstacles to file");


namespace apollo {
namespace perception {

DEFINE_string(output_path, "./output/", "output path");
DEFINE_int32(start_frame, 1, "start frame");

template <typename T>
void QuaternionToRotationMatrix(const T * quat, T * R) {
    T x2 = quat[0] * quat[0];
    T xy = quat[0] * quat[1];
    T rx = quat[3] * quat[0];
    T y2 = quat[1] * quat[1];
    T yz = quat[1] * quat[2];
    T ry = quat[3] * quat[1];
    T z2 = quat[2] * quat[2];
    T zx = quat[2] * quat[0];
    T rz = quat[3] * quat[2];
    T r2 = quat[3] * quat[3];
    R[0] = r2 + x2 - y2 - z2;         // fill diagonal terms
    R[4] = r2 - x2 + y2 - z2;
    R[8] = r2 - x2 - y2 + z2;
    R[3] = 2 * (xy + rz);             // fill off diagonal terms
    R[6] = 2 * (zx - ry);
    R[7] = 2 * (yz + rx);
    R[1] = 2 * (xy - rz);
    R[2] = 2 * (zx + ry);
    R[5] = 2 * (yz - rx);
}

bool ReadPoseFile(const std::string& filename, Eigen::Matrix4d& pose,
    int& frame_id, double& time_stamp) {
    std::ifstream ifs(filename.c_str());
    if (!ifs.is_open()) {
        std::cerr << "Failed to open file " << filename << std::endl;
        return false;
    }
    char buffer[1024];
    ifs.getline(buffer, 1024);
    int id = 0;
    double time_samp = 0;
    double quat[4];
    double matrix3x3[9];
    sscanf(buffer, "%d %lf %lf %lf %lf %lf %lf %lf %lf", &id, &(time_samp),
            &(pose(0, 3)), &(pose(1, 3)), &(pose(2, 3)),
            &(quat[0]), &(quat[1]), &(quat[2]), &(quat[3]));
    QuaternionToRotationMatrix<double>(quat, matrix3x3);

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            pose(i, j) = matrix3x3[i * 3 + j];
        }
    }

    frame_id = id;
    time_stamp = time_samp;
    return true;
}

class OfflineLidarPerceptionTool {
public:
    OfflineLidarPerceptionTool() {
        config_manager_ = NULL;
    };

    ~OfflineLidarPerceptionTool() = default;

    bool Init(bool use_visualization = false) {
        FLAGS_config_manager_path = "tool/offline_visualizer_tool/conf/config_manager.config";
        config_manager_ = Singleton<ConfigManager>::Get();
       
        if (config_manager_ == NULL || !config_manager_->Init()) {
            AERROR << "failed to init ConfigManager";
            return false;
        }

        lidar_process_.reset(new LidarProcess());

        if (!lidar_process_->Init()) {
          AERROR << "failed to init lidar_process.";
          return false;
        }

        output_dir_ = FLAGS_output_path;

        if (use_visualization) {
            visualizer_.reset(new OpenglVisualizer());
            if(!visualizer_->Init()) {
                AERROR<<"init visialuzer failed"<<std::endl;
            }; 
        }
        return true;
    }

    void Run(const std::string& pcd_path, const std::string pose_path) {

        std::string pcd_folder = pcd_path;
        std::string pose_folder = pose_path;
        std::vector<std::string> pcd_file_names;
        std::vector<std::string> pose_file_names;
        AINFO << "starting to run"; 
        GetFileNamesInFolderById(pose_folder, ".pose", &pose_file_names);
        GetFileNamesInFolderById(pcd_folder, ".pcd", &pcd_file_names);
        AINFO<<" pose size " << pose_file_names.size();
        AINFO<<" pcd size " << pcd_file_names.size(); 
        if (pose_file_names.size() != pcd_file_names.size()) {
            AERROR << "pcd file number does not match pose file number";
            return;
        }
        double time_stamp = 0.0;
        int start_frame = FLAGS_start_frame;
        AINFO << "starting frame is "<<start_frame;
        sleep(1);
        for (size_t i = start_frame; i < pcd_file_names.size(); i++) {
            AINFO << "***************** Frame " << i << " ******************";
            std::ostringstream oss;
            pcl_util::PointCloudPtr cloud(new pcl_util::PointCloud);
            AINFO << "load pcd file from file path" << pcd_folder + pcd_file_names[i];
            pcl::io::loadPCDFile<pcl_util::Point>(pcd_folder + pcd_file_names[i], *cloud);

            //read pose
            Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
            int frame_id = -1;
            if (!ReadPoseFile(pose_folder + pose_file_names[i],
                pose, frame_id, time_stamp)) {
                std::cout << "Failed to read file " << pose_file_names[i] << "\n";
                return ;
            }
            
            std::shared_ptr<Eigen::Matrix4d> velodyne_trans = std::make_shared<Eigen::Matrix4d>(pose);
            lidar_process_->Process(time_stamp, cloud, velodyne_trans);

            std::vector<ObjectPtr> result_objects = lidar_process_->GetObjects();
            const pcl_util::PointIndicesPtr roi_indices = lidar_process_->GetROIIndices();
            
            if (visualizer_) {
                pcl_util::PointDCloudPtr transformed_cloud(new pcl_util::PointDCloud);
                transform_perception_cloud(cloud, pose, transformed_cloud);
                AERROR<<"transformed cloud size is "<<transformed_cloud->size();

                pcl_util::PointIndices roi_indices_1;
                FrameContent content;
                content.SetLidarPose(pose);
                content.SetLidarCloud(cloud);
                content.SetTrackedObjects(result_objects);
                visualizer_->UpdateCameraSystem(&content);
                visualizer_->Render(content);
            }
            AINFO << "finish pc";
            
            if (FLAGS_save_obstacles) {
                oss << std::setfill('0') << std::setw(6) << i;
                std::string filename = FLAGS_output_path + oss.str() + ".txt";
                SaveObstacleInformaton(result_objects, filename);
            }
        }
    }

    void SaveObstacleInformaton(
        std::vector<ObjectPtr>& objects, std::string file_name) {
        std::cout << "save " << objects.size() << " objects to "
            << file_name << std::endl;
        std::fstream test_file;
        test_file.open(file_name.c_str(), std::fstream::out);
        std::cout << "sort each object \n";
        for (size_t i = 0; i < objects.size(); ++i) {
            std::cout << i << " ";
            ObjectPtr obs = objects[i];
            PolygonDType& cloud = obs->polygon;
            if (cloud.size() == 0) {
                continue;
            }
            for (size_t j = 0; j < cloud.points.size() - 1; ++j) {
                for (size_t k = j + 1; k < cloud.points.size(); ++k) {
                    pcl_util::PointD p1 = cloud.points[j];
                    pcl_util::PointD p2 = cloud.points[k];
                    bool need_change = false;
                    if (static_cast<long long>(10000 * p1.x) ==
                        static_cast<long long>(10000 * p2.x)) {
                        if (static_cast<long long>(10000 * p1.y)
                                == static_cast<long long>(10000 * p2.y)) {
                            need_change = static_cast<long long>(
                                    10000 * p1.z) > static_cast<long long>(10000 * p2.z);
                        } else {
                            need_change = static_cast<long long>(
                                    10000 * p1.y) > static_cast<long long>(10000 * p2.y);
                        }
                    } else {
                        need_change = static_cast<long long>(
                                10000 * p1.x) > static_cast<long long>(10000 * p2.x);
                    }
                    if (need_change) {
                        pcl_util::PointD tmp = cloud.points[j];
                        cloud.points[j] = cloud.points[k];
                        cloud.points[k] = tmp;
                    }
                }
            }
        }
        std::cout << "\nsort all objects\n";
        for (int i = 0; i < (int)objects.size() - 1; ++i) {
            std::cout << "\r" << i;
            for (size_t j = i + 1; j < objects.size(); ++j) {
                bool need_change = false;
                ObjectPtr obs_i = objects[i];
                ObjectPtr obs_j = objects[j];
                PolygonDType& cloud_i = obs_i->polygon;
                PolygonDType& cloud_j = obs_j->polygon;
                if (cloud_i.points.size() == cloud_j.points.size()) {
                    if (cloud_i.points.size() == 0) {
                        continue;
                    }
                    pcl_util::PointD p1 = cloud_i.points[0];
                    pcl_util::PointD p2 = cloud_j.points[0];
                    if (static_cast<long long>(10000 * p1.x) ==
                        static_cast<long long>(10000 * p2.x)) {
                        if (static_cast<long long>(10000 * p1.y)
                                == static_cast<long long>(10000 * p2.y)) {
                            need_change = static_cast<long long>(
                                    10000 * p1.z) > static_cast<long long>(10000 * p2.z);
                        } else {
                            need_change = static_cast<long long>(
                                    10000 * p1.y) > static_cast<long long>(10000 * p2.y);
                        }
                    } else {
                        need_change = static_cast<long long>(
                                10000 * p1.x) > static_cast<long long>(10000 * p2.x);
                    }
                } else {
                    need_change = cloud_i.points.size() > cloud_j.points.size();
                }
                if (need_change) {
                    ObjectPtr obj_tmp = objects[i];
                    objects[i] = objects[j];
                    objects[j] = obj_tmp;
                }
            }
        }
        std::cout << "\nstart to save objects\n";
        for (size_t i = 0; i < objects.size(); ++i) {
            ObjectPtr obs = objects[i];
            if (obs->polygon.size() <= 0) {
                std::cout << "Find object with empty convex hull "
                    << obs->cloud->size() << "\n";
                continue;
            }
            // id
            test_file << obs->track_id << "  ";
            // test_file << obs.get_idx() << " ";
            // position
            test_file << std::setprecision(4) << std::fixed << obs->center[0]
                << " " << std::setprecision(4) << std::fixed << obs->center[1]
                << " " << std::setprecision(4) << std::fixed << obs->center[2] << " ";
            // theta
            double theta = 0;
            Eigen::Vector3d dir = obs->direction;
            if (fabs(dir[0]) < DBL_MIN) {
                if (dir[1] > 0) {
                    theta = M_PI / 2.0;
                } else {
                    theta = - M_PI / 2.0;
                }
            } else {
                theta = atan(dir[1] / dir[0]);
            }
            test_file << std::setprecision(4) << theta << std::fixed << " ";

            // velocity
            test_file << std::setprecision(4) << std::fixed << obs->velocity[0] << std::fixed << " "
                    << std::setprecision(4) << obs->velocity[1] << std::fixed << " "
                    << std::setprecision(4) << obs->velocity[2] << std::fixed << " ";

            // size
            test_file << std::setprecision(4) << std::fixed << obs->length
                << std::fixed << " " << std::setprecision(4) << obs->width
                << std::fixed << " " << std::setprecision(4) << obs->height << std::fixed << " ";
            // dir
            // test_file << std::setprecision(4) << obs.get_dir()[0] << fixed << " " << std::setprecision(4) << obs.get_dir()[1] << fixed << " ";
            // polygon
            PolygonDType& cloud = obs->polygon;
            test_file << cloud.size() << " ";
            for (size_t j = 0; j < cloud.size(); ++j) {
                test_file << std::setprecision(4) << std::fixed << cloud.points[j].x << std::fixed << " "
                        << std::setprecision(4) << cloud.points[j].y << std::fixed << " "
                        << std::setprecision(4) << cloud.points[j].z << std::fixed << " ";
            }
            // type
            std::string type = "unknow";
            if (obs->type == UNKNOWN) {
                type = "unknow";
            } else if (obs->type == PEDESTRIAN) {
                type = "pedestrian";
            } else if (obs->type == VEHICLE) {
                type = "car";
            }
            test_file << type << " ";
            test_file << std::endl;
        }
    }

protected:
    ConfigManager* config_manager_;
    std::unique_ptr<LidarProcess> lidar_process_;
    std::unique_ptr<OpenglVisualizer> visualizer_;
    std::string output_dir_;
    std::string pose_dir_;
    std::string label_file_;
};

}  // namespace perception
}  // namespace apollo


int main(int argc, char* argv[]) {
    FLAGS_flagfile = "./modules/perception/tool/offline_visualizer_tool/conf/offline_lidar_perception_test.flag";
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    apollo::perception::OfflineLidarPerceptionTool tool;
    tool.Init(FLAGS_enable_visualization);
    tool.Run(FLAGS_pcd_path, FLAGS_pose_path);
    return 0;
}
