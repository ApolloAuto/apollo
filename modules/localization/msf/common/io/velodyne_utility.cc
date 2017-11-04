#include <pcl/io/pcd_io.h>
#include <yaml-cpp/yaml.h>
#include "./pcl_point_types.h"
#include "./velodyne_utility.h"

namespace apollo {
namespace localization {
namespace msf {
namespace velodyne {

void load_pcd_poses(std::string file_path, std::vector<Eigen::Affine3d>& poses,
        std::vector<double>& timestamps) {
    std::vector<unsigned int> pcd_indices;
    load_pcd_poses(file_path, poses, timestamps, pcd_indices);
}

void load_pcd_poses(std::string file_path, std::vector<Eigen::Affine3d>& poses,
                    std::vector<double>& timestamps, 
                    std::vector<unsigned int>& pcd_indices) {
    poses.clear();
    timestamps.clear();
    pcd_indices.clear();

    FILE * file = fopen(file_path.c_str(), "r");
    if (file) {
        unsigned int index;
        double timestamp;
        double x, y, z;
        double qx, qy, qz, qr;
        int size = 9;
        while ((size = fscanf(file, "%u %lf %lf %lf %lf %lf %lf %lf %lf\n", &index,
                        &timestamp, &x, &y, &z, &qx, &qy, &qz, &qr)) == 9) {

            Eigen::Translation3d trans(Eigen::Vector3d(x, y, z));
            Eigen::Quaterniond quat(qr, qx, qy, qz);
            // numerical::Quaternion<double> quat = numerical::Quaternion<double>(qx, qy, qz, qr);
            // numerical::Transform<double> pose = numerical::Transform<double>(quat, trans);
            poses.push_back(trans * quat);
            timestamps.push_back(timestamp);
            pcd_indices.push_back(index);
        }
        fclose(file);
    }
    else {
        std::cerr << "Can't open file to read: " << file_path << std::endl;
    }
}

// void save_pcd_poses(std::string file_path,
//    const std::vector<numerical::Transform<double> >& poses,
//    const std::vector<double>& timestamps) {
//   FILE * file = fopen(file_path.c_str(), "w");
//   if (file) {
//     for (size_t i = 0; i < poses.size(); ++i) {
//       const numerical::Transform<double>& pose = poses[i];
//       double timestamp = timestamps[i];
//       fprintf(file, "%lu %lf %lf %lf %lf %lf %lf %lf %lf\n", i,
//           timestamp, pose.get_translation()[0], pose.get_translation()[1],
//           pose.get_translation()[2], pose.get_quaternion().get_x(),
//           pose.get_quaternion().get_y(), pose.get_quaternion().get_z(),
//           pose.get_quaternion().get_r());
//     }
//     fclose(file);
//   }
//   else {
//     std::cerr << "Can't open file to read: " << file_path << std::endl;
//   }
// }

bool load_extrinsic(std::string file_path, Eigen::Affine3d& extrinsic) {
  YAML::Node config = YAML::LoadFile(file_path);
  if (config["transform"]) {
    if (config["transform"]["translation"]) {
      extrinsic.translation()(0)
          = config["transform"]["translation"]["x"].as<double>();
      extrinsic.translation()(1)
          = config["transform"]["translation"]["y"].as<double>();
      extrinsic.translation()(2)
          = config["transform"]["translation"]["z"].as<double>();
      if (config["transform"]["rotation"]) {
        double qx = config["transform"]["rotation"]["x"].as<double>();
        double qy = config["transform"]["rotation"]["y"].as<double>();
        double qz = config["transform"]["rotation"]["z"].as<double>();
        double qw = config["transform"]["rotation"]["w"].as<double>();
        extrinsic.linear()
            = Eigen::Quaterniond(qw, qx, qy, qz).toRotationMatrix();
        return true;
      }
    }
  }
  return false;
}

void load_pcds(std::string file_path, 
        unsigned int frame_index,
        const Eigen::Affine3d& pose,
        VelodyneFrame& velodyne_frame, 
        bool is_global) {
    velodyne_frame.frame_index = frame_index;
    velodyne_frame.pose = pose;
    load_pcds(file_path, frame_index, pose, velodyne_frame.pt3ds, 
              velodyne_frame.intensities, is_global);
}

void load_pcds(std::string file_path, 
    unsigned int frame_index,
    const Eigen::Affine3d& pose,
    std::vector<Eigen::Vector3d>& pt3ds, 
    std::vector<unsigned char>& intensities,
    bool is_global) {    
  Eigen::Affine3d pose_inv = pose.inverse();
  pcl::PointCloud<PointXYZIT>::Ptr cloud(new pcl::PointCloud<PointXYZIT>);
  if (pcl::io::loadPCDFile(file_path, *cloud) >= 0) {
    if (cloud->height == 1 || cloud->width == 1) {
      std::cerr << "Un-organized-point-cloud" << std::endl;
      for (unsigned int i = 0; i < cloud->size(); ++i) {
        Eigen::Vector3d pt3d;
        pt3d[0] = (*cloud)[i].x;
        pt3d[1] = (*cloud)[i].y;
        pt3d[2] = (*cloud)[i].z;

        if (pt3d[0] == pt3d[0] && pt3d[1] == pt3d[1] && pt3d[2] == pt3d[2]) {
          Eigen::Vector3d pt3d_local;
          if (is_global) {
            pt3d_local = pose_inv * pt3d;
          }
          else {
            pt3d_local = pt3d;
          }
          unsigned char intensity
              = static_cast<unsigned char>((*cloud)[i].intensity);
          pt3ds.push_back(pt3d_local);
          intensities.push_back(intensity);
        }
      }
    } else {
      for (unsigned int h = 0; h < cloud->height; ++h) {
        for (unsigned int w = 0; w < cloud->width; ++w) {
          double x = cloud->at(w, h).x;
          double y = cloud->at(w, h).y;
          double z = cloud->at(w, h).z;
          Eigen::Vector3d pt3d(x, y, z);
            if (pt3d[0] == pt3d[0] && pt3d[1] == pt3d[1]
                && pt3d[2] == pt3d[2]) {
              Eigen::Vector3d pt3d_local;
              if (is_global) {
                pt3d_local = pose_inv * pt3d;
              }
              else {
                pt3d_local = pt3d;
              }
              unsigned char intensity
                  = static_cast<unsigned char>(cloud->at(w, h).intensity);
              pt3ds.push_back(pt3d_local);
              intensities.push_back(intensity);
            }
          }
        }
      }
  }
  else {
    std::cerr << "Failed to load PCD file: " << file_path << "." << std::endl;
  }
}

} // namespace velodyne
} // namespace msf
} // namespace localization
} // namespace apollo
