#ifndef IDL_CAR_VELODYNE_UTILITY_H
#define IDL_CAR_VELODYNE_UTILITY_H

#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace apollo {
namespace localization {
namespace msf {
namespace velodyne {

struct VelodyneFrame {
    /**@brief The frame index. */
    unsigned int frame_index;
    /**@brief The time stamp. */
    double timestamp;
    /**@brief The 3D point cloud in this frame. */
    std::vector<Eigen::Vector3d> pt3ds;
    /**@brief The laser reflection values in this frames. */
    std::vector<unsigned char> intensities;
    /**@brief The laser IDs. */
    std::vector<unsigned char> laser_ids;
    /**@brief The pose of the frame. */
    Eigen::Affine3d pose;
};

void load_pcds(std::string file_path, 
        unsigned int frame_index,
        const Eigen::Affine3d& pose,
        VelodyneFrame& velodyne_frame, 
        bool is_global);

void load_pcds(std::string file_path, 
        unsigned int frame_index,
        const Eigen::Affine3d& pose,
        std::vector<Eigen::Vector3d>& pt3ds, 
        std::vector<unsigned char>& intensities,
        bool is_global);

/**@brief Load the PCD poses with their timestamps. */
void load_pcd_poses(std::string file_path, std::vector<Eigen::Affine3d>& poses,
                    std::vector<double>& timestamps);

/**@brief Load the PCD poses with their timestamps and indices. */
void load_pcd_poses(std::string file_path, std::vector<Eigen::Affine3d>& poses,
                    std::vector<double>& timestamps, 
                    std::vector<unsigned int>& pcd_indices);

// /**@brief Save the PCD poses with their timestamps. */
// void save_pcd_poses(std::string file_path, const std::vector<Eigen::Affine3d>& poses,
//                     const std::vector<double>& timestamps);

/**@brief Load the velodyne extrinsic from a YAML file. */
bool load_extrinsic(std::string file_path, Eigen::Affine3d& extrinsic);

} // namespace velodyne
} // namespace msf
} // namespace localization
} // namespace apollo

#endif // IDL_CAR_VELODYNE_UTILITY_H

