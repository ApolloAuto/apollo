// Copyright 2016 Baidu Inc. All Rights Reserved.
// @author: wang jun(wangjun21@baidu.com)
//          zhang ye(zhangye09@baidu.com)
// @file: pose_util.cpp
// @brief: pose util

#include "modules/perception/obstacle/common/pose_util.h"

namespace apollo {
namespace perception {

bool read_pose_file(const std::string& filename, 
        Eigen::Matrix4d& pose, 
        int& frame_id, 
        double& time_stamp) {
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
    quaternion_to_rotation_matrix<double>(quat, matrix3x3);

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            pose(i, j) = matrix3x3[i * 3 + j];
        }
    }

    frame_id = id;
    time_stamp = time_samp;
    return true;
}

bool read_pose_file_mat12(const std::string& filename, 
        Eigen::Matrix4d& pose, 
        int& frame_id, 
        double& time_stamp) {
    std::ifstream ifs(filename.c_str());
    if (!ifs.is_open()) {
        std::cerr << "Failed to open file " << filename << std::endl;
        return false;
    }
    pose = Eigen::Matrix4d::Identity();
    ifs >> frame_id >> time_stamp;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            ifs >> pose(i, j);
        }
    }
    return true;
}

} // namespace perception
} // namespace apollo
