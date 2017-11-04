/**
  @file feature_xy_plane_pipeline.h
  @brief class of feature_xy_plane_pipeline
*/
#ifndef ADU_HDMAP_EXTRACT_GROUND_PLANE_H
#define ADU_HDMAP_EXTRACT_GROUND_PLANE_H
#include <vector>
#include <cmath>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/impl/ransac.hpp>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/impl/sac_model_plane.hpp>
#include "modules/localization/msf/common/util/voxel_grid_covariance_hdmap.h"

namespace apollo {
namespace localization {
namespace msf {
class FeatureXYPlane {
public:
    typedef pcl::PointXYZI PointT;
    typedef pcl::PointCloud<PointT> PointCloudT;
    typedef PointCloudT::Ptr PointCloudPtrT;
public:
    FeatureXYPlane() {
        _min_grid_size = 0.5;
        _max_grid_size = 4.00;
        _plane_inlier_distance = 0.05;
        _min_planepoints_number = 60;
        _plane_type_degree = 80.0;
        _below_lidar_height = 1.0;
        _xy_plane_cloud = PointCloudPtrT(new PointCloudT);
        _non_xy_plane_cloud = PointCloudPtrT(new PointCloudT);
    }

    void set_min_grid_size(double d) {
        _min_grid_size = d;
    }

    void set_max_grid_size(double d) {
        _max_grid_size = d;
    }

    void set_plane_inlier_distance(double d) {
        _plane_inlier_distance = d;
    }

    void set_min_planepoints_number(double d) {
        _min_planepoints_number = d;
    }

    void set_plane_type_degree(double d) {
        _plane_type_degree = d;
    }

    void set_below_lidar_height(double d) {
        _below_lidar_height = d;
    }

    float calculate_degree(const Eigen::Vector3f& tmp0,
                            const Eigen::Vector3f& tmp1) {
        float cos_theta = tmp0.dot(tmp1) / (tmp0.norm() * tmp1.norm());
        return std::acos(cos_theta) * 180.0 / M_PI;
    }

    PointCloudPtrT& get_xy_plane_cloud() {
        return _xy_plane_cloud;
    }

    PointCloudPtrT& get_non_xy_plane_cloud() {
        return _non_xy_plane_cloud;
    }

    bool get_plane_feature_point(PointCloudT& cloud,
                                 PointCloudT& cloud_outlier) {
        //ransac plane
        std::vector<int> inliers;
        PointCloudPtrT cloud_new(new PointCloudT);
        *cloud_new = cloud;
        pcl::SampleConsensusModelPlane<PointT>::Ptr
                model_plane(new pcl::SampleConsensusModelPlane<PointT>(cloud_new));
        pcl::RandomSampleConsensus<PointT> ransac(model_plane);
        ransac.setDistanceThreshold(_plane_inlier_distance);
        ransac.computeModel();
        ransac.getInliers(inliers);
        if (inliers.size() < _min_planepoints_number) {
            return false;
        }
        PointCloudPtrT cloud_inlier(new PointCloudT);
        pcl::copyPointCloud<PointT>(*cloud_new, inliers, *cloud_inlier);
        std::vector<int> outliers;
        int inlier_idx = 0;
        for (unsigned int i = 0; i < cloud_new->points.size(); ++i) {
            if (i < inliers[inlier_idx]) {
                outliers.push_back(i);
            } else {
                inlier_idx++;
            }
        }
        pcl::copyPointCloud<PointT>(*cloud_new, outliers, cloud_outlier);

        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud_inlier, centroid);

        if (centroid(2) > -_below_lidar_height) {
            return true;
        }

        //get plane's normal (which is normalized)
        Eigen::VectorXf coeff;
        ransac.getModelCoefficients(coeff);
        //determin the plane type
        double tan_theta = 0;
        double tan_refer_theta = std::tan(_plane_type_degree / 180.0 * M_PI);
        if ((std::abs(coeff(2)) > std::abs(coeff(0))) &&
                (std::abs(coeff(2)) > std::abs(coeff(1)))) {
            tan_theta = std::abs(coeff(2)) /
                    std::sqrt(coeff(0) * coeff(0) + coeff(1) * coeff(1));
            if (tan_theta > tan_refer_theta) {
                *_xy_plane_cloud += *cloud_inlier;
            } else {
                // cloud_outlier += *cloud_inlier;
            }
        }
        return true;
    }

    double power2(int x) {
        double result = 1.0;
        for (int i = 0; i < x; ++i) {
            result *= 2.0;
        }
        return result;
    }

    void extract_xy_plane(const PointCloudPtrT& cloud) {
        _xy_plane_cloud.reset(new PointCloudT);
        PointCloudPtrT pointcloud_ptr(new PointCloudT);
        pcl::copyPointCloud<PointT>(*cloud, *pointcloud_ptr);
        int iter_num = log2(_max_grid_size / _min_grid_size);
        if (iter_num == 0) {
            iter_num = 1;
        }
        std::clock_t plane_time;
        plane_time = std::clock();
        int total_plane_num = 0;
        for (int iter = 0; iter <= iter_num; ++iter) {

            double grid_size = _max_grid_size / power2(iter);

            VoxelGridCovariance<PointT> vgc;
            vgc.setInputCloud(pointcloud_ptr);
            vgc.set_min_point_per_voxel(_min_planepoints_number);
            vgc.setLeafSize(grid_size, grid_size, grid_size);
            vgc.filter(false);

            PointCloudT cloud_tmp;
            int plane_num = 0;
            typename std::map<size_t,
                    VoxelGridCovariance<PointT>::Leaf>::iterator it;
            for (it = vgc.get_leaves().begin(); it != vgc.get_leaves().end(); it++) {
                if (it->second.get_point_count() < _min_planepoints_number) {
                    cloud_tmp += it->second.cloud_;
                    continue;
                }
                PointCloudT cloud_outlier;
                if (get_plane_feature_point(it->second.cloud_, cloud_outlier)) {
                    cloud_tmp += cloud_outlier;
                    plane_num++;
                } else {
                    cloud_tmp += it->second.cloud_;
                }
            }
            std::cerr << "the " << iter << " interation: plane_num = " << plane_num << std::endl;
            total_plane_num += plane_num;
            pointcloud_ptr.reset(new PointCloudT);
            *pointcloud_ptr = cloud_tmp;
        }
        *_non_xy_plane_cloud = *pointcloud_ptr;
        plane_time = std::clock() - plane_time;
        std::cerr << "plane_patch takes:" <<
                     static_cast<double>(plane_time) / CLOCKS_PER_SEC << "sec." << std::endl;
        std::cerr << "total_plane_num = " << total_plane_num << std::endl;
        std::cerr << "total_points_num = " << _xy_plane_cloud->points.size() << std::endl;
        return;
    }
private:
    //parameters
    double _min_grid_size;
    double _max_grid_size;
    double _plane_inlier_percent;
    double _plane_inlier_distance;
    int _min_planepoints_number;
    double _plane_type_degree;
    double _below_lidar_height;

    PointCloudPtrT _xy_plane_cloud;
    PointCloudPtrT _non_xy_plane_cloud;
};
} // msf
} // localization
} // apollo
#endif //ADU_HDMAP_EXTRACT_GROUND_PLANE_H
