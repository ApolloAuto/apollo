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

#ifndef MODULES_LOCALIZATION_MSF_VOXEL_GRID_COVARIANCE_HDMAP_H_
#define MODULES_LOCALIZATION_MSF_VOXEL_GRID_COVARIANCE_HDMAP_H_

#include <map>
#include <pcl/common/common.h>
#include <pcl/filters/boost.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Dense>
#include <Eigen/Cholesky>

namespace apollo {
namespace localization {
namespace msf {

template<typename PointT>
class VoxelGridCovariance : public pcl::VoxelGrid<PointT>
{
 public:
  enum LeafType {
    FEW,
    BAD,
    PLANE,
    LINE
  };
 protected:
  using pcl::VoxelGrid<PointT>::filter_name_;
  using pcl::VoxelGrid<PointT>::getClassName;
  using pcl::VoxelGrid<PointT>::input_;
  using pcl::VoxelGrid<PointT>::indices_;
  using pcl::VoxelGrid<PointT>::filter_limit_negative_;
  using pcl::VoxelGrid<PointT>::filter_limit_min_;
  using pcl::VoxelGrid<PointT>::filter_limit_max_;
  using pcl::VoxelGrid<PointT>::filter_field_name_;

  using pcl::VoxelGrid<PointT>::downsample_all_data_;
  using pcl::VoxelGrid<PointT>::leaf_layout_;
  using pcl::VoxelGrid<PointT>::save_leaf_layout_;
  using pcl::VoxelGrid<PointT>::leaf_size_;
  using pcl::VoxelGrid<PointT>::min_b_;
  using pcl::VoxelGrid<PointT>::max_b_;
  using pcl::VoxelGrid<PointT>::inverse_leaf_size_;
  using pcl::VoxelGrid<PointT>::div_b_;
  using pcl::VoxelGrid<PointT>::divb_mul_;

  typedef typename pcl::traits::fieldList<PointT>::type FieldList;
  typedef typename pcl::PointCloud<PointT> PointCloud;
  typedef typename PointCloud::Ptr PointCloudPtr;
  typedef typename PointCloud::ConstPtr PointCloudConstPtr;

 public:
  typedef boost::shared_ptr< pcl::VoxelGrid<PointT> > Ptr;
  typedef boost::shared_ptr< const pcl::VoxelGrid<PointT> > ConstPtr;

  struct Leaf {
    Leaf() :
      nr_points_(0),
      mean_(Eigen::Vector3d::Zero ()),
      centroid(),
      cov_(Eigen::Matrix3d::Identity ()),
      icov_(Eigen::Matrix3d::Zero ()),
      evecs_(Eigen::Matrix3d::Identity ()),
      evals_(Eigen::Vector3d::Zero ()) {
    }

    // Get the voxel covariance.
    Eigen::Matrix3d GetCov() const {
      return cov_;
    }

    // Get the inverse of the voxel covariance.
    Eigen::Matrix3d GetInverseCov() const {
      return icov_;
    }

    // Get the voxel centroid.
    Eigen::Vector3d GetMean() const {
      return mean_;
    }

    // Get the eigen vectors of the voxel covariance.
    Eigen::Matrix3d GetEvecs() const {
      return evecs_;
    }

    // Get the eigen values of the voxel covariance.
    Eigen::Vector3d GetEvals() const {
      return evals_;
    }

    // Get the number of points contained by this voxel.
    int GetPointCount() const {
      return nr_points_;
    }

    // Number of points contained by voxel.
    int nr_points_;

    // 3D voxel centroid.
    Eigen::Vector3d mean_;

    // voxel centroid.
    Eigen::VectorXf centroid;

    // Voxel covariance matrix.
    Eigen::Matrix3d cov_;

    // Inverse of voxel covariance matrix.
    Eigen::Matrix3d icov_;

    // Eigen vectors of voxel covariance matrix.
    Eigen::Matrix3d evecs_;

    // Eigen values of voxel covariance matrix.
    Eigen::Vector3d evals_;

    pcl::PointCloud<PointT> cloud_;
    LeafType type_;
  };
  // Pointer to VoxelGridCovariance leaf structure.
  typedef Leaf* LeafPtr;
  // Const pointer to VoxelGridCovariance leaf structure.
  typedef const Leaf* LeafConstPtr;

 public:
  VoxelGridCovariance() :
    searchable_(true),
    min_points_per_voxel_(6),
    min_covar_eigvalue_mult_(0.01),
    leaves_(),
    voxel_centroids_(),
    voxel_centroidsleaf_indices_(),
    kdtree_() {
    downsample_all_data_ = false;
    save_leaf_layout_ = false;
    leaf_size_.setZero();
    min_b_.setZero();
    max_b_.setZero();
    filter_name_ = "VoxelGridCovariance";
  }

  // Set the minimum number of points required for a cell.
  inline void SetMinPointPerVoxel(int min_points_per_voxel) {
    if (min_points_per_voxel > 2) {
      min_points_per_voxel_ = min_points_per_voxel;
    } else {
      PCL_WARN("%s, Covariance need 3 pts, set min_pt_per_vexel to 3",
          this->getClassName().c_str());
      min_points_per_voxel_ = 3;
    }
  }

  // Get the minimum number of points required for a cell to be used.
  inline int GetMinPointPerVoxel() {
    return min_points_per_voxel_;
  }

  // Set the minimum allowable ratio for eigenvalues
  inline void SetCovEigValueInflationRatio(double min_covar_eigvalue_mult) {
    min_covar_eigvalue_mult_ = min_covar_eigvalue_mult;
  }
  // Get the minimum allowable ratio
  inline double GetCovEigValueInflationRatio() {
    return min_covar_eigvalue_mult_;
  }

  // Filter cloud and initializes voxel structure.
  inline void Filter (PointCloud& output, bool searchable = false) {
    searchable_ = searchable;
    ApplyFilter(output);
    voxel_centroids_ = PointCloudPtr(new PointCloud(output));
    if (searchable_ && voxel_centroids_->size() > 0) {
      kdtree_.setInputCloud(voxel_centroids_);
    }
  }

  // Initializes voxel structure.
  inline void Filter(bool searchable = false) {
    searchable_ = searchable;
    voxel_centroids_ = PointCloudPtr(new PointCloud);
    ApplyFilter(*voxel_centroids_);
    if (searchable_ && voxel_centroids_->size() > 0) {
      kdtree_.setInputCloud(voxel_centroids_);
    }
  }

  // Get the voxel containing point p.
  inline LeafConstPtr GetLeaf(int index) {
    typename std::map<size_t, Leaf>::iterator leaf_iter = leaves_.find(index);
    if (leaf_iter != leaves_.end()) {
      LeafConstPtr ret(&(leaf_iter->second));
      return ret;
    } else {
      return NULL;
    }
  }

  // Get the voxel containing point p.
  inline LeafConstPtr GetLeaf(PointT& p) {
    // Generate index associated with p
    int ijk0 = static_cast<int>
        (floor(p.x * inverse_leaf_size_[0]) - min_b_[0]);
    int ijk1 = static_cast<int>
        (floor(p.y * inverse_leaf_size_[1]) - min_b_[1]);
    int ijk2 = static_cast<int>
        (floor(p.z * inverse_leaf_size_[2]) - min_b_[2]);

    // Compute the centroid leaf index
    int idx = ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2];

    // Find leaf associated with index
    typename std::map<size_t, Leaf>::iterator leaf_iter = leaves_.find(idx);
    if (leaf_iter != leaves_.end()) {
      // If such a leaf exists return the pointer to the leaf structure
      LeafConstPtr ret(&(leaf_iter->second));
      return ret;
    } else {
      return NULL;
    }
  }

  // Get the voxel containing point p.
  inline LeafConstPtr GetLeaf (Eigen::Vector3f& p) {
    // Generate index associated with p
    int ijk0 = static_cast<int>
        (floor(p[0] * inverse_leaf_size_[0]) - min_b_[0]);
    int ijk1 = static_cast<int>
        (floor(p[1] * inverse_leaf_size_[1]) - min_b_[1]);
    int ijk2 = static_cast<int>
        (floor(p[2] * inverse_leaf_size_[2]) - min_b_[2]);

    // Compute the centroid leaf index
    int idx = ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2];

    // Find leaf associated with index
    typename std::map<size_t, Leaf>::iterator leaf_iter = leaves_.find(idx);
    if (leaf_iter != leaves_.end()) {
      // If such a leaf exists return the pointer to the leaf structure
      LeafConstPtr ret(&(leaf_iter->second));
      return ret;
    } else {
      return NULL;
    }
  }
  // Get the voxels surrounding point p.
  int GetNeighborhoodAtPoint(const PointT& reference_point,
      std::vector<LeafConstPtr> &neighbors) {
    neighbors.clear();
    // Find displacement coordinates
    Eigen::MatrixXi relative_coordinates = pcl::getAllNeighborCellIndices();
    Eigen::Vector4i ijk(
        static_cast<int>(floor(reference_point.x / leaf_size_[0])),
        static_cast<int>(floor(reference_point.y / leaf_size_[1])),
        static_cast<int>(floor(reference_point.z / leaf_size_[2])), 0);
    Eigen::Array4i diff2min = min_b_ - ijk;
    Eigen::Array4i diff2max = max_b_ - ijk;
    neighbors.reserve(relative_coordinates.cols());

    // Check each neighbor if it is occupied and contains sufficient points
    for (int ni = 0; ni < relative_coordinates.cols(); ni++) {
      Eigen::Vector4i displacement =
          (Eigen::Vector4i() << relative_coordinates.col(ni), 0).finished();
      // Checking if the specified cell is in the grid
      if ((diff2min <= displacement.array()).all() &&
          (diff2max >= displacement.array()).all()) {
        typename std::map<size_t, Leaf>::iterator leaf_iter =
            leaves_.find(((ijk + displacement - min_b_).dot(divb_mul_)));
        if (leaf_iter != leaves_.end() &&
            leaf_iter->second.nr_points_ >= min_points_per_voxel_) {
          LeafConstPtr leaf = &(leaf_iter->second);
          neighbors.push_back(leaf);
        }
      }
    }

    return (static_cast<int> (neighbors.size()));
  }

  // Get the leaf structure map.
  inline std::map<size_t, Leaf>& GetLeaves() {
    return leaves_;
  }

  // Get a pointcloud containing the voxel centroids
  inline PointCloudPtr GetCentroids() {
    return voxel_centroids_;
  }

  // Get a cloud to visualize each voxels normal distribution.
  void GetDisplayCloud(PointCloud& cell_cloud) {
    cell_cloud.clear();

    int pnt_per_cell = 1000;
    boost::mt19937 rng;
    boost::normal_distribution<> nd(0.0, leaf_size_.head(3).norm());
    boost::variate_generator<boost::mt19937&, boost::normal_distribution<> >
        var_nor(rng, nd);

    Eigen::LLT<Eigen::Matrix3d> llt_of_cov;
    Eigen::Matrix3d cholesky_decomp;
    Eigen::Vector3d cell_mean;
    Eigen::Vector3d rand_point;
    Eigen::Vector3d dist_point;

    // Generate points for each occupied voxel with sufficient points.
    for (typename std::map<size_t, Leaf>::iterator it = leaves_.begin();
        it != leaves_.end(); ++it) {
      Leaf& leaf = it->second;
      if (leaf.nr_points_ >= min_points_per_voxel_) {
        cell_mean = leaf.mean_;
        llt_of_cov.compute(leaf.cov_);
        cholesky_decomp = llt_of_cov.matrixL();
        for (int i = 0; i < pnt_per_cell; i++) {
          rand_point = Eigen::Vector3d(var_nor(), var_nor(), var_nor());
          dist_point = cell_mean + cholesky_decomp * rand_point;
          cell_cloud.push_back(pcl::PointXYZ(
              static_cast<float>(dist_point(0)),
              static_cast<float>(dist_point(1)),
              static_cast<float>(dist_point(2))));
        }
      }
    }
  }

  // Search for the k-nearest occupied voxels for the given query point.
  int NearestKsearch(const PointT &point, int k,
      std::vector<LeafConstPtr> &k_leaves,
      std::vector<float> &k_sqr_distances) {
    k_leaves.clear();

    // Check if kdtree has been built
    if (!searchable_) {
      PCL_WARN("%s: Not Searchable", this->getClassName().c_str());
      return 0;
    }

    // Find k-nearest neighbors in the occupied voxel centroid cloud
    std::vector<int> k_indices;
    k = kdtree_.nearestKSearch(point, k, k_indices, k_sqr_distances);

    // Find leaves corresponding to neighbors
    k_leaves.reserve(k);
    for (std::vector<int>::iterator iter = k_indices.begin();
        iter != k_indices.end(); iter++) {
      k_leaves.push_back(&leaves_[voxel_centroidsleaf_indices_[*iter]]);
    }
    return k;
  }

  // Search for the k-nearest occupied voxels for the given query point.
  inline int NearestKsearch(const PointCloud &cloud, int index, int k,
      std::vector<LeafConstPtr> &k_leaves,
      std::vector<float> &k_sqr_distances) {
    if (index >= static_cast<int> (cloud.points.size()) || index < 0) {
        return (0);
    }
    return NearestKsearch(cloud.points[index], k, k_leaves, k_sqr_distances);
  }

  // Search for all the nearest occupied voxels in a given radius.
  int RadiusSearch(const PointT &point, double radius,
      std::vector<LeafConstPtr> &k_leaves,
      std::vector<float> &k_sqr_distances, unsigned int max_nn = 0) {
    k_leaves.clear();

    // Check if kdtree has been built
    if (!searchable_) {
      PCL_WARN("%s: Not Searchable", this->getClassName().c_str());
      return 0;
    }

    // Find neighbors within radius in the occupied voxel centroid cloud
    std::vector<int> k_indices;
    int k = kdtree_.radiusSearch(point, radius, k_indices,
        k_sqr_distances, max_nn);

    // Find leaves corresponding to neighbors
    k_leaves.reserve(k);
    for (std::vector<int>::iterator iter = k_indices.begin();
        iter != k_indices.end(); iter++) {
      k_leaves.push_back(&leaves_[voxel_centroidsleaf_indices_[*iter]]);
    }
    return k;
  }

  // Search for all the nearest occupied voxels of in a given radius.
  inline int RadiusSearch(const PointCloud &cloud, int index, double radius,
      std::vector<LeafConstPtr> &k_leaves,
      std::vector<float> &k_sqr_distances,
      unsigned int max_nn = 0) {
    if (index >= static_cast<int> (cloud.points.size()) || index < 0) {
        return (0);
    }
    return RadiusSearch(cloud.points[index], radius,
        k_leaves, k_sqr_distances, max_nn);
  }

 private:
  // Filter cloud and initializes voxel structure.
  void ApplyFilter(PointCloud& output) {
    voxel_centroidsleaf_indices_.clear();

    // Has the input dataset been set already?
    if (!input_) {
      PCL_WARN("[%s::ApplyFilter] No input dataset given!\n",
          getClassName().c_str());
      output.width = output.height = 0;
      output.points.clear();
      return;
    }

    output.height = 1;
    output.is_dense = true;
    output.points.clear();

    Eigen::Vector4f min_p, max_p;
    // Get the minimum and maximum dimensions
    if (!filter_field_name_.empty()) {
      pcl::getMinMax3D<PointT> (input_, filter_field_name_,
          static_cast<float> (filter_limit_min_),
          static_cast<float> (filter_limit_max_),
          min_p, max_p, filter_limit_negative_);
    } else {
      pcl::getMinMax3D<PointT> (*input_, min_p, max_p);
    }
    // Check that the leaf size is not too small.
    int64_t dx = static_cast<int64_t>
        ((max_p[0] - min_p[0]) * inverse_leaf_size_[0]) + 1;
    int64_t dy = static_cast<int64_t>
        ((max_p[1] - min_p[1]) * inverse_leaf_size_[1]) + 1;
    int64_t dz = static_cast<int64_t>
        ((max_p[2] - min_p[2]) * inverse_leaf_size_[2]) + 1;

    if ((dx * dy * dz) > std::numeric_limits<int32_t>::max()) {
      PCL_WARN("[%s::ApplyFilter] leaf size is too small. \
          Integer indices would overflow.",
          getClassName().c_str());
      output.clear();
      return;
    }
    // Compute the minimum and maximum bounding box values
    min_b_[0] = static_cast<int>(floor(min_p[0] * inverse_leaf_size_[0]));
    max_b_[0] = static_cast<int>(floor(max_p[0] * inverse_leaf_size_[0]));
    min_b_[1] = static_cast<int>(floor(min_p[1] * inverse_leaf_size_[1]));
    max_b_[1] = static_cast<int>(floor(max_p[1] * inverse_leaf_size_[1]));
    min_b_[2] = static_cast<int>(floor(min_p[2] * inverse_leaf_size_[2]));
    max_b_[2] = static_cast<int>(floor(max_p[2] * inverse_leaf_size_[2]));

    // Compute the number of divisions needed along all axis
    div_b_ = max_b_ - min_b_ + Eigen::Vector4i::Ones();
    div_b_[3] = 0;
    // Clear the leaves
    leaves_.clear();
    // Set up the division multiplier
    divb_mul_ = Eigen::Vector4i(1, div_b_[0], div_b_[0] * div_b_[1], 0);
    int centroid_size = 4;
    if (downsample_all_data_) {
      centroid_size = boost::mpl::size<FieldList>::value;
    }
    // ---[ RGB special case
    std::vector<pcl::PCLPointField> fields;
    int rgba_index = -1;
    rgba_index = pcl::getFieldIndex(*input_, "rgb", fields);
    if (rgba_index == -1) {
      rgba_index = pcl::getFieldIndex(*input_, "rgba", fields);
    }
    if (rgba_index >= 0) {
      rgba_index = fields[rgba_index].offset;
      centroid_size += 4;
    }
    // If we don't want to process the entire cloud,
    // but rather filter points far away from the viewpoint first.
    if (!filter_field_name_.empty()) {
      // Get the distance field index
      std::vector<pcl::PCLPointField> fields;
      int distance_idx = pcl::getFieldIndex(*input_,
          filter_field_name_, fields);
      if (distance_idx == -1) {
          PCL_WARN("[pcl::%s::ApplyFilter] Invalid filter field name. \
              Index is %d.\n", getClassName().c_str(), distance_idx);
      }
      // First pass: go over all points and insert them into the right leaf
      for (size_t cp = 0; cp < input_->points.size(); ++cp) {
        if (!input_->is_dense) {
          // Check if the point is invalid
          if (!pcl_isfinite(input_->points[cp].x) ||
              !pcl_isfinite(input_->points[cp].y) ||
              !pcl_isfinite(input_->points[cp].z)) {
            continue;
          }
        }
        // Get the distance value
        const uint8_t* pt_data
            = reinterpret_cast<const uint8_t*> (&input_->points[cp]);
        float distance_value = 0;
        memcpy(&distance_value,
            pt_data + fields[distance_idx].offset, sizeof(float));

        if (filter_limit_negative_) {
          // Use a threshold for cutting out points which inside the interval
          if ((distance_value < filter_limit_max_) &&
              (distance_value > filter_limit_min_)) {
            continue;
          }
        } else {
          // Use a threshold for cutting out points which are too close/far away
          if ((distance_value > filter_limit_max_) ||
              (distance_value < filter_limit_min_)) {
            continue;
          }
        }

        int ijk0 = static_cast<int>
            (floor(input_->points[cp].x * inverse_leaf_size_[0]) -
            static_cast<float>(min_b_[0]));
        int ijk1 = static_cast<int>
            (floor(input_->points[cp].y * inverse_leaf_size_[1]) -
            static_cast<float>(min_b_[1]));
        int ijk2 = static_cast<int>
            (floor(input_->points[cp].z * inverse_leaf_size_[2]) -
            static_cast<float>(min_b_[2]));
        // Compute the centroid leaf index
        int idx = ijk0 * divb_mul_[0] +
            ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2];

        Leaf& leaf = leaves_[idx];
        if (leaf.nr_points_ == 0) {
          leaf.centroid.resize(centroid_size);
          leaf.centroid.setZero();
        }

        //!added by wangcheng
        leaf.cloud_.points.push_back(input_->points[cp]);

        Eigen::Vector3d pt3d(input_->points[cp].x,
            input_->points[cp].y,
            input_->points[cp].z);
        // Accumulate point sum for centroid calculation
        leaf.mean_ += pt3d;
        // Accumulate x*xT for single pass covariance calculation
        leaf.cov_ += pt3d * pt3d.transpose();

        // Do we need to process all the fields?
        if (!downsample_all_data_) {
          Eigen::Vector4f pt(input_->points[cp].x,
              input_->points[cp].y,
              input_->points[cp].z, 0);
          leaf.centroid.template head<4> () += pt;
        } else {
          // Copy all the fields
          Eigen::VectorXf centroid = Eigen::VectorXf::Zero(centroid_size);
          pcl::for_each_type<FieldList> (pcl::NdCopyPointEigenFunctor<PointT>(
              input_->points[cp], centroid));
          // ---[ RGB special case
          if (rgba_index >= 0) {
            // Fill r/g/b data, assuming that the order is BGRA
            const pcl::RGB& rgb = *reinterpret_cast<const pcl::RGB*> (
                reinterpret_cast<const char*> (&input_->points[cp]) +
                rgba_index);
            centroid[centroid_size - 4] = rgb.a;
            centroid[centroid_size - 3] = rgb.r;
            centroid[centroid_size - 2] = rgb.g;
            centroid[centroid_size - 1] = rgb.b;
          }
          leaf.centroid += centroid;
        }
        ++leaf.nr_points_;
      }
    } else {// No distance filtering, process all data
      // First pass: go over all points and insert them into the right leaf
      for (size_t cp = 0; cp < input_->points.size(); ++cp) {
        if (!input_->is_dense) {
          // Check if the point is invalid
          if (!pcl_isfinite(input_->points[cp].x) ||
              !pcl_isfinite(input_->points[cp].y) ||
              !pcl_isfinite(input_->points[cp].z)) {
            continue;
          }
        }
        int ijk0 = static_cast<int>
            (floor(input_->points[cp].x * inverse_leaf_size_[0]) -
            static_cast<float>(min_b_[0]));
        int ijk1 = static_cast<int>
            (floor(input_->points[cp].y * inverse_leaf_size_[1]) -
            static_cast<float>(min_b_[1]));
        int ijk2 = static_cast<int>
            (floor(input_->points[cp].z * inverse_leaf_size_[2]) -
            static_cast<float>(min_b_[2]));

        // Compute the centroid leaf index
        int idx = ijk0 * divb_mul_[0] +
            ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2];

        // int idx = (((input_->points[cp].getArray4fmap () *
        // inverse_leaf_size_).template cast<int> ()).matrix ()
        //    - min_b_).dot (divb_mul_);

        Leaf& leaf = leaves_[idx];
        if (leaf.nr_points_ == 0) {
          leaf.centroid.resize(centroid_size);
          leaf.centroid.setZero();
        }

        //!added by wangcheng
        leaf.cloud_.points.push_back(input_->points[cp]);

        Eigen::Vector3d pt3d(input_->points[cp].x,
            input_->points[cp].y,
            input_->points[cp].z);
        // Accumulate point sum for centroid calculation
        leaf.mean_ += pt3d;
        // Accumulate x*xT for single pass covariance calculation
        leaf.cov_ += pt3d * pt3d.transpose();

        // Do we need to process all the fields?
        if (!downsample_all_data_) {
          Eigen::Vector4f pt(input_->points[cp].x,
              input_->points[cp].y,
              input_->points[cp].z, 0);
          leaf.centroid.template head<4>() += pt;
        } else {
          // Copy all the fields
          Eigen::VectorXf centroid = Eigen::VectorXf::Zero(centroid_size);
          pcl::for_each_type<FieldList> (pcl::NdCopyPointEigenFunctor<PointT>
              (input_->points[cp], centroid));
          // ---[ RGB special case
          if (rgba_index >= 0) {
            // Fill r/g/b data, assuming that the order is BGRA
            const pcl::RGB& rgb = *reinterpret_cast<const pcl::RGB*> (
                reinterpret_cast<const char*> (&input_->points[cp]) +
                rgba_index);
            centroid[centroid_size - 4] = rgb.a;
            centroid[centroid_size - 3] = rgb.r;
            centroid[centroid_size - 2] = rgb.g;
            centroid[centroid_size - 1] = rgb.b;
          }
          leaf.centroid += centroid;
        }
        ++leaf.nr_points_;
      }
    }

    // Second pass: go over all leaves and compute centroids and covariance
    output.points.reserve(leaves_.size());
    if (searchable_) {
      voxel_centroidsleaf_indices_.reserve(leaves_.size());
    }
    int cp = 0;
    if (save_leaf_layout_) {
      leaf_layout_.resize(div_b_[0] * div_b_[1] * div_b_[2], -1);
    }
    // Eigen values and vectors calculated to prevent near singluar matrices
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver;
    Eigen::Matrix3d eigen_val;
    Eigen::Vector3d pt_sum;

    // Eigen values less than a threshold of max eigen value are
    // inflated to a set fraction of the max eigen value.
    double min_covar_eigvalue;

    for (typename std::map<size_t, Leaf>::iterator it = leaves_.begin();
      it != leaves_.end(); ++it) {
      // Normalize the centroid
      Leaf& leaf = it->second;

      // Normalize the centroid
      leaf.centroid /= static_cast<float> (leaf.nr_points_);
      // Point sum used for single pass covariance calculation
      pt_sum = leaf.mean_;
      // Normalize mean
      leaf.mean_ /= leaf.nr_points_;

      if (leaf.nr_points_ >= min_points_per_voxel_) {
        if (save_leaf_layout_) {
          leaf_layout_[it->first] = cp++;
        }
        output.push_back(PointT());
        // Do we need to process all the fields?
        if (!downsample_all_data_) {
          output.points.back().x = leaf.centroid[0];
          output.points.back().y = leaf.centroid[1];
          output.points.back().z = leaf.centroid[2];
        } else {
          pcl::for_each_type<FieldList> (pcl::NdCopyEigenPointFunctor<PointT>
              (leaf.centroid, output.back()));
          // ---[ RGB special case
          if (rgba_index >= 0) {
            pcl::RGB& rgb = *reinterpret_cast<pcl::RGB*>
                (reinterpret_cast<char*>(&output.points.back()) + rgba_index);
            rgb.a = leaf.centroid[centroid_size - 4];
            rgb.r = leaf.centroid[centroid_size - 3];
            rgb.g = leaf.centroid[centroid_size - 2];
            rgb.b = leaf.centroid[centroid_size - 1];
          }
        }

        // Stores the voxel indice for fast access searching
        if (searchable_) {
          voxel_centroidsleaf_indices_.push_back(static_cast<int> (it->first));
        }

        // Single pass covariance calculation
        leaf.cov_ = (leaf.cov_ - 2 * (pt_sum * leaf.mean_.transpose())) /
            leaf.nr_points_ + leaf.mean_ * leaf.mean_.transpose();
        leaf.cov_ *= (leaf.nr_points_ - 1.0) / leaf.nr_points_;

        //Normalize Eigen Val such that max no more than 100x min.
        eigensolver.compute(leaf.cov_);
        eigen_val = eigensolver.eigenvalues().asDiagonal();
        leaf.evecs_ = eigensolver.eigenvectors();

        if (eigen_val(0, 0) < 0 || eigen_val(1, 1) < 0 ||
            eigen_val(2, 2) <= 0) {
          leaf.nr_points_ = -1;
          continue;
        }

        // Avoids matrices near singularities (eq 6.11)[Magnusson 2009]
        min_covar_eigvalue = min_covar_eigvalue_mult_ * eigen_val(2, 2);
        if (eigen_val(0, 0) < min_covar_eigvalue) {
          eigen_val(0, 0) = min_covar_eigvalue;
          if (eigen_val(1, 1) < min_covar_eigvalue) {
            eigen_val(1, 1) = min_covar_eigvalue;
          }
          leaf.cov_ = leaf.evecs_ * eigen_val * leaf.evecs_.inverse();
        }
        leaf.evals_ = eigen_val.diagonal();

        leaf.icov_ = leaf.cov_.inverse();
        if (leaf.icov_.maxCoeff() == std::numeric_limits<float>::infinity() ||
            leaf.icov_.minCoeff() == -std::numeric_limits<float>::infinity()) {
          leaf.nr_points_ = -1;
        }
      }
    }
    output.width = static_cast<uint32_t>(output.points.size());
  }

  // Flag to determine if voxel structure is searchable. */
  bool searchable_;

  // Minimum points contained with in a voxel to allow it to be useable.
  int min_points_per_voxel_;

  // Minimum allowable ratio between eigenvalues.
  double min_covar_eigvalue_mult_;

  // Voxel structure containing all leaf nodes.       
  std::map<size_t, Leaf> leaves_;

  /* Point cloud containing centroids of voxels
   * containing atleast minimum number of points. */
  PointCloudPtr voxel_centroids_;

  /* Indices of leaf structurs associated with each point in
   * \ref _voxel_centroids (used for searching). */
  std::vector<int> voxel_centroidsleaf_indices_;

  // KdTree used for searching.
  pcl::KdTreeFLANN<PointT> kdtree_;
};

} // msf
} // localization
} // apollo
#endif //MODULES_LOCALIZATION_MSF_VOXEL_GRID_COVARIANCE_HDMAP_H_
