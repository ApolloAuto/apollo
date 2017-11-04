/**
  @file voxel_grid_covariance_hdmap.h
  @brief this file is modified from PointCloudLibrary
 */
#ifndef ADU_HAD_MAP_VOXEL_GRID_COVARIANCE_HDMAP_H
#define ADU_HAD_MAP_VOXEL_GRID_COVARIANCE_HDMAP_H
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
/** \brief A searchable voxel strucure containing the mean and covariance of the data.
    \note For more information please see
    <b>Magnusson, M. (2009). The Three-Dimensional Normal-Distributions Transform —
    an Efﬁcient Representation for Registration, Surface Analysis, and Loop Detection.
    PhD thesis, Orebro University. Orebro Studies in Technology 36</b>
    \author Brian Okorn (Space and Naval Warfare Systems Center Pacific)
  */
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
    /** \brief Simple structure to hold a centroid, covarince and the number of points in a leaf.
      * Inverse covariance, eigen vectors and engen values are precomputed.
      */
    struct Leaf
    {
        /** \brief Constructor.
          * Sets \ref nr_points, \ref icov_, \ref mean_ and
          * \ref evals_ to 0 and \ref cov_ and \ref evecs_ to the identity matrix
          */
        Leaf() :
            nr_points(0),
            mean_(Eigen::Vector3d::Zero ()),
            centroid(),
            cov_(Eigen::Matrix3d::Identity ()),
            icov_(Eigen::Matrix3d::Zero ()),
            evecs_(Eigen::Matrix3d::Identity ()),
            evals_(Eigen::Vector3d::Zero ()) {
        }

        /** \brief Get the voxel covariance.
        * \return covariance matrix
        */
        Eigen::Matrix3d get_cov() const {
            return (cov_);
        }

        /** \brief Get the inverse of the voxel covariance.
        * \return inverse covariance matrix
        */
        Eigen::Matrix3d get_inverse_cov() const {
            return (icov_);
        }

        /** \brief Get the voxel centroid.
        * \return centroid
        */
        Eigen::Vector3d get_mean() const {
            return (mean_);
        }

        /** \brief Get the eigen vectors of the voxel covariance.
        * \note Order corresponds with \ref getEvals
        * \return matrix whose columns contain eigen vectors
        */
        Eigen::Matrix3d get_evecs() const {
            return (evecs_);
        }

        /** \brief Get the eigen values of the voxel covariance.
        * \note Order corresponds with \ref getEvecs
        * \return vector of eigen values
        */
        Eigen::Vector3d get_evals() const {
            return (evals_);
        }

        /** \brief Get the number of points contained by this voxel.
        * \return number of points
        */
        int get_point_count() const {
            return (nr_points);
        }

        /** \brief Number of points contained by voxel */
        int nr_points;

        /** \brief 3D voxel centroid */
        Eigen::Vector3d mean_;

        /** \brief Nd voxel centroid
        * \note Differs from \ref mean_ when color data is used
        */
        Eigen::VectorXf centroid;

        /** \brief Voxel covariance matrix */
        Eigen::Matrix3d cov_;

        /** \brief Inverse of voxel covariance matrix */
        Eigen::Matrix3d icov_;

        /** \brief Eigen vectors of voxel covariance matrix */
        Eigen::Matrix3d evecs_;

        /** \brief Eigen values of voxel covariance matrix */
        Eigen::Vector3d evals_;

        //added by wangcheng
        pcl::PointCloud<PointT> cloud_;
        LeafType type_;
    };
    /** \brief Pointer to VoxelGridCovariance leaf structure */
    typedef Leaf* LeafPtr;
    /** \brief Const pointer to VoxelGridCovariance leaf structure */
    typedef const Leaf* LeafConstPtr;
public:
    /** \brief Constructor.
      * Sets \ref leaf_size_ to 0 and \ref _searchable to false.
      */
    VoxelGridCovariance() :
        _searchable(true),
        _min_points_per_voxel(6),
        _min_covar_eigvalue_mult(0.01),
        _leaves(),
        _voxel_centroids(),
        __voxel_centroidsleaf_indices(),
        _kdtree() {
        downsample_all_data_ = false;
        save_leaf_layout_ = false;
        leaf_size_.setZero();
        min_b_.setZero();
        max_b_.setZero();
        filter_name_ = "VoxelGridCovariance";
    }

    /** \brief Set the minimum number of points required for a cell to
     * be used (must be 3 or greater for covariance calculation).
      * \param[in] min_points_per_voxel the minimum number of
      * points for required for a voxel to be used
      */
    inline void set_min_point_per_voxel(int min_points_per_voxel) {
        if (min_points_per_voxel > 2) {
            _min_points_per_voxel = min_points_per_voxel;
        } else {
            PCL_WARN("%s, Covariance need 3 pts, set min_pt_per_vexel to 3",
                      this->getClassName().c_str());
            _min_points_per_voxel = 3;
        }
    }
    /** \brief Get the minimum number of points required for a cell to be used.
      * \return the minimum number of points for required for a voxel to be used
      */
    inline int get_min_point_per_voxel() {
        return _min_points_per_voxel;
    }
    /** \brief Set the minimum allowable ratio between eigenvalues to prevent singular
      * covariance matrices.
      * \param[in] min_covar_eigvalue_mult the minimum allowable ratio between eigenvalues
      */
    inline void set_cov_eig_value_inflation_ratio(double min_covar_eigvalue_mult) {
        _min_covar_eigvalue_mult = min_covar_eigvalue_mult;
    }
    /** \brief Get the minimum allowable ratio between eigenvalues to
      * prevent singular covariance matrices.
      * \return the minimum allowable ratio between eigenvalues
      */
    inline double get_cov_eig_value_inflation_ratio() {
        return _min_covar_eigvalue_mult;
    }
    /** \brief Filter cloud and initializes voxel structure.
     * \param[out] output cloud containing centroids of voxels containing
     * a sufficient number of points
     * \param[in] searchable flag if voxel structure is searchable, if true then kdtree is built
     */
    inline void filter (PointCloud& output, bool searchable = false) {
        _searchable = searchable;
        apply_filter(output);

        _voxel_centroids = PointCloudPtr(new PointCloud(output));

        if (_searchable && _voxel_centroids->size() > 0) {
            // Initiates kdtree of the centroids of voxels containing
            // a sufficient number of points
            _kdtree.setInputCloud(_voxel_centroids);
        }
    }
    /** \brief Initializes voxel structure.
     * \param[in] searchable flag if voxel structure is searchable, if true then kdtree is built
     */
    inline void filter(bool searchable = false) {
        _searchable = searchable;
        _voxel_centroids = PointCloudPtr(new PointCloud);
        apply_filter(*_voxel_centroids);

        if (_searchable && _voxel_centroids->size() > 0) {
            // Initiates kdtree of the centroids of voxels containing
            // a sufficient number of points
            _kdtree.setInputCloud(_voxel_centroids);
        }
    }
    /** \brief Get the voxel containing point p.
     * \param[in] index the index of the leaf structure node
     * \return const pointer to leaf structure
     */
    inline LeafConstPtr get_leaf(int index) {
        typename std::map<size_t, Leaf>::iterator leaf_iter = _leaves.find(index);
        if (leaf_iter != _leaves.end()) {
            LeafConstPtr ret(&(leaf_iter->second));
            return ret;
        } else {
            return NULL;
        }
    }
    /** \brief Get the voxel containing point p.
     * \param[in] p the point to get the leaf structure at
     * \return const pointer to leaf structure
     */
    inline LeafConstPtr get_leaf(PointT& p) {
        // Generate index associated with p
        int ijk0 = static_cast<int> (floor(p.x * inverse_leaf_size_[0]) - min_b_[0]);
        int ijk1 = static_cast<int> (floor(p.y * inverse_leaf_size_[1]) - min_b_[1]);
        int ijk2 = static_cast<int> (floor(p.z * inverse_leaf_size_[2]) - min_b_[2]);

        // Compute the centroid leaf index
        int idx = ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2];

        // Find leaf associated with index
        typename std::map<size_t, Leaf>::iterator leaf_iter = _leaves.find(idx);
        if (leaf_iter != _leaves.end()) {
            // If such a leaf exists return the pointer to the leaf structure
            LeafConstPtr ret(&(leaf_iter->second));
            return ret;
        } else {
            return NULL;
        }
    }
    /** \brief Get the voxel containing point p.
     * \param[in] p the point to get the leaf structure at
     * \return const pointer to leaf structure
     */
    inline LeafConstPtr get_leaf (Eigen::Vector3f& p) {
        // Generate index associated with p
        int ijk0 = static_cast<int> (floor(p[0] * inverse_leaf_size_[0]) - min_b_[0]);
        int ijk1 = static_cast<int> (floor(p[1] * inverse_leaf_size_[1]) - min_b_[1]);
        int ijk2 = static_cast<int> (floor(p[2] * inverse_leaf_size_[2]) - min_b_[2]);

        // Compute the centroid leaf index
        int idx = ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2];

        // Find leaf associated with index
        typename std::map<size_t, Leaf>::iterator leaf_iter = _leaves.find(idx);
        if (leaf_iter != _leaves.end()) {
          // If such a leaf exists return the pointer to the leaf structure
            LeafConstPtr ret(&(leaf_iter->second));
            return ret;
        } else {
            return NULL;
        }
    }
    /** \brief Get the voxels surrounding point p, not including the voxel contating point p.
     * \note Only voxels containing a sufficient number of points are used
     * (slower than radius search in practice).
     * \param[in] reference_point the point to get the leaf structure at
     * \param[out] neighbors
     * \return number of neighbors found
     */
    int get_neighborhood_at_point(const PointT& reference_point,
                               std::vector<LeafConstPtr> &neighbors) {
        neighbors.clear();
        // Find displacement coordinates
        Eigen::MatrixXi relative_coordinates = pcl::getAllNeighborCellIndices();
        Eigen::Vector4i ijk(static_cast<int>(floor(reference_point.x / leaf_size_[0])),
                             static_cast<int>(floor(reference_point.y / leaf_size_[1])),
                             static_cast<int>(floor(reference_point.z / leaf_size_[2])), 0);
        Eigen::Array4i diff2min = min_b_ - ijk;
        Eigen::Array4i diff2max = max_b_ - ijk;
        neighbors.reserve(relative_coordinates.cols());

        // Check each neighbor to see if it is occupied and contains sufficient points
        // Slower than radius search because needs to check 26 indices
        for (int ni = 0; ni < relative_coordinates.cols(); ni++) {
            Eigen::Vector4i displacement =
                    (Eigen::Vector4i() << relative_coordinates.col(ni), 0).finished();
            // Checking if the specified cell is in the grid
            if ((diff2min <= displacement.array()).all() &&
                (diff2max >= displacement.array()).all()) {
                typename std::map<size_t, Leaf>::iterator leaf_iter =
                        _leaves.find(((ijk + displacement - min_b_).dot(divb_mul_)));
                if (leaf_iter != _leaves.end() &&
                    leaf_iter->second.nr_points >= _min_points_per_voxel) {
                    LeafConstPtr leaf = &(leaf_iter->second);
                    neighbors.push_back(leaf);
                }
            }
        }
        return (static_cast<int> (neighbors.size()));
    }
    /** \brief Get the leaf structure map
     * \return a map contataining all leaves
     */
    inline std::map<size_t, Leaf>& get_leaves() {
        return _leaves;
    }
    /** \brief Get a pointcloud containing the voxel centroids
     * \note Only voxels containing a sufficient number of points are used.
     * \return a map contataining all leaves
     */
    inline PointCloudPtr get_centroids() {
        return _voxel_centroids;
    }
    /** \brief Get a cloud to visualize each voxels normal distribution.
     * \param[out] cell_cloud a cloud created by sampling the normal distributions of each voxel
     */
    void get_display_cloud(PointCloud& cell_cloud) {
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
        for (typename std::map<size_t, Leaf>::iterator it = _leaves.begin();
            it != _leaves.end(); ++it) {
            Leaf& leaf = it->second;
            if (leaf.nr_points >= _min_points_per_voxel) {
                cell_mean = leaf.mean_;
                llt_of_cov.compute(leaf.cov_);
                cholesky_decomp = llt_of_cov.matrixL();

                // Random points generated by sampling the normal distribution
                // given by voxel mean and covariance matrix
                for (int i = 0; i < pnt_per_cell; i++) {
                    rand_point = Eigen::Vector3d(var_nor(), var_nor(), var_nor());
                    dist_point = cell_mean + cholesky_decomp * rand_point;
                    cell_cloud.push_back(pcl::PointXYZ(static_cast<float>(dist_point(0)),
                                                    static_cast<float>(dist_point(1)),
                                                    static_cast<float>(dist_point(2))));
                }
            }
        }
    }
    /** \brief Search for the k-nearest occupied voxels for the given query point.
     * \note Only voxels containing a sufficient number of points are used.
     * \param[in] point the given query point
     * \param[in] k the number of neighbors to search for
     * \param[out] k_leaves the resultant leaves of the neighboring points
     * \param[out] k_sqr_distances the resultant squared distances to the neighboring points
     * \return number of neighbors found
     */
    int nearest_ksearch(const PointT &point, int k,
                       std::vector<LeafConstPtr> &k_leaves,
                       std::vector<float> &k_sqr_distances) {
        k_leaves.clear();

        // Check if kdtree has been built
        if (!_searchable) {
            PCL_WARN("%s: Not Searchable", this->getClassName().c_str());
            return 0;
        }

        // Find k-nearest neighbors in the occupied voxel centroid cloud
        std::vector<int> k_indices;
        k = _kdtree.nearestKSearch(point, k, k_indices, k_sqr_distances);

        // Find leaves corresponding to neighbors
        k_leaves.reserve(k);
        for (std::vector<int>::iterator iter = k_indices.begin();
            iter != k_indices.end(); iter++) {
            k_leaves.push_back(&_leaves[__voxel_centroidsleaf_indices[*iter]]);
        }
        return k;
    }
    /** \brief Search for the k-nearest occupied voxels for the given query point.
     * \note Only voxels containing a sufficient number of points are used.
     * \param[in] cloud the given query point
     * \param[in] index the index
     * \param[in] k the number of neighbors to search for
     * \param[out] k_leaves the resultant leaves of the neighboring points
     * \param[out] k_sqr_distances the resultant squared distances to the neighboring points
     * \return number of neighbors found
     */
    inline int nearest_ksearch(const PointCloud &cloud, int index, int k,
                              std::vector<LeafConstPtr> &k_leaves,
                              std::vector<float> &k_sqr_distances) {
        if (index >= static_cast<int> (cloud.points.size()) || index < 0) {
            return (0);
        }
        return (nearest_ksearch(cloud.points[index], k, k_leaves, k_sqr_distances));
    }
    /** \brief Search for all the nearest occupied voxels of the query point in a given radius.
     * \note Only voxels containing a sufficient number of points are used.
     * \param[in] point the given query point
     * \param[in] radius the radius of the sphere bounding all of p_q's neighbors
     * \param[out] k_leaves the resultant leaves of the neighboring points
     * \param[out] k_sqr_distances the resultant squared distances to the neighboring points
     * \param[in] max_nn
     * \return number of neighbors found
     */
    int radius_search(const PointT &point, double radius, std::vector<LeafConstPtr> &k_leaves,
                      std::vector<float> &k_sqr_distances, unsigned int max_nn = 0) {
        k_leaves.clear();

        // Check if kdtree has been built
        if (!_searchable) {
            PCL_WARN("%s: Not Searchable", this->getClassName().c_str());
            return 0;
        }

        // Find neighbors within radius in the occupied voxel centroid cloud
        std::vector<int> k_indices;
        int k = _kdtree.radiusSearch(point, radius, k_indices, k_sqr_distances, max_nn);

        // Find leaves corresponding to neighbors
        k_leaves.reserve(k);
        for (std::vector<int>::iterator iter = k_indices.begin();
            iter != k_indices.end(); iter++) {
            k_leaves.push_back(&_leaves[__voxel_centroidsleaf_indices[*iter]]);
        }
        return k;
    }
    /** \brief Search for all the nearest occupied voxels of the query point in a given radius.
     * \note Only voxels containing a sufficient number of points are used.
     * \param[in] cloud the given query point
     * \param[in] index a valid index in cloud representing a valid (i.e., finite) query point
     * \param[in] radius the radius of the sphere bounding all of p_q's neighbors
     * \param[out] k_leaves the resultant leaves of the neighboring points
     * \param[out] k_sqr_distances the resultant squared distances to the neighboring points
     * \param[in] max_nn
     * \return number of neighbors found
     */
    inline int radius_search (const PointCloud &cloud, int index, double radius,
                             std::vector<LeafConstPtr> &k_leaves,
                             std::vector<float> &k_sqr_distances,
                             unsigned int max_nn = 0) {
        if (index >= static_cast<int> (cloud.points.size()) || index < 0) {
            return (0);
        }
        return (radius_search(cloud.points[index], radius, k_leaves, k_sqr_distances, max_nn));
    }

private:
    /** \brief Filter cloud and initializes voxel structure.
     * \param[out] output cloud containing centroids of voxels containing
     * a sufficient number of points
     */
    void apply_filter(PointCloud& output) {
        __voxel_centroidsleaf_indices.clear();

        // Has the input dataset been set already?
        if (!input_) {
            PCL_WARN("[%s::apply_filter] No input dataset given!\n", getClassName().c_str());
            output.width = output.height = 0;
            output.points.clear();
            return;
        }
        // Copy the header (and thus the frame_id) + allocate enough space for points
        output.height = 1;                // downsampling breaks the organized structure
        output.is_dense = true;           // we filter out invalid points
        output.points.clear();

        Eigen::Vector4f min_p, max_p;
        // Get the minimum and maximum dimensions
        if (!filter_field_name_.empty()) { // If we don't want to process the entire cloud...
            pcl::getMinMax3D<PointT> (input_, filter_field_name_,
                                 static_cast<float> (filter_limit_min_),
                                 static_cast<float> (filter_limit_max_),
                                 min_p, max_p, filter_limit_negative_);
        } else {
            pcl::getMinMax3D<PointT> (*input_, min_p, max_p);
        }
        // Check that the leaf size is not too small, given the size of the data
        int64_t dx = static_cast<int64_t>((max_p[0] - min_p[0]) * inverse_leaf_size_[0])+1;
        int64_t dy = static_cast<int64_t>((max_p[1] - min_p[1]) * inverse_leaf_size_[1])+1;
        int64_t dz = static_cast<int64_t>((max_p[2] - min_p[2]) * inverse_leaf_size_[2])+1;

        if ((dx*dy*dz) > std::numeric_limits<int32_t>::max()) {
            PCL_WARN("[%s::apply_filter] leaf size is too small.Integer indices would overflow.",
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
        _leaves.clear();
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
        // but rather filter points far away from the viewpoint first...
        if (!filter_field_name_.empty()) {
            // Get the distance field index
            std::vector<pcl::PCLPointField> fields;
            int distance_idx = pcl::getFieldIndex(*input_, filter_field_name_, fields);
            if (distance_idx == -1) {
                PCL_WARN("[pcl::%s::apply_filter] Invalid filter field name. Index is %d.\n",
                          getClassName().c_str(), distance_idx);
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
            const uint8_t* pt_data = reinterpret_cast<const uint8_t*> (&input_->points[cp]);
            float distance_value = 0;
            memcpy(&distance_value, pt_data + fields[distance_idx].offset, sizeof(float));

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

            int ijk0 = static_cast<int>(floor(input_->points[cp].x * inverse_leaf_size_[0]) -
                    static_cast<float>(min_b_[0]));
            int ijk1 = static_cast<int>(floor(input_->points[cp].y * inverse_leaf_size_[1]) -
                    static_cast<float>(min_b_[1]));
            int ijk2 = static_cast<int>(floor(input_->points[cp].z * inverse_leaf_size_[2]) -
                    static_cast<float>(min_b_[2]));
            // Compute the centroid leaf index
            int idx = ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2];

            Leaf& leaf = _leaves[idx];
            if (leaf.nr_points == 0) {
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
            ++leaf.nr_points;
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
                int ijk0 = static_cast<int>(floor(input_->points[cp].x * inverse_leaf_size_[0]) -
                        static_cast<float>(min_b_[0]));
                int ijk1 = static_cast<int>(floor(input_->points[cp].y * inverse_leaf_size_[1]) -
                        static_cast<float>(min_b_[1]));
                int ijk2 = static_cast<int>(floor(input_->points[cp].z * inverse_leaf_size_[2]) -
                        static_cast<float>(min_b_[2]));

                // Compute the centroid leaf index
                int idx = ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2];

                //int idx = (((input_->points[cp].getArray4fmap () *
                //inverse_leaf_size_).template cast<int> ()).matrix () - min_b_).dot (divb_mul_);

                Leaf& leaf = _leaves[idx];
                if (leaf.nr_points == 0) {
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
                    pcl::for_each_type<FieldList> (pcl::NdCopyPointEigenFunctor<PointT> (
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
                ++leaf.nr_points;
            }
        }

        // Second pass: go over all leaves and compute centroids and covariance matrices
        output.points.reserve(_leaves.size());
        if (_searchable) {
            __voxel_centroidsleaf_indices.reserve(_leaves.size());
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

        for (typename std::map<size_t, Leaf>::iterator it = _leaves.begin();
            it != _leaves.end(); ++it) {
            // Normalize the centroid
            Leaf& leaf = it->second;

            // Normalize the centroid
            leaf.centroid /= static_cast<float> (leaf.nr_points);
            // Point sum used for single pass covariance calculation
            pt_sum = leaf.mean_;
            // Normalize mean
            leaf.mean_ /= leaf.nr_points;

            // If the voxel contains sufficient points,
            // its covariance is calculated and is added to the voxel centroids and output clouds.
            // Points with less than the minimum points will
            // have a can not be accuratly approximated using a normal distribution.
            if (leaf.nr_points >= _min_points_per_voxel) {
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
                    pcl::for_each_type<FieldList> (pcl::NdCopyEigenPointFunctor<PointT> (
                                                     leaf.centroid, output.back()));
                    // ---[ RGB special case
                    if (rgba_index >= 0) {
                        pcl::RGB& rgb = *reinterpret_cast<pcl::RGB*> (reinterpret_cast<char*> (
                        &output.points.back()) + rgba_index);
                        rgb.a = leaf.centroid[centroid_size - 4];
                        rgb.r = leaf.centroid[centroid_size - 3];
                        rgb.g = leaf.centroid[centroid_size - 2];
                        rgb.b = leaf.centroid[centroid_size - 1];
                    }
                }

                // Stores the voxel indice for fast access searching
                if (_searchable) {
                    __voxel_centroidsleaf_indices.push_back(static_cast<int> (it->first));
                }

                // Single pass covariance calculation
                leaf.cov_ = (leaf.cov_ - 2 * (pt_sum * leaf.mean_.transpose())) /
                        leaf.nr_points + leaf.mean_ * leaf.mean_.transpose();
                leaf.cov_ *= (leaf.nr_points - 1.0) / leaf.nr_points;

                //Normalize Eigen Val such that max no more than 100x min.
                eigensolver.compute(leaf.cov_);
                eigen_val = eigensolver.eigenvalues().asDiagonal();
                leaf.evecs_ = eigensolver.eigenvectors();

                if (eigen_val(0, 0) < 0 || eigen_val(1, 1) < 0 || eigen_val(2, 2) <= 0) {
                    leaf.nr_points = -1;
                    continue;
                }

                // Avoids matrices near singularities (eq 6.11)[Magnusson 2009]
                min_covar_eigvalue = _min_covar_eigvalue_mult * eigen_val(2, 2);
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
                    leaf.nr_points = -1;
                }
            }
        }
        output.width = static_cast<uint32_t>(output.points.size());
    }

        /** \brief Flag to determine if voxel structure is searchable. */
        bool _searchable;

        /** \brief Minimum points contained with in a voxel to allow it to be useable. */
        int _min_points_per_voxel;

        /** \brief Minimum allowable ratio between eigenvalues to
        * prevent singular covariance matrices. */
        double _min_covar_eigvalue_mult;

        /** \brief Voxel structure containing all leaf nodes
        * (includes voxels with less than a sufficient number of points). */
        std::map<size_t, Leaf> _leaves;

        /** \brief Point cloud containing centroids of voxels
        * containing atleast minimum number of points. */
        PointCloudPtr _voxel_centroids;

        /** \brief Indices of leaf structurs associated with each point in
        * \ref _voxel_centroids (used for searching). */
        std::vector<int> __voxel_centroidsleaf_indices;

        /** \brief KdTree generated using \ref _voxel_centroids (used for searching). */
        pcl::KdTreeFLANN<PointT> _kdtree;
    };

} // msf
} // localization
} // apollo
#endif //ADU_HAD_MAP_VOXEL_GRID_COVARIANCE_HDMAP_H
