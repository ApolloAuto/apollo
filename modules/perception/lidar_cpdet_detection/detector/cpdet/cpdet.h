/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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

#pragma once

#include <deque>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "modules/perception/common/base/object.h"
#include "modules/perception/common/base/point_cloud.h"
#include "modules/perception/common/base/blob.h"
#include "modules/perception/common/inference/inference.h"
#include "modules/perception/common/inference/inference_factory.h"
#include "modules/perception/common/lidar/common/lidar_frame.h"
#include "modules/perception/common/lidar/common/cloud_mask.h"
#include "modules/perception/common/lidar/common/lidar_point_label.h"
#include "modules/perception/lidar_cpdet_detection/interface/base_cpdetector.h"
#include "modules/perception/common/interface/base_down_sample.h"

#include "modules/perception/lidar_cpdet_detection/detector/cpdet/proto/model_param.pb.h"

namespace apollo {
namespace perception {
namespace lidar {

static const int kBoxBlockSize = 7;

struct ResultsOutput {
    float x = 0.0;
    float y = 0.0;
    float z = 0.0;
    float l = 0.0;
    float w = 0.0;
    float h = 0.0;
    float yaw = 0.0;
    int model_type_index = 0;
    float confidence = 0.0;
    std::vector<int> point_ids;
    inline void AddPointId(int point_id) {
        point_ids.push_back(point_id);
    }
    inline void clear() {
        x = 0.0;
        y = 0.0;
        z = 0.0;
        l = 0.0;
        w = 0.0;
        h = 0.0;
        yaw = 0.0;
        model_type_index = 0;
        confidence = 0.0;
        point_ids.clear();
    }
};

class CPDetection : public BaseCPDetector {
    public:
        CPDetection();
        virtual ~CPDetection() = default;

        bool Init(const CPDetectorInitOptions &options =
            CPDetectorInitOptions()) override;

        bool Detect(const CPDetectorOptions &options,
            LidarFrame *frame) override;

        std::string Name() const { return "CPDetection"; }

        void GeneratePfnFeature() {
            if (use_gpu_generate_feature_) {
                GeneratePfnFeatureGPU();
            } else {
                GeneratePfnFeatureCPU();
            }
        }

        void GenerateBackboneFeature(
            const base::Blob<float>* pillar_feature_blob) {
            if (use_gpu_generate_feature_) {
                GenerateBackboneFeatureGPU(pillar_feature_blob);
            } else {
                GenerateBackboneFeatureCPU(pillar_feature_blob);
            }
        }

        void GetObjectsAndAssignPoints() {
            // all-gpu
            if (!use_cpu_get_objects_ && !use_cpu_assign_points_) {
                GetObjectsAndAssignPointsGPU();
                return;
            }
            if (use_cpu_get_objects_) {
                GetObjectsCPU();
            } else {
                GetObjectsGPU();
            }
            if (use_cpu_assign_points_) {
                AssignPointsCPU();
            } else {
                AERROR << "Error: Not Support AssignPoints is False when GetObjects is not False";
                return;
            }
        }
    private:
        void LoadParams(const cpdet::CPDetConfig &cpdet_config);
        
        bool InitModel(const cpdet::CPDetModelParam &model_param);

        void PointCloudPreprocess();

        base::ObjectSubType GetObjectSubType(const int label);

        void GenerateObjects(LidarFrame *frame);

        void SetPointsInROI(std::vector<base::ObjectPtr> *objects);

        void RemoveForePoints(std::vector<base::ObjectPtr> *objects);

        void MapPointsToGridCpu(int* point2grid_data);

        void GeneratePfnFeatureGPU();
        void GeneratePfnFeatureCPU();
        void GenerateBackboneFeatureCPU(const base::Blob<float>* pillar_feature_blob);
        void GenerateBackboneFeatureGPU(const base::Blob<float>* pillar_feature_blob);
        
        void GetObjectsAndAssignPointsGPU();
        void GetObjectsGPU();
        void GetObjectsCPU();
        void AssignPointsCPU();

        void FilterObjectsbyClassNMS(
            std::vector<std::shared_ptr<base::Object>> *objects);

        void FilterObjectsbySemanticType(
            std::vector<std::shared_ptr<base::Object>> *objects);

        void AssignPoints2Boxid(const std::vector<int>& kept_indices);

        void SimpleAssignPoints2Boxid(const std::vector<int>& kept_indices);
        
        void ApplyRotateNms(const bool *rotate_overlapped,
            const int valid_box_num, const int* all_sorted_indices,
            std::vector<int> *box_reverve_flag, bool skip_suppressed);

        std::vector<ResultsOutput> ApplyNMSCPU(std::vector<ResultsOutput>& boxes);

        int ApplyNmsGPU(const int box_num_pre, const int all_res_num, std::vector<int>* kept_indices);

        void DecodeValidObjects(std::vector<int>* kept_indices);

        float CalculateUnionArea(const float *corners1, const float *corners2);

        template <typename T>
        void HOST_SAVE(const T *array, int row, int col,
            std::string filename, bool is_canvus) {
            std::ofstream out_file(filename, std::ios::out);
            if (out_file.is_open()) {
                for (int i = 0; i < row; ++i) {
                    for (int j = 0; j < col; ++j) {
                        if (is_canvus) {
                            if (array[i * col + j] == 0) {
                                continue;
                            }
                            out_file << array[i * col + j] << ", i: " << i << ", j: " << j << "\n";
                        } else{
                            out_file << array[i * col + j] << " ";
                        }
                    }
                    out_file << "\n";
                }
            }
            out_file.close();
        };

        template <typename T>
        void DEVICE_SAVE(const T *array, int row, int col,
            std::string filename, bool is_canvus = false) {
            T *temp_ = new T[row * col];
            cudaMemcpy(temp_, array,
                row * col * sizeof(T), cudaMemcpyDeviceToHost);
            HOST_SAVE<T>(temp_, row, col, filename, is_canvus);
            delete[] temp_;
        };

    private:
        // cpdet model param
        cpdet::CPDetConfig cpdet_config_;

        std::shared_ptr<inference::Inference> pfn_inference_;
        std::shared_ptr<inference::Inference> backbone_inference_;
        
        // pfn model blobs
        // input: (points_size, 9)
        std::shared_ptr<base::Blob<float>> voxels_blob_;
        // output: (points_size, 48/64)
        std::shared_ptr<base::Blob<float>> pfn_pillar_feature_blob_;
        // backbone model blobs
        // input: (1, 64, map_x, map_y)
        std::shared_ptr<base::Blob<float>> canvas_feature_blob_;
        // output-box: (1, 6 * task_number, head_x, head_y)
        // output-cls: (1, 4, head_x, head_y)
        // output-dir: (1, 2 * task_number, head_x, head_y)
        std::shared_ptr<base::Blob<float>> output_box_blob_;
        std::shared_ptr<base::Blob<float>> output_cls_blob_;          
        std::shared_ptr<base::Blob<float>> output_dir_blob_;

        // feature generate
        std::shared_ptr<base::Blob<int>> point2grid_blob_ = nullptr;
        std::shared_ptr<base::Blob<int>> grid2pointnum_blob_ = nullptr;
        
        // get objects
        std::shared_ptr<base::Blob<float>> all_res_box_blob_ = nullptr;
        std::shared_ptr<base::Blob<float>> all_res_conf_blob_ = nullptr;
        std::shared_ptr<base::Blob<int>> all_res_cls_blob_ = nullptr;
        std::shared_ptr<base::Blob<float>> res_box_blob_ = nullptr;
        std::shared_ptr<base::Blob<float>> res_conf_blob_ = nullptr;
        std::shared_ptr<base::Blob<int>> res_cls_blob_ = nullptr;
        std::shared_ptr<base::Blob<int>> res_box_num_blob_ = nullptr;
        std::shared_ptr<base::Blob<float>> score_class_map_blob_ = nullptr;

        // NMS
        std::shared_ptr<base::Blob<int>> res_sorted_indices_blob_ = nullptr;
        std::shared_ptr<base::Blob<float>> box_for_nms_blob_ = nullptr;
        std::shared_ptr<base::Blob<float>> box_corner_blob_ = nullptr;
        std::shared_ptr<base::Blob<float>> remain_conf_blob_ = nullptr;
        std::shared_ptr<base::Blob<bool>> rotate_overlapped_blob_ = nullptr;
        std::shared_ptr<base::Blob<int>> kept_indices_blob_ = nullptr;
        // assign point
        std::shared_ptr<base::Blob<float>> box_corners_blob_ = nullptr;
        std::shared_ptr<base::Blob<float>> box_rects_blob_ = nullptr;
        std::shared_ptr<base::Blob<int>> valid_point_num_blob_ = nullptr;
        std::shared_ptr<base::Blob<int>> valid_point_indices_blob_ = nullptr;
        std::shared_ptr<base::Blob<int>> valid_point2boxid_blob_ = nullptr;

        // reference pointer of lidar frame
        LidarFrame *lidar_frame_ref_ = nullptr;
        std::shared_ptr<
            base::AttributePointCloud<base::PointF>> original_cloud_;
        std::shared_ptr<
            base::AttributePointCloud<base::PointD>> original_world_cloud_;
        base::PointFCloudPtr cur_cloud_ptr_;
        base::PointF* pc_gpu_ = nullptr;
        int model_cloud_size_ = 0;
        int total_cloud_size_ = 0;
        int max_point_number_ = 200000;
        int max_valid_point_size_ = 0;

        // point cloud range
        float x_min_range_ = -51.2;
        float x_max_range_ = 51.2;
        float y_min_range_ = -51.2;
        float y_max_range_ = 51.2;
        float z_min_range_ = -3.5;
        float z_max_range_ = 3.5;
        float voxel_x_size_ = 0.2;
        float voxel_y_size_ = 0.2;
        float voxel_z_size_ = 7;
        float x_offset_ = 0.0;
        float y_offset_ = 0.0;
        float z_offset_ = 0.0;
        int grid_x_size_ = 512;
        int grid_y_size_ = 512;
        int grid_z_size_ = 1;
        int map_size_ = 512 * 512;
        float point_dim_ = 4;
        bool enable_rotate_45degree_ = true;
        bool use_input_norm_ = true;

        // cnnseg feature
        bool use_cnnseg_features_ = true;
        int cnnseg_feature_dim_ = 16;
        float height_bin_min_height_ = -3.5;
        float height_bin_max_height_ = 3.5;
        float height_bin_voxel_size_ = 0.5;
        int height_bin_dim_ = 10;

        // cnnseg feature data
        float* max_height_data_ = nullptr;
        float* mean_height_data_ = nullptr;
        float* count_data_ = nullptr;
        float* top_intensity_data_ = nullptr;
        float* mean_intensity_data_ = nullptr;
        float* nonempty_data_ = nullptr;
        float* height_bin_data_ = nullptr;

        // feature generate
        bool use_gpu_generate_feature_ = true;
        int max_voxel_num_ = 40000;
        int max_points_in_voxel_ = 20;
        int voxel_feature_dim_ = 9;
        int pillar_feature_dim_ = 48;
        int num_classes_ = 4;

        // pointcloud preprocess
        bool enable_downsample_ = false;

        // post process
        int downsample_size_ = 4;
        int num_tasks_ = 1;
        int head_x_size_ = 212;
        int head_y_size_ = 212;
        int head_map_size_ = 44944;
        int nms_pre_max_size_ = 500;
        int nms_post_max_size_ = 300;
        int score_thresh_ = 0.1;
        int nms_overlap_thresh_ = 0.01;
        int min_pts_num_fg_ = 5;
        float max_candidate_num_ = 1;
        float top_enlarge_value_ = 0.0;
        float bottom_enlarge_value_ = 0.0;
        float width_enlarge_value_ = 0.0;
        float length_enlarge_value_ = 0.0;
        bool use_cpu_get_objects_ = false;
        bool use_cpu_assign_points_ = false;

        // model objects filter strategy
        bool remove_semantic_ground_ = false;
        bool remove_raw_ground_ = false;
        bool point_unique_ = false;
        bool inter_class_nms_ = false;
        bool filter_by_semantic_type_ = false;
        std::map<int, std::pair<float, float>> fore_semantic_filter_map_;
        float class_nms_iou_thres_ = 0.25;
        bool nms_strategy_ = false;
        
        std::vector<int> num_classes_in_task_;
        std::vector<float> score_thresh_map_;
        std::map<int, std::pair<std::string, float>> score_thresh_per_class_;

        std::map<std::string, int> head_map_;
        std::map<std::string, int> feature_offset_;

        const int kGPUThreadSize = 512;

        std::vector<ResultsOutput> res_outputs_;

        cudaStream_t stream_ = 0;

        std::shared_ptr<BaseDownSample> down_sample_;
};  // class CPDetection

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
