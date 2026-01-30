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

/*
 * @file
 */

#include "cyber/common/file.h"
#include "modules/planning/planning_open_space/coarse_trajectory_generator/hybrid_a_star.h"

namespace apollo {
namespace planning {

class HybridAObstacleContainer {
public:
    HybridAObstacleContainer() = default;
    void AddVirtualObstacle(double* obstacle_x, double* obstacle_y, int vertice_num) {
        std::vector<common::math::Vec2d> obstacle_vertices;
        for (int i = 0; i < vertice_num; i++) {
            common::math::Vec2d vertice(obstacle_x[i], obstacle_y[i]);
            obstacle_vertices.emplace_back(vertice);
        }
        obstacles_list.emplace_back(obstacle_vertices);
    }
    const std::vector<std::vector<common::math::Vec2d>>& GetObstaclesVerticesVec() {
        return obstacles_list;
    }

private:
    std::vector<std::vector<common::math::Vec2d>> obstacles_list;
};

class HybridASoftBoundaryContainer {
public:
    HybridASoftBoundaryContainer() = default;
    void AddVirtualSoftBoundary(double* obstacle_x, double* obstacle_y, int vertice_num) {
        std::vector<common::math::Vec2d> soft_boundary_vertices;
        for (int i = 0; i < vertice_num; i++) {
            common::math::Vec2d vertice(obstacle_x[i], obstacle_y[i]);
            soft_boundary_vertices.emplace_back(vertice);
        }
        soft_boundary_list.emplace_back(soft_boundary_vertices);

        for (int i = 0; i < soft_boundary_list.size(); ++i) {
            for (const auto& vertice : soft_boundary_list[i]) {
                AERROR << "Soft boundary: " << i << ", " << vertice.x() << ", " << vertice.y();
            }
        }
    }
    const std::vector<std::vector<common::math::Vec2d>>& GetSoftBoundaryVerticesVec() {
        return soft_boundary_list;
    }

private:
    std::vector<std::vector<common::math::Vec2d>> soft_boundary_list;
};

class HybridAResultContainer {
public:
    HybridAResultContainer() = default;
    void LoadResult() {
        x_ = std::move(result_.x);
        y_ = std::move(result_.y);
        phi_ = std::move(result_.phi);
        v_ = std::move(result_.v);
        a_ = std::move(result_.a);
        steer_ = std::move(result_.steer);
    }
    std::vector<double>* GetX() {
        return &x_;
    }
    std::vector<double>* GetY() {
        return &y_;
    }
    std::vector<double>* GetPhi() {
        return &phi_;
    }
    std::vector<double>* GetV() {
        return &v_;
    }
    std::vector<double>* GetA() {
        return &a_;
    }
    std::vector<double>* GetSteer() {
        return &steer_;
    }
    HybridAStartResult* PrepareResult() {
        return &result_;
    }

private:
    HybridAStartResult result_;
    std::vector<double> x_;
    std::vector<double> y_;
    std::vector<double> phi_;
    std::vector<double> v_;
    std::vector<double> a_;
    std::vector<double> steer_;
};

extern "C" {
HybridAStar* CreatePlannerPtr() {
    FLAGS_planner_open_space_config_filename
            = "/apollo/modules/planning/planning_component/"
              "conf/planner_hybrid_a_star_config.pb.txt";
    apollo::planning::WarmStartConfig planner_open_space_config_;

    ACHECK(apollo::cyber::common::GetProtoFromFile(
            FLAGS_planner_open_space_config_filename, &planner_open_space_config_))
            << "Failed to load open space config file " << FLAGS_planner_open_space_config_filename;
    return new HybridAStar(planner_open_space_config_);
}
HybridAObstacleContainer* CreateObstaclesPtr() {
    return new HybridAObstacleContainer();
}
HybridASoftBoundaryContainer* CreateSoftBoundaryPtr() {
    return new HybridASoftBoundaryContainer();
}
HybridAResultContainer* CreateResultPtr() {
    return new HybridAResultContainer();
}
void AddVirtualObstacle(
        HybridAObstacleContainer* obstacles_ptr,
        double* obstacle_x,
        double* obstacle_y,
        int vertice_num) {
    obstacles_ptr->AddVirtualObstacle(obstacle_x, obstacle_y, vertice_num);
}
void AddVirtualSoftBoundary(
        HybridASoftBoundaryContainer* soft_boundary_ptr,
        double* soft_boundary_x,
        double* soft_boundary_y,
        int vertice_num) {
    soft_boundary_ptr->AddVirtualSoftBoundary(soft_boundary_x, soft_boundary_y, vertice_num);
}
bool Plan(
        HybridAStar* planner_ptr,
        HybridAObstacleContainer* obstacles_ptr,
        HybridASoftBoundaryContainer* soft_boundary_ptr,
        HybridAResultContainer* result_ptr,
        double sx,
        double sy,
        double sphi,
        double ex,
        double ey,
        double ephi,
        double* XYbounds) {
    AERROR << "sx: " << sx;
    std::vector<double> XYbounds_(XYbounds, XYbounds + 4);
    return planner_ptr->Plan(
            sx,
            sy,
            sphi,
            ex,
            ey,
            ephi,
            XYbounds_,
            obstacles_ptr->GetObstaclesVerticesVec(),
            result_ptr->PrepareResult(),
            soft_boundary_ptr->GetSoftBoundaryVerticesVec(),
            false);
}
void GetResult(
        HybridAResultContainer* result_ptr,
        double* x,
        double* y,
        double* phi,
        double* v,
        double* a,
        double* steer,
        size_t* output_size) {
    result_ptr->LoadResult();
    size_t size = result_ptr->GetX()->size();
    std::cout << "return size is " << size << std::endl;
    for (size_t i = 0; i < size; i++) {
        x[i] = result_ptr->GetX()->at(i);
        y[i] = result_ptr->GetY()->at(i);
        phi[i] = result_ptr->GetPhi()->at(i);
        v[i] = result_ptr->GetV()->at(i);
    }
    for (size_t i = 0; i < size - 1; i++) {
        a[i] = result_ptr->GetA()->at(i);
        steer[i] = result_ptr->GetSteer()->at(i);
    }
    *output_size = size;
}
};

}  // namespace planning
}  // namespace apollo
