# GENERATE COARSE TRAJECTORY IN THE OPEN SPACE

# Introduction
The goal of htbrid_a_star is to generate the coarse trajectory in the open space. Hybrid_a_star contains node3d， grid_search， reeds_shepp_path and hybrid_a_star. hybrid_a_star is the most important component generating the coarse trajectory and call the grid_search and reeds_shepp_path.

# Where is the code
Please refer to [hybrid a star.cc](https://github.com/ApolloAuto/apollo/blob/master/modules/planning/planning_open_space/coarse_trajectory_generator/hybrid_a_star.cc)

# Code Reading
1. Input: current point(planned start point), goal point(planned end point), ROI_xy_boundary(the maximum and minimum boundary value of x and y), obstacles vertices vector(the corner position information). Optional, soft_boundary_vertices_vec (vectors which want to far away), reeds_sheep_last_straight (is the rs curve straight). The function is follow:
   ```  cpp
        bool HybridAStar::Plan(
            double sx, double sy, double sphi, double ex, double ey, double ephi,
            const std::vector<double>& XYbounds,
            const std::vector<std::vector<common::math::Vec2d>>& obstacles_vertices_vec,
            HybridAStartResult* result,
            const std::vector<std::vector<common::math::Vec2d>>&
                soft_boundary_vertices_vec,
            bool reeds_sheep_last_straight)
   ```
The input HybridAStar::Plan() is called by the open_space_trajectory_provider.cc, please refer to [open_space_trajectory_provider.cc](https://github.com/ApolloAuto/apollo/blob/master/modules/planning/tasks/open_space_trajectory_provider/open_space_trajectory_provider.cc)

2. Construct obstacles_linesegment_vector. The main method is to form a line segment from a single obstacle point in order; then, each obstacle line segment is stored in obstacles_linesegment_vector that will be used to generate the DP map.
    ``` cpp
        std::vector<std::vector<common::math::LineSegment2d>>
        obstacles_linesegments_vec;
        for (const auto& obstacle_vertices : obstacles_vertices_vec) {
            size_t vertices_num = obstacle_vertices.size();
            std::vector<common::math::LineSegment2d> obstacle_linesegments;
            for (size_t i = 0; i < vertices_num - 1; ++i) {
            common::math::LineSegment2d line_segment = common::math::LineSegment2d(
                obstacle_vertices[i], obstacle_vertices[i + 1]);
            obstacle_linesegments.emplace_back(line_segment);
            }
            obstacles_linesegments_vec.emplace_back(obstacle_linesegments);
        }
        obstacles_linesegments_vec_ = std::move(obstacles_linesegments_vec);
    ```
3. Construct the planned point same as Node3d, please refer to [node3d.h](https://github.com/ApolloAuto/apollo/blob/master/modules/planning/planning_open_space/coarse_trajectory_generator/node3d.h). The planned starting point and the ending point are constructed in the form of Node3d that will be save to open set and will be checked by the ValidityCheck() function.
    ``` cpp
        start_node_.reset(
        new Node3d({sx}, {sy}, {sphi}, XYbounds_, planner_open_space_config_));
        end_node_.reset(
        new Node3d({ex}, {ey}, {ephi}, XYbounds_, planner_open_space_config_));
    ```

4. Enter the main while loop to get a set of nodes.
   1. Exit trajectory generation when open_pq_ is empty. The open_pq_ is a std::priority_queue type that the first element represents the order of nodes in the open set, the second element represents the cost of nodes which storage in descending order. 
   2. Use AnalyticExpansion() function to determine whether there is a collision free trajectory that based on the reeds_shepp_path from current point to target end point. if it exists, exit the while loop search.
      ``` cpp
        if (AnalyticExpansion(current_node)) {
            break;
        }
      ```
   3. Store the current point in the close set and Expand the next node according to the bicycle kinematics model. The number of nodes is a parameter. Generate the next node by Next_node_generator() function and use ValidityCheck() function to detect this node.

5. Generate the coarse trajectory by nodes. The GetResult() function is used to generate the coarse trajectory.
   ``` cpp
   bool HybridAStar::GetResult(HybridAStartResult* result)
   ```
6. Output: The optput is partial trajectory information, which include x,y,phi,v,a,steer,s.

# Algorithm Detail
   ``` cpp
   bool HybridAStar::ValidityCheck(std::shared_ptr<Node3d> node)
   ```
   The detection method is based on boundary range judgment and boundary overlap judgment.
1. Parameter: the input parameter is node which same as node3d. 
2. Introduction: the function is used to check for collisions. 
3. Process detail: 
   1. Boundary range judgment. If the x and y of node exceed the range of the corresponding x and y of boundary, then return false,         reprents invalid. 
   2. Boundary overlap judgment. If the bounding box of vehicle overlaps any line segment, then return false. Judge the overlap by          whether the line and box intersect.

    ``` cpp
    bool GridSearch::GenerateDpMap(
        const double ex,
        const double ey,
        const std::vector<double>& XYbounds,
        const std::vector<std::vector<common::math::LineSegment2d>>&
            obstacles_linesegments_vec,
        const std::vector<std::vector<common::math::LineSegment2d>>&
            soft_boundary_linesegments_vec) 
    ```
    the function is used to generate dp map by dynamic programming, please refer  [grid_search.cc](https://github.com/ApolloAuto/apollo/blob/master/modules/planning/planning_open_space/coarse_trajectory_generator/grid_search.cc)
1. Parameter: ex and ey are the postion of goal point, XYbounds_ is the boundary of x and y, obstacles_linesegments_ is the line segments which is composed of boundary point.
2. Introduction: the function is used to generate the dp map
3. Process detail: 
   1. Grid the XYbounds_ according to grid resolution, then get the max grid.
   2. Dp map store the cost of node.

    ``` cpp
    bool HybridAStar::AnalyticExpansion(std::shared_ptr<Node3d> current_node, std::shared_ptr<Node3d>* candidate_final_node)
    ```
    The function is used to check if an analystic curve could be connected from current configuration to the end configuration without collision. if so, search ends.
1. Parameter: current node is start point of planning.
2. introduction: the function based on the reeds shepp method which is a geometric algorithm composed of arc and line. Reeds shepp is       used for search acceleration. 
3. Process detail:
   1. Generate the reeds shepps path by the ShortestRSP() function. The length is optimal and shortest.
   2. Check the path is collision free by the RSPCheck() function which call the ValidityCheck() function.
   3. Load the whole reeds shepp path as nodes and add nodes to the close set.

    ``` cpp
    bool HybridAStar::AnalyticExpansion(std::shared_ptr<Node3d> current_node, std::shared_ptr<Node3d>* candidate_final_node)
    ```
    The funtion is used to generate next node based on the current node.
1. Parameter: the current node of the search and the next node serial number 
2. Introduction: the next node is calculated based on steering wheel uniform sampling and vehicle kinematics.
3. Process detail: 
   1. The steering angle is calculated according to the number and order of sampling points.
   2. Generate the next node according to the kinematic model.
   3. Check if the next node runs outside of XY boundary.

    ``` cpp
    void HybridAStar::CalculateNodeCost(std::shared_ptr<Node3d> current_node,
                                        std::shared_ptr<Node3d> next_node)
    ```
   The function is used to calculate the cost of node.
1. Parameter: current node(vehicle position) and next node.
2. Introduction: the calculated cost include trajectory cost and heuristic cost considering obstacles based on holonomic.
3. Process detail: 
   1. the trajectory cost include the current node's trajectory cost and the trajectory cost from current node to next node.
   2. trajectory cost is determined by the sampling distance, the gear change between them and the steering change rate. 
   3. heuristic cost is get from the dp map.