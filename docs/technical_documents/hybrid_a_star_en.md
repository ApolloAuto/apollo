Generate coarse trajectory on the open space 

# Introduction
The goal of this part is to generate the coarse trajectory in the open space. This part of the code consists of four parts: node3d，grid_search，reeds_shepp_path，hybrid_a_star. Hybrid_a_star the most important part of generating the coarse trajectory, it calls the grid_search and reeds_shepp_path.

# Where is the code
Please refer [code](https://github.com/ApolloAuto/apollo/tree/master/modules/planning/open_space/coarse_trajectory_generator/hybrid_a_star.cc)

# Code Reading
![Diagram](images/opne_space_planner.png)

1. Input:  The meaning of the input information is as follows: Vehicle current point(planned start point), goal point(planner end point),ROI_xy_boundary(the maximum and minimum boundary value of x and y), obstacles vertices vector(the corner position information). The function is follow:
   ```  cpp
        bool HybridAStar::Plan(double sx, double sy, double sphi, 
                              double ex, double ey, double ephi,
                              const std::vector<double>& XYbounds,
                              const std::vector<std::vector<common::math::Vec2d>>& obstacles_vertices_vec,HybridAStartResult* result)
   ```
The input HybridAStar::Plan() is called by the open_space_trajectory_provider.cc, please refer(https://github.com/ApolloAuto/apollo/blob/master/modules/planning/tasks/optimizers/open_space_trajectory_generation/open_space_trajectory_provider.cc)

2. Process1: construct obstacles_linesegment_vector. The main method is to form a line segment from a single obstacle point in order; then, each obstacle line segment set is stored in obstacles_linesegment_vector that will be used to generate the DP map.
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
3. Process2: construct the planned point same as Node3d,please refer (https://github.com/ApolloAuto/apollo/blob/master/modules/planning/open_space/coarse_trajectory_generator/node3d.h). The planned starting point and the ending point are constructed in the form of Node3d that will be save to open set and will be checked by the function of ValidityCheck().
    ``` cpp
        start_node_.reset(
        new Node3d({sx}, {sy}, {sphi}, XYbounds_, planner_open_space_config_));
        end_node_.reset(
        new Node3d({ex}, {ey}, {ephi}, XYbounds_, planner_open_space_config_));
    ```

4. Process3: enter the main while loop to get the a set of nodes.
fisrt:Exit trajectory generation when open_pq_ is empty. The open_pq_ is a std::priority_queue type that the first element represents the order of nodes in the open table, the second element represents the cost of nodes which storage in descending order. 

second: Use AnalyticExpansion() function to determine whether there is a collision free trajectory that based on the reeds_shepp_path from current point to target end point. if it exists, exit the while loop search.
    ``` cpp
        if (AnalyticExpansion(current_node)) {
            break;
        }
    ```
third: Store the current point in the close table and According to the number of samples set for the next node, the cycle mode is carried out. generate the next node by Next_node_generator() and use ValidityCheck() to detect this node.

5. Process4: Generate the coarse trajectory by nodes. the GetResult() function is used to generate the coarse trajectory.
   ``` cpp
   bool HybridAStar::GetResult(HybridAStartResult* result)
   ```
6. Output: the optput is partial trajectory information, which include x,y,phi,v,a,steer,s

# Algorithm Detail
   ``` cpp
   bool HybridAStar::ValidityCheck(std::shared_ptr<Node3d> node)
   ```
   the detection method is based on boundary range judgment and boundary overlap judgment.
1. parameter: the input parameter is node which same as node3d 
2. introduction: the function is used to check for collisions 
3. Process: 1.boundary range judgment: if the x and y of node exceed the range of the corresponding x and y of boundary, then return false, reprents invalid; 2.boundary overlap judgment: if the bounding box of vehicle overlaps any line segment, then return false. the overlap check is based on whether the line segment intersects bounding box.

    ``` cpp
    bool GridSearch::GenerateDpMap(const double ex, const double ey, 
                                   const std::vector<double>& XYbounds,
                                   const std::vector<std::vector<common::math::LineSegment2d>> &obstacles_linesegments_vec) 
    ```
    the function is used to generate dp map by dynamic programming, please refer (https://github.com/ApolloAuto/apollo/blob/master/modules/planning/open_space/coarse_trajectory_generator/grid_search.cc)
1. parameter: ex and ey are the postion of goal point, XYbounds_ is the boundary of x and y, obstacles_linesegments_ is the line segments which is composed of boundary point.
2. introduction: the function is used to generate the dp map
3. process: 1. Grid the XYbounds_ according to grid resolution, then get the max grid; 2. dp map store the cost of node.

    ``` cpp
    bool HybridAStar::AnalyticExpansion(std::shared_ptr<Node3d> current_node)
    ```
    the function is used to check if an analystic curve could be connected from current configuration to the end configuration without collision. if so, search ends.
1. parameter: the current node is the starting point of planning and the current location of vehicles.
2. introduction: the function based on the reeds shepp method which 
is a geometric programming algorithm composed of arc and line. Used for search acceleration 
3. process:1. generate the reeds shepps path that the length of the path is optimal and is shortest by the ShortestRSP() function; 2. Check the path is collision free by the RSPCheck() function which call the ValidityCheck(); 3.load the whole RS path as nodes and add to the close set.

    ``` cpp
    bool HybridAStar::AnalyticExpansion(std::shared_ptr<Node3d> current_node)
    ```
    the funtion is used to generate next node based on the current node.
1. parameter: The current node of the search and the next node serial number 
2. introduction: The next node is calculated based on steering wheel uniform sampling and vehicle kinematics.
3. process: 1. The steering angle is calculated according to the number and order of sampling points; 2. that generate the next node                 based on the vehicle kinematics; 3. check if the next node runs outside of XY boundary.

    ``` cpp
    void HybridAStar::CalculateNodeCost(std::shared_ptr<Node3d> current_node,
                                        std::shared_ptr<Node3d> next_node)
    ```
   the function is used to calculate the cost of node
1. parameter: current node(vehicle position) and generated the next node which refer Next_node_generator()
2. introduction: the calculated cost include trajectory cost and heuristic cost considering obstacles based on holonomic.
3. process: 1. the trajectory cost include the current node's trajectory cost and the trajectory cost from current node to next node, trajectory cost is determined by the sampling distance, the gear change between them and the steering change rate. 2. heuristic cost is get from the dp map.