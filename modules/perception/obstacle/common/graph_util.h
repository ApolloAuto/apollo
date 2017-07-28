#ifndef MODULES_PERCEPTION_OBSTACLE_COMMON_GRAPH_UTIL_H_
#define MODULES_PERCEPTION_OBSTACLE_COMMON_GRAPH_UTIL_H_

#include<vector>

namespace apollo {
namespace perception {

/*bfs based component analysis*/
void connected_component_analysis(const std::vector<std::vector<int> >& graph,
    std::vector<std::vector<int> >& components);

} // namespace perception
} // namespace apollo

#endif // MODULES_PERCEPTION_OBSTACLE_COMMON_GRAPH_UTIL_H_
