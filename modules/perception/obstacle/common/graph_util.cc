// Copyright 2016 Baidu Inc. All Rights Reserved.
// @author: yanhe (yanhe@baidu.com)
// @file: graph_util.cpp
// @brief:

#include "modules/perception/obstacle/common/graph_util.h"

#include <queue>
#include <iostream>

namespace apollo {
namespace perception {

void connected_component_analysis(const std::vector<std::vector<int> >& graph,
    std::vector<std::vector<int> >& components) {
    int no_item = graph.size();
    std::vector<int> visited;
    visited.resize(no_item, 0);
    std::queue<int> que;
    std::vector<int> component;
    components.clear();

    for (int i = 0; i < no_item; i++) {
        if (visited[i]) {
            continue;
        }
        component.push_back(i);
        que.push(i);
        visited[i] = 1;
        while (!que.empty()) {
            int id = que.front();
            que.pop();
            for (int j = 0; j < graph[id].size(); j++) {
                int nb_id = graph[id][j];
                if (visited[nb_id] == 0) {
                    component.push_back(nb_id);
                    que.push(nb_id);
                    visited[nb_id] = 1;
                }
            }
        }
        components.push_back(component);
        component.clear();
    }
    // std::cout << "Find components \n";
    // for (int i = 0; i < components.size(); i++) {
    //     for (int j = 0; j < components[i].size(); j++) {
    //         std::cout << components[i][j] << "  ";
    //     }
    //     std::cout << std::endl;
    // }
}

} // namespace perception
} // namespace apollo
