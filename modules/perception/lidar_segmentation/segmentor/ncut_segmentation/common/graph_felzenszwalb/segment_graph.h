/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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
Copyright (C) 2006 Pedro Felzenszwalb

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
*/

#pragma once
#include <algorithm>
#include <cmath>

#include "modules/perception/lidar_segmentation/segmentor/ncut_segmentation/common/graph_felzenszwalb/disjoint_set.h"

namespace apollo {
namespace perception {
namespace lidar {

// threshold function
#define THRESHOLD(size, c) (c / size)

typedef struct {
    float w;
    int a;
    int b;
} edge;

bool operator<(const edge &a, const edge &b) {
    return a.w < b.w;
}

/**
 * @brief Segment a graph
 *
 * @param num_vertices number of vertices in graph
 * @param num_edges number of edges in graph
 * @param edges array of edges
 * @param c constant for threshold function
 * @return Universe* returns a disjoint-set forest representing the segmentation
 */
Universe *segment_graph(int num_vertices, int num_edges, edge *edges, float c) {
    // sort edges by weight
    std::sort(edges, edges + num_edges);

    // make a disjoint-set forest
    Universe *u = new Universe(num_vertices);

    // init thresholds
    float *threshold = new float[num_vertices];
    for (int i = 0; i < num_vertices; i++) {
        threshold[i] = THRESHOLD(1, c);
    }

    // for each edge, in non-decreasing weight order...
    for (int i = 0; i < num_edges; i++) {
        edge *pedge = &edges[i];

        // components connected by this edge
        int a = u->find(pedge->a);
        int b = u->find(pedge->b);
        if (a != b) {
            if ((pedge->w <= threshold[a]) && (pedge->w <= threshold[b])) {
                u->join(a, b);
                a = u->find(a);
                threshold[a] = pedge->w + THRESHOLD(u->size(a), c);
            }
        }
    }

    // free up
    delete threshold;
    return u;
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
