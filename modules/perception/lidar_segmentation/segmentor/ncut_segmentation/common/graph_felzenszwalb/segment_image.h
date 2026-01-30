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

#include <cstdlib>

#include "modules/perception/lidar_segmentation/segmentor/ncut_segmentation/common/graph_felzenszwalb/filter.h"
#include "modules/perception/lidar_segmentation/segmentor/ncut_segmentation/common/graph_felzenszwalb/image.h"
#include "modules/perception/lidar_segmentation/segmentor/ncut_segmentation/common/graph_felzenszwalb/misc.h"
#include "modules/perception/lidar_segmentation/segmentor/ncut_segmentation/common/graph_felzenszwalb/segment_graph.h"

namespace apollo {
namespace perception {
namespace lidar {

/**
 * @brief Dissimilarity measure between pixels
 *
 * @param I image data
 * @param x1
 * @param y1
 * @param x2
 * @param y2
 * @return float
 */
inline float diff(Image<float> *I, int x1, int y1, int x2, int y2) {
    return std::fabs(imRef(I, x1, y1) - imRef(I, x2, y2));
}

/**
 * @brief Segment an image
 *
 * @param im image to segment
 * @param sigma to smooth the image
 * @param c constant for threshold function
 * @param min_size minimum component size (enforced by post-processing stage)
 * @param num_ccs number of connected components in the segmentation
 * @return Image<int>* returns a color image representing the segmentation
 */
Image<int> *segment_image(Image<float> *im, float sigma, float c, int min_size, int *num_ccs) {
    int width = im->width();
    int height = im->height();
    // smooth each color channel
    Image<float> *smooth_r = smooth(im, sigma);
    // build graph
    edge *edges = new edge[width * height * 4];
    int num = 0;
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            if (x < width - 1) {
                edges[num].a = y * width + x;
                edges[num].b = y * width + (x + 1);
                edges[num].w = diff(smooth_r, x, y, x + 1, y);
                num++;
            }
            if (y < height - 1) {
                edges[num].a = y * width + x;
                edges[num].b = (y + 1) * width + x;
                edges[num].w = diff(smooth_r, x, y, x, y + 1);
                num++;
            }
            if ((x < width - 1) && (y < height - 1)) {
                edges[num].a = y * width + x;
                edges[num].b = (y + 1) * width + (x + 1);
                edges[num].w = diff(smooth_r, x, y, x + 1, y + 1);
                num++;
            }
            if ((x < width - 1) && (y > 0)) {
                edges[num].a = y * width + x;
                edges[num].b = (y - 1) * width + (x + 1);
                edges[num].w = diff(smooth_r, x, y, x + 1, y - 1);
                num++;
            }
        }
    }
    delete smooth_r;
    // segment
    Universe *u = segment_graph(width * height, num, edges, c);
    // post process small components
    for (int i = 0; i < num; i++) {
        int a = u->find(edges[i].a);
        int b = u->find(edges[i].b);
        if ((a != b) && ((u->size(a) < min_size) || (u->size(b) < min_size)))
            u->join(a, b);
    }
    delete[] edges;
    *num_ccs = u->num_sets();
    Image<int> *output = new Image<int>(width, height);
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int comp = u->find(y * width + x);
            imRef(output, x, y) = comp;
        }
    }
    delete u;
    return output;
}
}  // namespace lidar
}  // namespace perception
}  // namespace apollo
