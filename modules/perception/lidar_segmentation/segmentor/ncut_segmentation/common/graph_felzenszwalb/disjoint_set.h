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

namespace apollo {
namespace perception {
namespace lidar {

// disjoint-set forests using union-by-rank and path compression (sort of).

typedef struct {
    int rank;
    int p;
    int size;
} uni_elt;

class Universe {
public:
    explicit Universe(int elements);
    ~Universe();
    /**
     * @brief Find parent of x
     *
     * @param x
     * @return int
     */
    int find(int x);
    /**
     * @brief Join set x and set y
     *
     * @param x
     * @param y
     */
    void join(int x, int y);
    /**
     * @brief Get size of set x
     *
     * @param x
     * @return int
     */
    int size(int x) const {
        return _elts[x].size;
    }
    /**
     * @brief Get set number
     *
     * @return int
     */
    int num_sets() const {
        return _num;
    }

private:
    uni_elt *_elts;
    int _num;
};

Universe::Universe(int elements) {
    _elts = new uni_elt[elements];
    _num = elements;
    for (int i = 0; i < elements; i++) {
        _elts[i].rank = 0;
        _elts[i].size = 1;
        _elts[i].p = i;
    }
}

Universe::~Universe() {
    delete[] _elts;
}

int Universe::find(int x) {
    int y = x;
    while (y != _elts[y].p) {
        y = _elts[y].p;
    }
    _elts[x].p = y;
    return y;
}

void Universe::join(int x, int y) {
    if (_elts[x].rank > _elts[y].rank) {
        _elts[y].p = x;
        _elts[x].size += _elts[y].size;
    } else {
        _elts[x].p = y;
        _elts[y].size += _elts[x].size;
        if (_elts[x].rank == _elts[y].rank) {
            _elts[y].rank++;
        }
    }
    _num--;
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
