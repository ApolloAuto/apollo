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
/* a simple image class */
#pragma once
#include <cstring>

namespace apollo {
namespace perception {
namespace lidar {

template <class T>
class Image {
public:
    /* create an image */
    Image(const int width, const int height, const bool init = true);
    /* delete an image */
    ~Image();

    /**
     * @brief Init an image
     *
     */
    void init(const T &val);
    /**
     * @brief Copy an image
     *
     * @return Image<T>*
     */
    Image<T> *copy() const;

    /**
     * @brief Get the width of an image
     *
     * @return int
     */
    int width() const {
        return _w;
    }

    /**
     * @brief Get the height of an image
     *
     * @return int
     */
    int height() const {
        return _h;
    }

    /* image data. */
    T *_data;

    /* row pointers. */
    T **_access;

private:
    int _w;
    int _h;
};
/* use imRef to access image data. */
#define imRef(im, x, y) (im->_access[y][x])
/* use imPtr to get pointer to image data. */
#define imPtr(im, x, y) &(im->_access[y][x])
template <class T>
Image<T>::Image(const int width, const int height, const bool init) {
    _w = width;
    _h = height;
    _data = new T[_w * _h];  // allocate space for image data
    _access = new T *[_h];   // allocate space for row pointers

    // initialize row pointers
    for (int i = 0; i < _h; i++) {
        _access[i] = _data + (i * _w);
    }

    if (init) {
        memset(_data, 0, _w * _h * sizeof(T));
    }
}
template <class T>
Image<T>::~Image() {
    delete[] _data;
    delete[] _access;
}
template <class T>
void Image<T>::init(const T &val) {
    T *ptr = imPtr(this, 0, 0);
    T *end = imPtr(this, _w - 1, _h - 1);
    while (ptr <= end) {
        *ptr++ = val;
    }
}
template <class T>
Image<T> *Image<T>::copy() const {
    Image<T> *im = new Image<T>(_w, _h, false);
    memcpy(im->_data, _data, _w * _h * sizeof(T));
    return im;
}
}  // namespace lidar
}  // namespace perception
}  // namespace apollo
