/******************************************************************************
 * Copyright 2020 The Beijing Smarter Eye Technology Co.Ltd Authors. All
 * Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#ifndef OBSTACLEPAINTER_H
#define OBSTACLEPAINTER_H

#include "smartpainterdef.h"

class OutputObstacles;

typedef struct _SmartRoi_
{
    int x;
    int y;
    int width;
    int height;
}SmartRoi;

class SMARTPAINTER_SHARED_EXPORT ObstaclePainter
{
public:
    ObstaclePainter();
    ~ObstaclePainter();

    static bool paintObstacle(void * _obstacleParam, unsigned char * _rgbImageData, int _width, int _height,
                              bool showDetials, bool singleObs);

protected:
    static void paintRectBorder(unsigned char *_imageData, OutputObstacles &_rectBorder,
                                int _width, int _height, bool isWarn);
    static void drawObsFrame(unsigned char *_imageData, OutputObstacles &_rectBorder,
                             int _width, int _height, int _offsetX, int _offsetY, bool showDetails = true);
    static void drawObsInfo(unsigned char *_imageData, OutputObstacles &_rectBorder,
                            int _width, int _height,const unsigned char color[3]);
};

#endif // OBSTACLEPAINTER_H
