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
