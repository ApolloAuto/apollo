#ifndef MYCAMERAHANDLER_H
#define MYCAMERAHANDLER_H

#include "framehandler.h"
struct Result{
    bool successed;
    int warning;
};
class CameraHandler : public FrameHandler
{
public:
    virtual void handleCameraDataFile(const char *path){}
    virtual void handleStereoCameraParam(float focus, float baseline, float pixelSize, int opticalCenterX, int opticalCenterY){}
    virtual void handleUpdateFinished(Result result){}
};

#endif // MYCAMERAHANDLER_H
