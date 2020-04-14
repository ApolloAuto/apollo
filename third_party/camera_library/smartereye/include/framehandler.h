#ifndef FRAMEHANDLER_H
#define FRAMEHANDLER_H

struct RawImageFrame;

class FrameHandler
{
public:
    virtual void handleRawFrame(const RawImageFrame *rawFrame) = 0;
};

#endif // FRAMEHANDLER_H
