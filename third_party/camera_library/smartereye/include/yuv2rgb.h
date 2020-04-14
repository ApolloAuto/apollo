#ifndef YUV2RGB_H
#define YUV2RGB_H

#include "dataprocessordef.h"

class RGB{
public:
    char r, g, b;
};

class DATAPROCESSOR_SHARED_EXPORT YuvToRGB
{
public:
    static RGB Yuv2Rgb(char Y, char U, char V);
    static RGB YCbCr2Rgb(unsigned char Y, unsigned char Cb, unsigned char Cr);
    static char *YCbYCr2Rgb(const unsigned char* src, char* dest, int width, int height);
    static char *YCbYCrPlannar2Rgb(const unsigned char* src, char* dest, int width, int height);
    static unsigned char *YCbYCrGetY(const unsigned char* src, unsigned char* dest, int width, int height);
};

#endif // YUV2RGB_H
