#ifndef CAMERA_PX2_H
#define CAMERA_PX2_H
#include "ProgramArguments.hpp"
void init(int& image_width, int& image_height, ProgramArguments arg);
void start();
bool read_frame(unsigned char** image_data);
void reset_frame();
void stop();
void release();
#endif
