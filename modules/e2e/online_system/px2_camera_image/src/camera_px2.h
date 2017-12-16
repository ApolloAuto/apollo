#ifndef CAMERA_PX2_H
#define CAMERA_PX2_H
void init(int& image_width, int& image_height, const char* csi_port);
void start();
bool read_frame(unsigned char** image_data);
void reset_frame();
void stop();
void release();
#endif
