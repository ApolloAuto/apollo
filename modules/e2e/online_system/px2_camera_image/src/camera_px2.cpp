/******************************************************************************
  * Copyright 2017 The Apollo Authors. All Rights Reserved.
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
#include "camera_px2.h"

#include <iostream>
#include <signal.h>
#include <fstream>

#include <stdio.h>
#include <stdlib.h>
#include <memory>

#include <execinfo.h>
#include <unistd.h>

#include <cstring>
#include <functional>
#include <list>
#include <iomanip>
#include <thread>
#include <ctime>

#include <chrono>
#include <mutex>
#include <condition_variable>

// SAMPLE COMMON
#include "Checks.hpp"
#include "WindowGLFW.hpp"
#include "WindowEGL.hpp"
#include "ProgramArguments.hpp"
#include "ConsoleColor.hpp"

// CORE
#include <dw/core/Context.h>
#include <dw/core/Logger.h>

// RENDERER
#include <dw/renderer/Renderer.h>

// HAL
#include <dw/sensors/Sensors.h>
#include <dw/sensors/SensorSerializer.h>
#include <dw/sensors/camera/Camera.h>

// IMAGE
#include <dw/image/FormatConverter.h>
#include <dw/image/ImageStreamer.h>

//------------------------------------------------------------------------------
// Variables
//------------------------------------------------------------------------------
static volatile bool g_run = true;

// 1KB should be plenty for data lines from any sensor
// Actual size is returned during runtime
#define MAX_EMBED_DATA_SIZE (1024 * 1024)
NvMediaISCEmbeddedData sensorData;

//SDK objects
WindowBase *window            = nullptr;
dwContextHandle_t sdk         = DW_NULL_HANDLE;
dwRendererHandle_t renderer   = DW_NULL_HANDLE;
dwSALHandle_t sal             = DW_NULL_HANDLE;
dwSensorHandle_t cameraSensor = DW_NULL_HANDLE;

// create HAL and camera
uint32_t image_width_ = 0;
uint32_t image_height_ = 0;
dwImageType cameraImageType;

// start
dwImageProperties cameraImageProperties;
dwCameraProperties cameraProperties;
std::vector<dwImageNvMedia *> rgbaImagePool;
dwImageFormatConverterHandle_t yuv2rgba = DW_NULL_HANDLE;
dwImageStreamerHandle_t nvm2gl = DW_NULL_HANDLE;
dwImageStreamerHandle_t nvm2cpu = DW_NULL_HANDLE;

dwImageCPU *frameCPU = nullptr;

//------------------------------------------------------------------------------
void initGL(WindowBase **window)
{
    *window = new WindowOffscreenEGL(1280, 800);
}

//------------------------------------------------------------------------------
void initSdk(dwContextHandle_t *context, WindowBase *window)
{
    // create a Logger to log to console
    // we keep the ownership of the logger at the application level
    dwLogger_initialize(getConsoleLoggerCallback(true));
    dwLogger_setLogLevel(DW_LOG_VERBOSE);

    // instantiate Driveworks SDK context
    dwContextParameters sdkParams;
    memset(&sdkParams, 0, sizeof(dwContextParameters));

#ifdef VIBRANTE
    sdkParams.eglDisplay = window->getEGLDisplay();
#else
    (void)window;
#endif

    dwInitialize(context, DW_VERSION, &sdkParams);
}

//------------------------------------------------------------------------------
void initRenderer(dwRendererHandle_t *renderer,
                  dwContextHandle_t context, WindowBase *window)
{
    dwStatus result;

    result = dwRenderer_initialize(renderer, context);
    if (result != DW_SUCCESS)
        throw std::runtime_error(std::string("Cannot init renderer: ") +
                                 dwGetStatusName(result));

    // Set some renderer defaults
    dwRect rect;
    rect.width   = window->width();
    rect.height  = window->height();
    rect.x = 0;
    rect.y = 0;

    dwRenderer_setRect(rect, *renderer);
}

//------------------------------------------------------------------------------
void initSensors(dwSALHandle_t *sal, dwSensorHandle_t *camera,
                 uint32_t *imageWidth, uint32_t *imageHeight, dwImageType *cameraImageType,
                 dwContextHandle_t context, ProgramArguments arg)
{
    dwStatus result;

    result = dwSAL_initialize(sal, context);
    if (result != DW_SUCCESS) {
        std::cerr << "Cannot initialize SAL: "
                    << dwGetStatusName(result) << std::endl;
        exit(1);
    }

    // create GMSL Camera interface
    dwSensorParams params;
    std::string parameterString = arg.parameterString();
    parameterString             += ",output-format=yuv+data";
    params.parameters           = parameterString.c_str();
    params.protocol             = "camera.gmsl";
    result = dwSAL_createSensor(camera, params, *sal);
    if (result != DW_SUCCESS) {
        std::cerr << "Cannot create driver: camera.gmsl with params: "
                    << params.parameters << std::endl
                    << "Error: " << dwGetStatusName(result) << std::endl;
        exit(1);
    }

    dwImageProperties cameraImageProperties;
    dwSensorCamera_getImageProperties(&cameraImageProperties,
                                     DW_CAMERA_PROCESSED_IMAGE,
                                    *camera);
    *imageWidth = cameraImageProperties.width;
    *imageHeight = cameraImageProperties.height;
    *cameraImageType = cameraImageProperties.type;

    dwCameraProperties cameraProperties;
    dwSensorCamera_getSensorProperties(&cameraProperties, *camera);

    std::cout << "Camera image with " << *imageWidth << "x" << *imageHeight
              << " at " << cameraProperties.framerate << " FPS" << std::endl;
}

void init(int& image_width, int& image_height, ProgramArguments arg) {
    initGL(&window);
    initSdk(&sdk, window);
    initRenderer(&renderer, sdk, window);
    initSensors(&sal, &cameraSensor, &image_width_, &image_height_, &cameraImageType, sdk, arg);
    if(cameraImageType != DW_IMAGE_NVMEDIA)
    {
        std::cerr << "Error: Expected nvmedia image type, received "
                  << cameraImageType << " instead." << std::endl;
        exit(-1);
    }

    // Allocate buffer for parsed embedded data
    sensorData.top.data    = new uint8_t[MAX_EMBED_DATA_SIZE];
    sensorData.bottom.data = new uint8_t[MAX_EMBED_DATA_SIZE];
    sensorData.top.bufferSize    = MAX_EMBED_DATA_SIZE;
    sensorData.bottom.bufferSize = MAX_EMBED_DATA_SIZE;

    image_width = image_width_;
    image_height = image_height_;
}

void start() {
    // RGBA image pool for conversion from YUV camera output
    NvMediaDevice *nvmedia;
    dwContext_getNvMediaDevice(&nvmedia, sdk);

    for (int i = 0; i < 1; ++i) {
        NvMediaImageAdvancedConfig advConfig;
        memset(&advConfig, 0, sizeof(advConfig));
        dwImageNvMedia *rgbaImage = new dwImageNvMedia();
        NvMediaImage *rgbaNvMediaImage;

        rgbaNvMediaImage = NvMediaImageCreate(nvmedia, NvMediaSurfaceType_Image_RGBA,
                                              NVMEDIA_IMAGE_CLASS_SINGLE_IMAGE, 1,
                                              image_width_, image_height_,
                                              0,
                                              &advConfig);
        dwImageNvMedia_setFromImage(rgbaImage, rgbaNvMediaImage);

        rgbaImagePool.push_back(rgbaImage);
    }

    // format converter
    dwSensorCamera_getImageProperties(&cameraImageProperties, DW_CAMERA_PROCESSED_IMAGE, cameraSensor);
    dwImageProperties displayImageProperties = cameraImageProperties;
    displayImageProperties.pxlFormat = DW_IMAGE_RGBA;

    dwStatus status                         = dwImageFormatConverter_initialize(&yuv2rgba,
                                                        cameraImageProperties,
                                                        displayImageProperties,
                                                        sdk);
    if (status != DW_SUCCESS) {
        std::cerr << "Cannot initialize pixel format converter" << std::endl;
        exit(1);
    }

    // image API translator
    dwImageStreamer_initialize(&nvm2cpu, DW_IMAGE_NVMEDIA, DW_IMAGE_CPU,
                               DW_IMAGE_RGBA, DW_TYPE_UINT8, image_width_, image_height_, sdk);

    // Start Sensor and Processing
    dwSensorCamera_getSensorProperties(&cameraProperties, cameraSensor);

    g_run = dwSensor_start(cameraSensor) == DW_SUCCESS;
}

bool read_frame(unsigned char** image_data) {
    if (image_data == nullptr) {
        return false;
    }

    dwCameraFrameHandle_t frameHandle;
    dwImageNvMedia *frame = nullptr;
    uint32_t camera = 0u;
    dwStatus status = dwSensorCamera_readFrame(&frameHandle, camera, 1000000, cameraSensor);
    if (status != DW_SUCCESS) {
        std::cout << "\n ERROR readFrame: " << dwGetStatusName(status) << std::endl;
        return false;
    }

    if( cameraProperties.outputTypes & DW_CAMERA_PROCESSED_IMAGE) {
        status = dwSensorCamera_getImageNvMedia(&frame, DW_CAMERA_PROCESSED_IMAGE, frameHandle);
        if( status != DW_SUCCESS ) {
            std::cout << "\n ERROR getImageNvMedia " << dwGetStatusName(status) << std::endl;
            return false;
        }
    }

    // get embedded lines
    if( cameraProperties.outputTypes & DW_CAMERA_DATALINES) {
        const dwCameraDataLines* dataLines = nullptr;
        status = dwSensorCamera_getDataLines(&dataLines, frameHandle);
        // parse the data
        if( status == DW_SUCCESS ) {
            status = dwSensorCamera_parseDataNvMedia(&sensorData, dataLines, cameraSensor);
            if( status == DW_SUCCESS ) {
                std::cout << "Exposure Time (s): " << sensorData.exposureMidpointTime << "\r";// std::endl;
            } else {
                std::cout << "Could not parse embedded data: " << dwGetStatusName(status) << "\r"; //std::endl;
                return false;
            }
        } else {
            std::cout << "Error getting datalines: " << dwGetStatusName(status) << "\r"; //std::endl;
            return false;
        }
    }

    // Convert from YUV to RGBA
    if (frame && rgbaImagePool.size() > 0) {
        std::cout << "rgbaImagePool size: " << rgbaImagePool.size() << std::endl;
        dwImageNvMedia *rgbaImage = rgbaImagePool.back();
        rgbaImagePool.pop_back();

        std::cout << " CONVERSION YUV->RGBA\n";
        status = dwImageFormatConverter_copyConvertNvMedia(rgbaImage, frame, yuv2rgba);
        if (status != DW_SUCCESS) {
            std::cout << "\n ERROR copyConvert: " << dwGetStatusName(status) << std::endl;
            rgbaImagePool.push_back(rgbaImage);
        } else {
            status = dwImageStreamer_postNvMedia(rgbaImage, nvm2cpu);
            if (status != DW_SUCCESS) {
                std::cout << "\n ERROR postNvMedia: " << dwGetStatusName(status) << std::endl;
                return false;
            }
            status = dwImageStreamer_receiveCPU(&frameCPU, 60000, nvm2cpu);
            if (status == DW_SUCCESS && frameCPU) {
                *image_data = frameCPU->data[0];
            } else {
                memset(image_data, 0, image_width_ * image_height_ * 4);
            }
            dwImageNvMedia *retimg = nullptr;
            dwImageStreamer_waitPostedNvMedia(&retimg, 33000, nvm2cpu);
            if (retimg)
                rgbaImagePool.push_back(retimg);
            else
                rgbaImagePool.push_back(rgbaImage);
        }
    }

    dwSensorCamera_returnFrame(&frameHandle);

    //if (window)
    //    window->swapBuffers();

    return true;
}

void reset_frame() {
    dwImageStreamer_returnReceivedCPU(frameCPU, nvm2cpu);
}

void stop() {
    dwSensor_stop(cameraSensor);
    dwImageStreamer_release(&nvm2gl);

    for (auto frame : rgbaImagePool) {
        NvMediaImageDestroy(frame->img);
        delete frame;
    }

    dwImageFormatConverter_release(&yuv2rgba);
}

void release() {
    // release used objects in correct order
    dwSAL_releaseSensor(&cameraSensor);
    dwSAL_release(&sal);
    dwRenderer_release(&renderer);
    dwRelease(&sdk);
    dwLogger_release();
    delete window;
    delete[] sensorData.top.data;
    delete[] sensorData.bottom.data;
}
