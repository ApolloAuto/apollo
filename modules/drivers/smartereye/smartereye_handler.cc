/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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
#include "smartereye_handler.h"

#include <math.h>

#include <iostream>
#include <string>

#include "third_party/camera_library/smartereye/include/LdwDataInterface.h"
#include "third_party/camera_library/smartereye/include/disparityconvertor.h"
#include "third_party/camera_library/smartereye/include/frameid.h"
#include "third_party/camera_library/smartereye/include/obstacleData.h"
#include "third_party/camera_library/smartereye/include/satpext.h"

namespace apollo {
namespace drivers {
namespace smartereye {

static const std::string kHomeDir = getenv("HOME") ? getenv("HOME") : "/var";
static const std::string k3dPointByPointFilePath =
    kHomeDir + "/3d_pointbypoint.txt";
static const std::string k3dByLookUpTableFilePath =
    kHomeDir + "/3d_by_lookuptable.txt";
static const std::string kRotationMartrixFilePath =
    kHomeDir + "/rotation_matrix.txt";

static const int kDisparityCount = 81;

SmartereyeHandler::SmartereyeHandler(std::string name)
    : mName(name), mIsCalibParamReady(false) {
  pCallbackFunc = nullptr;
  mIsLookupTableGenerated = false;
}

SmartereyeHandler::~SmartereyeHandler() {
  pCallbackFunc = nullptr;
  mIsLookupTableGenerated = false;
}

void SmartereyeHandler::setStereoCalibParams(
    StereoCalibrationParameters &params) {
  mStereoCalibrationParameters = params;
  mIsCalibParamReady = true;
  printf("************ Camera params ************\n");
  printf("Focus: %e pixel\n", params.focus);
  printf("Optical center X: %e pixel\n", params.cx);
  printf("Optical center Y: %e pixel\n", params.cy);
  printf("R-vector roll: %e rad\n", params.RRoll);
  printf("R-vector pitch: %e rad\n", params.RPitch);
  printf("R-vector yaw: %e rad\n", params.RYaw);
  printf("Translation x: %e mm\n", params.Tx);
  printf("Translation y: %e mm\n", params.Ty);
  printf("Translation z: %e mm\n", params.Tz);
  printf("**************************************\n");
}

void SmartereyeHandler::setRotationMatrix(RotationMatrix &rotationMatrix) {
  mRotationMatrix = rotationMatrix;
  FILE *fp = nullptr;
  fp = fopen(kRotationMartrixFilePath.data(), "wb+");
  if (!fp) {
    std::cout << kRotationMartrixFilePath << " file not open" << std::endl;
    return;
  }

  fprintf(fp, "Real3DToImage:\n");
  for (int row = 0; row < 3; row++) {
    for (int col = 0; col < 4; col++) {
      fprintf(fp, "%e\t", mRotationMatrix.real3DToImage[col + 4 * row]);
    }
    fprintf(fp, "\n");
  }

  fprintf(fp, "ImageToReal3D:\n");
  for (int row = 0; row < 3; row++) {
    for (int col = 0; col < 3; col++) {
      fprintf(fp, "%e\t", mRotationMatrix.imageToReal3D[col + 3 * row]);
    }
    fprintf(fp, "\n");
  }

  fclose(fp);
}

bool SmartereyeHandler::SetCallback(CallbackFunc ptr) {
  pCallbackFunc = ptr;

  return true;
}

void SmartereyeHandler::handleRawFrame(const RawImageFrame *rawFrame) {
  pCallbackFunc(const_cast<RawImageFrame *>(rawFrame));
}

void SmartereyeHandler::processFrame(int frameId, char *image,
                                     uint32_t dataSize, int width, int height,
                                     int frameFormat) {
  switch (frameId) {
    case FrameId::Lane: {
      std::cout << "case FrameId::Lane:" << std::endl;
      LdwDataPack *ldwDataPack = (LdwDataPack *)image;
      std::cout << "ldw degree is: "
                << ldwDataPack->roadway.left_Lane.left_Boundary.degree
                << std::endl;
    } break;
    case FrameId::Obstacle: {
      std::cout << "case FrameId::Obstacle:" << std::endl;
      int blockNum = ((int *)image)[0];
      std::cout << "blockNum is: " << blockNum << std::endl;
    } break;
    case FrameId::Disparity: {
      std::cout << "case FrameId::Disparity:" << std::endl;
      int bitNum = DisparityConvertor::getDisparityBitNum(frameFormat);
      if (mIsCalibParamReady) {
        handleDisparityByLookupTable((unsigned char *)image, width, height,
                                     bitNum);
      }
    } break;
    case FrameId::CalibLeftCamera: {
      std::cout << "case FrameId::CalibLeftCamera:" << std::endl;
    } break;
    default:
      break;
  }
}

void SmartereyeHandler::handleDisparityPointByPoint(unsigned char *image,
                                                    int width, int height,
                                                    int bitNum) {
  std::cout << "width: " << width << ", height: " << height << std::endl;

  static float *floatData = new float[width * height];
  DisparityConvertor::convertDisparity2FloatFormat(image, width, height, bitNum,
                                                   floatData);
  static int index = 0;
  index++;
  FILE *fp = nullptr;
  if (index == 1) {
    fp = fopen(k3dPointByPointFilePath.data(), "wb+");
    if (!fp) {
      std::cout << k3dPointByPointFilePath << " file not open" << std::endl;
      return;
    }
  }

  for (int posY = 0; posY < height; posY++) {
    for (int posX = 0; posX < width; posX++) {
      float x, y, z;
      DisparityConvertor::getPointXYZDistance(
          image, width, height, bitNum, mStereoCalibrationParameters.Tx,
          mStereoCalibrationParameters.focus, mStereoCalibrationParameters.cx,
          mStereoCalibrationParameters.cy, posX, posY, x, y, z);

      if ((fabs(x) > 200000.0f) || (fabs(y) > 200000.0f) ||
          (fabs(z) > 200000.0f)) {
        x = 0.0f;
        y = 0.0f;
        z = 0.0f;
      }
      if (index == 1) {
        fprintf(fp, "%f %f %f\n", x, y, z);
      }
    }
  }
  if (index == 1) {
    fclose(fp);
  }
}

void SmartereyeHandler::handleDisparityByLookupTable(unsigned char *image,
                                                     int width, int height,
                                                     int bitNum) {
  static float *lookUpTableX =
      new float[kDisparityCount * (int)pow(2, bitNum) * width];
  static float *lookUpTableY =
      new float[kDisparityCount * (int)pow(2, bitNum) * height];
  static float *lookUpTableZ = new float[kDisparityCount * (int)pow(2, bitNum)];
  if (!mIsLookupTableGenerated) {
    DisparityConvertor::generateLookUpTableX(
        width, bitNum, mStereoCalibrationParameters.Tx,
        mStereoCalibrationParameters.cx, lookUpTableX);
    DisparityConvertor::generateLookUpTableY(
        height, bitNum, mStereoCalibrationParameters.Tx,
        mStereoCalibrationParameters.cy, lookUpTableY);
    DisparityConvertor::generateLookUpTableZ(
        bitNum, mStereoCalibrationParameters.Tx,
        mStereoCalibrationParameters.focus, lookUpTableZ);
    mIsLookupTableGenerated = true;
  }

  float *xDistance = new float[width * height];
  DisparityConvertor::getWholeXDistanceByLookupTable(
      image, width, height, bitNum, lookUpTableX, xDistance);
  float *yDistance = new float[width * height];
  DisparityConvertor::getWholeYDistanceByLookupTable(
      image, width, height, bitNum, lookUpTableY, yDistance);
  float *zDistance = new float[width * height];
  DisparityConvertor::getWholeZDistanceByLookupTable(image, width, height,
                                                     lookUpTableZ, zDistance);

  static int index = 0;
  index++;
  FILE *fp = nullptr;
  if (index == 1) {
    fp = fopen(k3dByLookUpTableFilePath.data(), "wb+");
    if (!fp) {
      std::cout << k3dByLookUpTableFilePath << " file not open" << std::endl;
      return;
    }
  }
  for (int i = 0; i < width * height; i++) {
    float x, y, z;
    x = xDistance[i];
    y = yDistance[i];
    z = zDistance[i];
    if ((fabs(x) > 200000.0f) || (fabs(y) > 200000.0f) ||
        (fabs(z) > 200000.0f)) {
      x = 0.0f;
      y = 0.0f;
      z = 0.0f;
    }
    if (index == 1) {
      fprintf(fp, "%f %f %f %d\n", x, y, z, i);
    }
  }
  if (index == 1) {
    fclose(fp);
  }

  delete[] xDistance;
  delete[] yDistance;
  delete[] zDistance;
}

void SmartereyeHandler::handleUpdateFinished(Result result) {
  mUpgradeResult = result;
  if (!mUpgradeResult.successed) {
    std::cout << "Update failed with warning:" << mUpgradeResult.warning
              << std::endl;
    std::cout
        << "If you want to update firmware, please press 'u' to try again !!! "
        << std::endl;
  }
}
}  // namespace smartereye
}  // namespace drivers
}  // namespace apollo
