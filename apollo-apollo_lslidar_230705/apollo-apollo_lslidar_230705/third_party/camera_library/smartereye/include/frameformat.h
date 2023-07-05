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
#ifndef FRAMEFORMATDEF
#define FRAMEFORMATDEF

struct FrameFormat
{
    enum Enumeration {
        Gray = 0,
        Color = 1,
        YUV422 = 2,
        RGB565 = 3,
        YUV422Plannar = 4,
        Custom = 65,
        Disparity7 = 512,
        Disparity8,
        Disparity10,
        Disparity12,
        DisparitySparse,
        Disparity16,
        DisparityDens16,
        DefaultFormat  = Gray,
    };
    static int getDisparityBitNum(const unsigned short format)
    {
        int disparityBitNum;
        switch(format) {
        case Disparity7:
            disparityBitNum = 0;
            break;
        case Disparity8:
            disparityBitNum = 1;
            break;
        case Disparity12:
            disparityBitNum = 4;
            break;
        case Disparity16:
        case DisparityDens16:
            disparityBitNum = 5;
            break;
        case DisparitySparse:
            disparityBitNum = 16;
            break;
        default:
            disparityBitNum = -1;
        }
        return disparityBitNum;
    }
    static int getBitWidth(const unsigned short format)
    {
        int bitwidth;
        switch(format) {
        case Gray:
        case Disparity7:
        case Disparity8:
            bitwidth = 8;
            break;
        case Disparity12:
            bitwidth = 12;
            break;
        case Disparity16:
        case DisparitySparse:
        case YUV422:
        case YUV422Plannar:
        case RGB565:
        case DisparityDens16:
            bitwidth = 16;
            break;
        case Color:
            bitwidth = 24;
            break;
        default:
            bitwidth = -1;
        }
        return bitwidth;
    }

};

#endif // FRAMEDEF

