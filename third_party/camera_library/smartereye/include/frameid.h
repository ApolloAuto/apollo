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
#ifndef FRAMEIDDEF
#define FRAMEIDDEF

#ifndef FRAMEIDHELPER_LIBRARY
struct FrameId
{
#endif
    enum Enumeration {
        NotUsed              = 0,
        LeftCamera           = 1 << 0,
        RightCamera          = 1 << 1,
        CalibLeftCamera      = 1 << 2,
        CalibRightCamera     = 1 << 3,
        DisparityDSBak       = 1 << 4,
        DisparityUV          = 1 << 5,
        Disparity            = 1 << 6,
        DisparityPlus        = 1 << 7,
        DisparityDS          = 1 << 8,
        Lane                 = 1 << 9,
        Obstacle             = 1 << 10,
        Compound             = 1 << 11,
        LDownSample          = 1 << 12,
        RDownSample          = 1 << 13,
        LaneExt              = 1 << 14,
    };
#ifndef FRAMEIDHELPER_LIBRARY
};
#else
    Q_ENUM(Enumeration)
#endif

#endif // FRAMEDEF

