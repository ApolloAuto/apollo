/////////////////////////////////////////////////////////////////////////////////////////
// This code contains NVIDIA Confidential Information and is disclosed
// under the Mutual Non-Disclosure Agreement.
//
// Notice
// ALL NVIDIA DESIGN SPECIFICATIONS AND CODE ("MATERIALS") ARE PROVIDED "AS IS" NVIDIA MAKES
// NO REPRESENTATIONS, WARRANTIES, EXPRESSED, IMPLIED, STATUTORY, OR OTHERWISE WITH RESPECT TO
// THE MATERIALS, AND EXPRESSLY DISCLAIMS ANY IMPLIED WARRANTIES OF NONINFRINGEMENT,
// MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE.
//
// NVIDIA Corporation assumes no responsibility for the consequences of use of such
// information or for any infringement of patents or other rights of third parties that may
// result from its use. No license is granted by implication or otherwise under any patent
// or patent rights of NVIDIA Corporation. No third party distribution is allowed unless
// expressly authorized by NVIDIA.  Details are subject to change without notice.
// This code supersedes and replaces all information previously supplied.
// NVIDIA Corporation products are not authorized for use as critical
// components in life support devices or systems without express written approval of
// NVIDIA Corporation.
//
// Copyright (c) 2014-2016 NVIDIA Corporation. All rights reserved.
//
// NVIDIA Corporation and its licensors retain all intellectual property and proprietary
// rights in and to this software and related documentation and any modifications thereto.
// Any use, reproduction, disclosure or distribution of this software and related
// documentation without an express license agreement from NVIDIA Corporation is
// strictly prohibited.
//
/////////////////////////////////////////////////////////////////////////////////////////

#ifndef SAMPLES_COMMON_WINDOWEGL_HPP__
#define SAMPLES_COMMON_WINDOWEGL_HPP__

#include <EGL/egl.h>
#include <EGL/eglext.h>

#include "Window.hpp"
#include <memory>

/**
 * @brief The EGLDisplay class
 */
class WindowOffscreenEGL : public WindowBase
{
  public:
    WindowOffscreenEGL(int width, int height);
    virtual ~WindowOffscreenEGL();

    EGLDisplay getEGLDisplay() override
    {
        return m_display;
    }
    EGLContext getEGLContext() override
    {
        return m_context;
    }
    EGLConfig getEGLConfig() const
    {
        return m_config;
    }

    bool makeCurrent() override;
    bool resetCurrent() override;
    bool swapBuffers() override;
    void resetContext() override;

  protected:
    bool initDrm(const char *name, EGLAttrib *layerAttribs, int &dispWidth, int &dispHeight);

    EGLDisplay m_display;
    EGLContext m_context;
    EGLConfig m_config;
    EGLSurface m_surface;
    EGLStreamKHR m_stream;

    bool m_offscreen;
};

#endif // SAMPLES_COMMON_WINDOWEGL_HPP__
