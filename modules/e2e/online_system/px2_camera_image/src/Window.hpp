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

#ifndef SAMPLES_COMMON_WINDOW_HPP__
#define SAMPLES_COMMON_WINDOW_HPP__

//#include <dw/core/EGL.h>
#include <dw/gl/GL.h>

class WindowBase
{
  public:

    typedef void (*KeyPressCallback)(int key);
    typedef void (*MouseDownCallback)(int button, float x, float y);
    typedef void (*MouseUpCallback)(int button, float x, float y);
    typedef void (*MouseMoveCallback)(float x, float y);
    typedef void (*MouseWheelCallback)(float dx, float dy);
    typedef void (*ResizeWindowCallback)(int width, int height);


    // create an X11 window
    //   width: width of window
    //   height: height of window
    WindowBase(int windowWidth, int windowHeight)
        : m_width(windowWidth)
        , m_height(windowHeight)
        , m_keyPressCallback(nullptr)
        , m_mouseDownCallback(nullptr)
        , m_mouseUpCallback(nullptr)
        , m_mouseMoveCallback(nullptr)
        , m_mouseWheelCallback(nullptr)
        , m_resizeWindowCallback(nullptr)
    {
    }

    // release window
    virtual ~WindowBase()
    {
    }

    // swap back and front buffers
    virtual bool swapBuffers() = 0;

    // reset EGL context
    virtual void resetContext() = 0;

    // make window context current to the calling thread
    virtual bool makeCurrent() = 0;

    // remove current window context from the calling thread
    virtual bool resetCurrent() = 0;

    // indicate that a window should be closed, i.e. requested by the user
    virtual bool shouldClose() { return false; }

    // Set the window size
    virtual bool setWindowSize(int w, int h) { (void)w; (void)h; return false; }

    // get EGL display
    virtual EGLDisplay getEGLDisplay(void) = 0;
    virtual EGLContext getEGLContext(void) = 0;

    int width() const { return m_width; }
    int height() const { return m_height; }

    void setOnKeypressCallback(KeyPressCallback callback)
    {
        m_keyPressCallback = callback;
    }

    void setOnMouseDownCallback(MouseDownCallback callback)
    {
        m_mouseDownCallback = callback;
    }

    void setOnMouseUpCallback(MouseUpCallback callback)
    {
        m_mouseUpCallback = callback;
    }

    void setOnMouseMoveCallback(MouseMoveCallback callback)
    {
        m_mouseMoveCallback = callback;
    }

    void setOnMouseWheelCallback(MouseWheelCallback callback)
    {
        m_mouseWheelCallback = callback;
    }

    void setOnResizeWindowCallback(ResizeWindowCallback callback)
    {
        m_resizeWindowCallback = callback;
    }

  protected:
    int m_width;
    int m_height;

    KeyPressCallback m_keyPressCallback;
    MouseDownCallback m_mouseDownCallback;
    MouseUpCallback m_mouseUpCallback;
    MouseMoveCallback m_mouseMoveCallback;
    MouseWheelCallback m_mouseWheelCallback;
    ResizeWindowCallback m_resizeWindowCallback;
};

#endif // SAMPLES_COMMON_WINDOW_HPP__
