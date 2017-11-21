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

#include "WindowEGL.hpp"

#include <iostream>
#include <iomanip>
#include <xf86drm.h>
#include <xf86drmMode.h>

static PFNEGLGETPLATFORMDISPLAYEXTPROC eglGetPlatformDisplayEXT = nullptr;
static PFNEGLQUERYDEVICESEXTPROC eglQueryDevicesEXT             = nullptr;
static PFNEGLQUERYDEVICESTRINGEXTPROC eglQueryDeviceStringEXT   = nullptr;
static PFNEGLGETOUTPUTLAYERSEXTPROC eglGetOutputLayersEXT       = nullptr;

static PFNEGLCREATESTREAMKHRPROC eglCreateStreamKHR                               = nullptr;
static PFNEGLDESTROYSTREAMKHRPROC eglDestroyStreamKHR                             = nullptr;
static PFNEGLCREATESTREAMPRODUCERSURFACEKHRPROC eglCreateStreamProducerSurfaceKHR = nullptr;
static PFNEGLSTREAMCONSUMEROUTPUTEXTPROC eglStreamConsumerOutputEXT               = nullptr;

// -----------------------------------------------------------------------------
WindowOffscreenEGL::WindowOffscreenEGL(int width, int height)
    : WindowBase(width, height)
    , m_display(EGL_NO_DISPLAY)
    , m_context(EGL_NO_CONTEXT)
    , m_surface(EGL_NO_SURFACE)
    , m_stream(EGL_NO_STREAM_KHR)
    , m_offscreen(true)
{
    eglGetPlatformDisplayEXT =
        (PFNEGLGETPLATFORMDISPLAYEXTPROC)eglGetProcAddress("eglGetPlatformDisplayEXT");
    eglQueryDevicesEXT      = (PFNEGLQUERYDEVICESEXTPROC)eglGetProcAddress("eglQueryDevicesEXT");
    eglQueryDeviceStringEXT = (PFNEGLQUERYDEVICESTRINGEXTPROC)eglGetProcAddress("eglQueryDeviceStringEXT");
    eglGetOutputLayersEXT   = (PFNEGLGETOUTPUTLAYERSEXTPROC)eglGetProcAddress("eglGetOutputLayersEXT");

    eglCreateStreamKHR  = (PFNEGLCREATESTREAMKHRPROC)eglGetProcAddress("eglCreateStreamKHR");
    eglDestroyStreamKHR = (PFNEGLDESTROYSTREAMKHRPROC)eglGetProcAddress("eglDestroyStreamKHR");

    eglCreateStreamProducerSurfaceKHR =
        (PFNEGLCREATESTREAMPRODUCERSURFACEKHRPROC)eglGetProcAddress("eglCreateStreamProducerSurfaceKHR");
    eglStreamConsumerOutputEXT =
        (PFNEGLSTREAMCONSUMEROUTPUTEXTPROC)eglGetProcAddress("eglStreamConsumerOutputEXT");

    if (!eglGetPlatformDisplayEXT || !eglQueryDevicesEXT || !eglQueryDeviceStringEXT) {
        std::cout << "WindowEGL: Cannot load EGL extensions: eglGetPlatformDisplayEXT,"
                  << " eglQueryDevicesEXT, eglQueryDeviceStringEXT" << std::endl;
        throw std::exception();
    }

    EGLBoolean status;

    EGLint deviceCount  = 4;
    EGLint foundDevices = 0;
    EGLDeviceEXT devices[deviceCount];

    // -----------------------
    std::cout << "WindowEGL: find EGL devices" << std::endl;
    {
        // Get pointers of required EGL functions
        status = eglQueryDevicesEXT(deviceCount, devices, &foundDevices);
        if (status != EGL_TRUE) {
            std::cout << "WindowEGL: Failed to query devices (error: "
                      << std::hex << eglGetError() << ")" << std::dec << std::endl;
            throw std::exception();
        }

        std::cout << "WindowEGL: found " << foundDevices << " EGL devices" << std::endl;
    }

    EGLAttrib layerAttribs[] = {
        EGL_NONE, EGL_NONE,
        EGL_NONE, EGL_NONE,
    };

    // -----------------------
    if (m_offscreen == false) {
        // get device name to setup DRM, assume that devices[0] is the one to use
        const char *drmName = eglQueryDeviceStringEXT(devices[0], EGL_DRM_DEVICE_FILE_EXT);
        if (!drmName) {
            std::cout << "WindowEGL: unable to query DRM device name" << std::endl;
        } else {
            std::cout << "WindowEGL: use drm device: " << drmName << std::endl;
        }

        std::cout << "WindowEGL: init DRM with " << width << "x" << height << " resolution" << std::endl;
        if (!initDrm(drmName, layerAttribs, m_width, m_height)) {
            std::cout << "WindowEGL: Failed to init DRM" << std::endl;
            throw std::exception();
        }
        std::cout << "WindowEGL: DRM initialized, selected resolution is "
                  << m_width
                  << ", "
                  << m_height
                  << std::endl;
    }

    // -----------------------
    std::cout << "WindowEGL: init EGL with GLES3 context" << std::endl;
    {
        EGLenum platform     = EGL_PLATFORM_DEVICE_EXT;
        EGLint dispAttribs[] = {EGL_NONE, EGL_NONE};

        // create display on top of DRM display
        m_display = eglGetPlatformDisplayEXT(platform, (NativeDisplayType)devices[0], dispAttribs);
        if (m_display == EGL_NO_DISPLAY) {
            std::cout << "WindowEGL: Failed to create EGL display: "
                      << "eglGetPlatformDisplayEXT failed with: "
                      << std::hex << eglGetError() << std::dec << std::endl;
            throw std::exception();
        }

        // setup EGL like in windowing system
        status = eglInitialize(m_display, 0, 0);
        if (!status) {
            eglTerminate(m_display);
            m_display = EGL_NO_DISPLAY;

            std::cout << "WindowEGL: Could not init EGL eglInitialize failed with: "
                      << std::hex << eglGetError() << std::dec << std::endl;
            throw std::exception();
        }

        std::cout << "WindowEGL: bind OpenGLES3 API" << std::endl;
        EGLint cfgAttribs[] = {
            EGL_SURFACE_TYPE, EGL_PBUFFER_BIT | EGL_STREAM_BIT_KHR,
            EGL_RENDERABLE_TYPE, EGL_OPENGL_ES3_BIT,
            EGL_RED_SIZE, 8,
            EGL_GREEN_SIZE, 8,
            EGL_BLUE_SIZE, 8,
            EGL_ALPHA_SIZE, 8,
            EGL_DEPTH_SIZE, 8,
            //EGL_STENCIL_SIZE, 8,
            EGL_NONE, EGL_NONE};
        EGLint cfgCount = 0;

        status = eglChooseConfig(m_display, cfgAttribs, &m_config, 1, &cfgCount);
        if (!status || cfgCount == 0) {
            std::cout << "Could not read EGL config count! error: 0x"
                      << std::hex << eglGetError() << std::dec << std::endl;
            throw std::exception();
        }

        // in offscreen mode we use Pbuffer surface to manage GL context
        if (m_offscreen == true) {
            std::cout << "WindowEGL: offscreen mode -> use EGL PBuffer surface "
                      << m_width << "x" << m_height << std::endl;
            EGLint pbsAttrib[] = {
                EGL_WIDTH, m_width,
                EGL_HEIGHT, m_height,
                //EGL_MIPMAP_TEXTURE, EGL_TRUE,
                //EGL_TEXTURE_FORMAT, EGL_TEXTURE_RGBA,
                //EGL_TEXTURE_TARGET, EGL_TEXTURE_2D,
                EGL_NONE, EGL_NONE};
            m_surface = eglCreatePbufferSurface(m_display, m_config, pbsAttrib);

            // in onscreen mode we pass GL output through EGL stream directly to display device
        } else {
            std::cout << "WindowEGL: onscreen mode -> use EGL stream to display" << std::endl;

            EGLOutputLayerEXT layer;
            int n;
            if (!eglGetOutputLayersEXT(m_display, layerAttribs, &layer, 1, &n) || !n) {
                std::cout << "WindowEGL: Unable to get output layer" << std::endl;
                throw std::exception();
            }

            // Create a stream and connect to the output
            EGLint stream_attr[] = {
                EGL_STREAM_FIFO_LENGTH_KHR, 1,
                EGL_NONE};
            m_stream = eglCreateStreamKHR(m_display, stream_attr);
            if (m_stream == EGL_NO_STREAM_KHR) {
                std::cout << "WindowEGL: Unable to create stream" << std::endl;
                throw std::exception();
            }

            if (!eglStreamConsumerOutputEXT(m_display, m_stream, layer)) {
                std::cout << "Unable to connect output to stream" << std::endl;
                return;
            }

            EGLint srfAttribs[] = {
                EGL_WIDTH, m_width,
                EGL_HEIGHT, m_height,
                EGL_NONE, EGL_NONE};

            // create a surface as EGL stream producer
            m_surface = eglCreateStreamProducerSurfaceKHR(m_display, m_config, m_stream, srfAttribs);
        }

        if (m_surface == EGL_NO_SURFACE) {
            std::cout << "WindowEGL: Could not create rendering surface: 0x"
                      << std::hex << eglGetError() << std::dec << std::endl;
            throw std::exception();
        }

        eglBindAPI(EGL_OPENGL_ES_API);
    }

    // -----------------------
    std::cout << "WindowEGL: create EGL context" << std::endl;
    {
        EGLint ctxAttribs[] = {
            EGL_CONTEXT_CLIENT_VERSION, 3,
            EGL_CONTEXT_OPENGL_ROBUST_ACCESS_EXT, EGL_FALSE,
            EGL_CONTEXT_OPENGL_RESET_NOTIFICATION_STRATEGY_EXT, EGL_NO_RESET_NOTIFICATION_EXT,
            EGL_NONE, EGL_NONE};
        m_context = eglCreateContext(m_display, m_config, EGL_NO_CONTEXT, ctxAttribs);
        if (m_context == EGL_NO_CONTEXT) {
            std::cout << "WindowEGL: Failed to create EGL context" << std::endl;
            throw std::exception();
        }
    }

    // -----------------------
    std::cout << "WindowEGL: assign EGL context to current thread" << std::endl;
    {
        status = eglMakeCurrent(m_display, m_surface, m_surface, m_context);
        if (status == EGL_FALSE) {
            std::cout << "WindowEGL: Could not makeCurrent: "
                      << std::hex << eglGetError() << std::dec << std::endl;
            throw std::exception();
        }
    }
}

// -----------------------------------------------------------------------------
WindowOffscreenEGL::~WindowOffscreenEGL()
{
    if (m_stream != EGL_NO_STREAM_KHR)
        eglDestroyStreamKHR(m_display, m_stream);
    if (m_surface != EGL_NO_SURFACE)
        eglDestroySurface(m_display, m_surface);

    eglDestroyContext(m_display, m_context);
    eglTerminate(m_display);
    eglReleaseThread();
}

// -----------------------------------------------------------------------------
void WindowOffscreenEGL::resetContext()
{
    eglDestroyContext(m_display, m_context);

    // -----------------------
    std::cout << "WindowEGL: create EGL context" << std::endl;
    {
        EGLint ctxAttribs[] = {
            EGL_CONTEXT_CLIENT_VERSION, 3,
            EGL_CONTEXT_OPENGL_ROBUST_ACCESS_EXT, EGL_FALSE,
            EGL_CONTEXT_OPENGL_RESET_NOTIFICATION_STRATEGY_EXT, EGL_NO_RESET_NOTIFICATION_EXT,
            EGL_NONE, EGL_NONE};
        m_context = eglCreateContext(m_display, m_config, EGL_NO_CONTEXT, ctxAttribs);
        if (m_context == EGL_NO_CONTEXT) {
            std::cout << "WindowEGL: Failed to create EGL context" << std::endl;
            throw std::exception();
        }
    }

    // -----------------------
    std::cout << "WindowEGL: assign EGL context to current thread" << std::endl;
    {
        EGLBoolean status = eglMakeCurrent(m_display, m_surface, m_surface, m_context);
        if (status == EGL_FALSE) {
            std::cout << "WindowEGL: Could not makeCurrent: "
                      << std::hex << eglGetError() << std::dec << std::endl;
            throw std::exception();
        }
    }
}

// -----------------------------------------------------------------------------
bool WindowOffscreenEGL::makeCurrent()
{
    bool ok = eglMakeCurrent(m_display, m_surface, m_surface, m_context) == EGL_TRUE;
    if (!ok)
    {
        std::cout << "eglMakeCurrent error: 0x" << std::hex << eglGetError() << std::endl;
    }
    return ok;
}

// -----------------------------------------------------------------------------
bool WindowOffscreenEGL::resetCurrent()
{
    bool ok = eglMakeCurrent(m_display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT) == EGL_TRUE;
    if (!ok)
    {
        std::cout << "eglMakeCurrent error: 0x" << std::hex << eglGetError() << std::endl;
    }
    return ok;
}


// -----------------------------------------------------------------------------
bool WindowOffscreenEGL::swapBuffers()
{
    return eglSwapBuffers(m_display, m_surface) == EGL_TRUE;
}

// -----------------------------------------------------------------------------
bool WindowOffscreenEGL::initDrm(const char *name, EGLAttrib *layerAttribs, int &dispWidth, int &dispHeight)
{
    // screen attached to which connector id?
    int conn = 0;
    int crtc = -1, plane = -1;
    int xSurfSize = 0, ySurfSize = 0;
    int xOffset = 0, yOffset = 0;
    // auto detect resolution by setting modesize to 0
    //int xModeSize = dispWidth, yModeSize = dispHeight;
    int xModeSize = 0, yModeSize = 0;
    int bounce = 0;

    int drmFd;
    uint32_t drmConnId, drmEncId, drmCrtcId, drmPlaneId;
    uint32_t crtcMask;
    drmModeRes *drmResInfo           = nullptr;
    drmModePlaneRes *drmPlaneResInfo = nullptr;
    drmModeCrtc *drmCrtcInfo         = nullptr;
    drmModeConnector *drmConnInfo    = nullptr;
    drmModeEncoder *drmEncInfo       = nullptr;
    drmModePlane *drmPlaneInfo       = nullptr;
    int drmModeIndex                 = 0;

    bool setMode = false;

    drmFd = drmOpen(name, nullptr);
    if (drmFd == -1) {
        std::cout << "Couldn't open device file " << name << std::endl;
        return false;
    }

    // Obtain DRM-KMS resources
    drmResInfo = drmModeGetResources(drmFd);
    if (!drmResInfo) {
        std::cout << "Couldn't obtain DRM-KMS resources" << std::endl;
        return false;
    }

    // If a specific crtc was requested, make sure it exists
    if (crtc >= drmResInfo->count_crtcs) {
        std::cout << "Requested crtc index (" << crtc
                  << ") exceeds count (" << drmResInfo->count_crtcs << ")" << std::endl;
        return false;
    }
    crtcMask = (crtc >= 0) ? (1 << crtc) : ((1 << drmResInfo->count_crtcs) - 1);

    // If drawing to a plane is requested, obtain the plane info
    if (plane >= 0) {
        drmPlaneResInfo = drmModeGetPlaneResources(drmFd);
        if (!drmPlaneResInfo) {
            std::cout << "Unable to obtain plane resource list" << std::endl;
            return false;
        }
        if (plane >= (int)drmPlaneResInfo->count_planes) {
            std::cout << "Requested plane index (" << plane
                      << ") exceeds count (" << drmPlaneResInfo->count_planes << ")" << std::endl;
            return false;
        }
        drmPlaneId   = drmPlaneResInfo->planes[plane];
        drmPlaneInfo = drmModeGetPlane(drmFd, drmPlaneId);
        if (!drmPlaneInfo) {
            std::cout << "Unable to obtain info for plane (" << drmPlaneId << ")" << std::endl;
            return false;
        }
        crtcMask &= drmPlaneInfo->possible_crtcs;
        if (!crtcMask) {
            std::cout << "Requested crtc and plane not compatible" << std::endl;
            return false;
        }
        std::cout << "Obtained plane information\n" << std::endl;
    }

    // Query info for requested connector
    if (conn >= drmResInfo->count_connectors) {
        std::cout << "Requested connector index (" << conn << ") exceeds count ("
                  << drmResInfo->count_connectors << ")" << std::endl;
        return false;
    }
    drmConnId   = drmResInfo->connectors[conn];
    drmConnInfo = drmModeGetConnector(drmFd, drmConnId);
    if (!drmConnInfo) {
        std::cout << "Unable to obtain info for connector (" << drmConnId << ")" << std::endl;
        return false;
    } else if (drmConnInfo->connection != DRM_MODE_CONNECTED) {
        std::cout << "Requested connnector is not connected (mode="
                  << drmConnInfo->connection << ")" << std::endl;
        return false;
    } else if (drmConnInfo->count_modes <= 0) {
        std::cout << "Requested connnector has no available modes" << std::endl;
        return false;
    }

    // If there is already an encoder attached to the connector, choose
    //   it unless not compatible with crtc/plane
    drmEncId   = drmConnInfo->encoder_id;
    drmEncInfo = drmModeGetEncoder(drmFd, drmEncId);
    if (drmEncInfo) {
        if (!(drmEncInfo->possible_crtcs & crtcMask)) {
            drmModeFreeEncoder(drmEncInfo);
            drmEncInfo = nullptr;
        }
    }

    // If we didn't have a suitable encoder, find one
    if (!drmEncInfo) {
        int i;
        for (i = 0; i < drmConnInfo->count_encoders; ++i) {
            drmEncId   = drmConnInfo->encoders[i];
            drmEncInfo = drmModeGetEncoder(drmFd, drmEncId);
            if (drmEncInfo) {
                if (crtcMask & drmEncInfo->possible_crtcs) {
                    crtcMask &= drmEncInfo->possible_crtcs;
                    break;
                }
                drmModeFreeEncoder(drmEncInfo);
                drmEncInfo = nullptr;
            }
        }
        if (i == drmConnInfo->count_encoders) {
            std::cout << "Unable to find suitable encoder" << std::endl;
            return false;
        }
    }

    // Select a suitable crtc. Give preference to any that is already
    // attached to the encoder.
    drmCrtcId = drmEncInfo->crtc_id;
    for (int i = 0; i < drmResInfo->count_crtcs; ++i) {
        if (crtcMask & (1 << i)) {
            drmCrtcId = drmResInfo->crtcs[i];
            if (drmResInfo->crtcs[i] == drmEncInfo->crtc_id) {
                break;
            }
        }
    }

    // Query info for crtc
    drmCrtcInfo = drmModeGetCrtc(drmFd, drmCrtcId);
    if (!drmCrtcInfo) {
        std::cout << "Unable to obtain info for crtc (" << drmCrtcId << ")" << std::endl;
        return false;
    }

    // If dimensions are specified and not using a plane, find closest mode
    if ((xModeSize || yModeSize) && (plane < 0)) {
        // Find best fit among available modes
        int best_index = 0;
        int best_fit = 0x7fffffff;
        for (int i = 0; i < drmConnInfo->count_modes; ++i) {
            drmModeModeInfoPtr mode = drmConnInfo->modes + i;
            int fit                 = 0;

            if (xModeSize) {
                fit += abs((int)mode->hdisplay - xModeSize) * (int)mode->vdisplay;
            }
            if (yModeSize) {
                fit += abs((int)mode->vdisplay - yModeSize) * (int)mode->hdisplay;
            }

            if (fit < best_fit) {
                best_index = i;
                best_fit   = fit;
            }
            std::cout << i << ": " << (int)mode->hdisplay << " x "
                      << (int)mode->vdisplay - yModeSize << std::endl;
        }

        // Choose this size/mode
        drmModeIndex = best_index;
        xModeSize    = (int)drmConnInfo->modes[best_index].hdisplay;
        yModeSize    = (int)drmConnInfo->modes[best_index].vdisplay;
    }

    // We'll only set the mode if we have to.
    if ((drmConnInfo->encoder_id != drmEncId) ||
        (drmEncInfo->crtc_id != drmCrtcId) ||
        !drmCrtcInfo->mode_valid ||
        ((plane < 0) && xModeSize && (xModeSize != (int)drmCrtcInfo->mode.hdisplay)) ||
        ((plane < 0) && yModeSize && (yModeSize != (int)drmCrtcInfo->mode.vdisplay))) {
        setMode = true;
    }

    // If dimensions haven't been specified, figure out good values to use
    if (!xModeSize || !yModeSize) {
        // If mode requires reset, just pick the first one available
        //   from the connector
        if (setMode) {
            xModeSize = (int)drmConnInfo->modes[0].hdisplay;
            yModeSize = (int)drmConnInfo->modes[0].vdisplay;
        }
        // Otherwise get it from the current crtc settings
        else {
            xModeSize = (int)drmCrtcInfo->mode.hdisplay;
            yModeSize = (int)drmCrtcInfo->mode.vdisplay;
        }
    }

    // If surf size is unspecified, default to fullscreen normally
    // or to 1/4 fullscreen if in animated bounce mode.
    if (!xSurfSize || !ySurfSize) {
        if (bounce) {
            xSurfSize = xModeSize / 2;
            ySurfSize = yModeSize / 2;
        } else {
            xSurfSize = xModeSize;
            ySurfSize = yModeSize;
        }
    }

    // If necessary, set the mode
    if (setMode) {
        drmModeSetCrtc(drmFd, drmCrtcId, -1, 0, 0, &drmConnId, 1, drmConnInfo->modes + drmModeIndex);
        std::cout << "Set mode" << std::endl;
    }

    // If plane is in use, set it
    if (plane >= 0) {
        drmModeSetPlane(drmFd, drmPlaneId, drmCrtcId, -1, 0,
                        xOffset, yOffset, xSurfSize, ySurfSize,
                        0, 0, xSurfSize << 16, ySurfSize << 16);
        std::cout << "Set plane configuration" << std::endl;
    }

    // Get the layer for this crtc/plane
    if (plane >= 0) {
        layerAttribs[0] = EGL_DRM_PLANE_EXT;
        layerAttribs[1] = (EGLAttrib)drmPlaneId;
    } else {
        layerAttribs[0] = EGL_DRM_CRTC_EXT;
        layerAttribs[1] = (EGLAttrib)drmCrtcId;
    }
    dispWidth  = xSurfSize;
    dispHeight = ySurfSize;

    return true;
}
