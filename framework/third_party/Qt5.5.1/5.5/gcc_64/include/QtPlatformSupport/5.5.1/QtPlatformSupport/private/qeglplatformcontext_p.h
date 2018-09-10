/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the plugins of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:LGPL21$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see http://www.qt.io/terms-conditions. For further
** information use the contact form at http://www.qt.io/contact-us.
**
** GNU Lesser General Public License Usage
** Alternatively, this file may be used under the terms of the GNU Lesser
** General Public License version 2.1 or version 3 as published by the Free
** Software Foundation and appearing in the file LICENSE.LGPLv21 and
** LICENSE.LGPLv3 included in the packaging of this file. Please review the
** following information to ensure the GNU Lesser General Public License
** requirements will be met: https://www.gnu.org/licenses/lgpl.html and
** http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html.
**
** As a special exception, The Qt Company gives you certain additional
** rights. These rights are described in The Qt Company LGPL Exception
** version 1.1, included in the file LGPL_EXCEPTION.txt in this package.
**
** $QT_END_LICENSE$
**
****************************************************************************/

#ifndef QEGLPLATFORMCONTEXT_H
#define QEGLPLATFORMCONTEXT_H

//
//  W A R N I N G
//  -------------
//
// This file is not part of the Qt API.  It exists purely as an
// implementation detail.  This header file may change from version to
// version without notice, or even be removed.
//
// We mean it.
//

#include <QtCore/qtextstream.h>
#include <qpa/qplatformwindow.h>
#include <qpa/qplatformopenglcontext.h>
#include <QtCore/QVariant>
#include <EGL/egl.h>

QT_BEGIN_NAMESPACE

class QEGLPlatformContext : public QPlatformOpenGLContext
{
public:
    QEGLPlatformContext(const QSurfaceFormat &format, QPlatformOpenGLContext *share, EGLDisplay display,
                        EGLConfig *config = 0, const QVariant &nativeHandle = QVariant());
    ~QEGLPlatformContext();

    void initialize() Q_DECL_OVERRIDE;
    bool makeCurrent(QPlatformSurface *surface) Q_DECL_OVERRIDE;
    void doneCurrent() Q_DECL_OVERRIDE;
    void swapBuffers(QPlatformSurface *surface) Q_DECL_OVERRIDE;
    QFunctionPointer getProcAddress(const QByteArray &procName) Q_DECL_OVERRIDE;

    QSurfaceFormat format() const Q_DECL_OVERRIDE;
    bool isSharing() const Q_DECL_OVERRIDE { return m_shareContext != EGL_NO_CONTEXT; }
    bool isValid() const Q_DECL_OVERRIDE { return m_eglContext != EGL_NO_CONTEXT; }

    EGLContext eglContext() const;
    EGLDisplay eglDisplay() const;
    EGLConfig eglConfig() const;

protected:
    virtual EGLSurface eglSurfaceForPlatformSurface(QPlatformSurface *surface) = 0;
    virtual EGLSurface createTemporaryOffscreenSurface();
    virtual void destroyTemporaryOffscreenSurface(EGLSurface surface);

private:
    void init(const QSurfaceFormat &format, QPlatformOpenGLContext *share);
    void adopt(const QVariant &nativeHandle, QPlatformOpenGLContext *share);
    void updateFormatFromGL();

    EGLContext m_eglContext;
    EGLContext m_shareContext;
    EGLDisplay m_eglDisplay;
    EGLConfig m_eglConfig;
    QSurfaceFormat m_format;
    EGLenum m_api;
    int m_swapInterval;
    bool m_swapIntervalEnvChecked;
    int m_swapIntervalFromEnv;
    bool m_ownsContext;
};

QT_END_NAMESPACE

#endif //QEGLPLATFORMCONTEXT_H
