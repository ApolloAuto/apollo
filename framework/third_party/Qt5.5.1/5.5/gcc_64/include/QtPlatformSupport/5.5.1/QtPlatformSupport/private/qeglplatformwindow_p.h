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

#ifndef QEGLPLATFORMWINDOW_H
#define QEGLPLATFORMWINDOW_H

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

#include <qpa/qplatformwindow.h>
#include <QtPlatformSupport/private/qopenglcompositor_p.h>
#include <EGL/egl.h>

QT_BEGIN_NAMESPACE

class QOpenGLCompositorBackingStore;
class QPlatformTextureList;

class QEGLPlatformWindow : public QPlatformWindow, public QOpenGLCompositorWindow
{
public:
    QEGLPlatformWindow(QWindow *w);

    virtual void create();

    QOpenGLCompositorBackingStore *backingStore() { return m_backingStore; }
    void setBackingStore(QOpenGLCompositorBackingStore *backingStore) { m_backingStore = backingStore; }
    bool isRaster() const;

    QWindow *sourceWindow() const Q_DECL_OVERRIDE;
    const QPlatformTextureList *textures() const Q_DECL_OVERRIDE;
    void endCompositing() Q_DECL_OVERRIDE;

    WId winId() const Q_DECL_OVERRIDE;
    void setOpacity(qreal opacity) Q_DECL_OVERRIDE;

    virtual EGLNativeWindowType eglWindow() const = 0;

private:
    QOpenGLCompositorBackingStore *m_backingStore;
    bool m_raster;
    WId m_winId;
};

QT_END_NAMESPACE

#endif // QEGLPLATFORMWINDOW_H
