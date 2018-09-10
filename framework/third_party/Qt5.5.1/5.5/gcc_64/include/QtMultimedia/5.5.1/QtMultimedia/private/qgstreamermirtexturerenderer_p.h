/****************************************************************************
**
** Copyright (C) 2014 Canonical Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the Qt Toolkit.
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

#ifndef QGSTREAMERMIRTEXTURERENDERER_H
#define QGSTREAMERMIRTEXTURERENDERER_H

#include <qmediaplayer.h>
#include <qvideorenderercontrol.h>
#include <private/qvideosurfacegstsink_p.h>
#include <qabstractvideosurface.h>

#include "qgstreamervideorendererinterface_p.h"

QT_BEGIN_NAMESPACE

class QGstreamerMirTextureBuffer;
class QGstreamerPlayerSession;
class QGLContext;
class QOpenGLContext;
class QSurfaceFormat;

class QGstreamerMirTextureRenderer : public QVideoRendererControl, public QGstreamerVideoRendererInterface
{
    Q_OBJECT
    Q_INTERFACES(QGstreamerVideoRendererInterface)
public:
    QGstreamerMirTextureRenderer(QObject *parent = 0, const QGstreamerPlayerSession *playerSession = 0);
    virtual ~QGstreamerMirTextureRenderer();

    QAbstractVideoSurface *surface() const;
    void setSurface(QAbstractVideoSurface *surface);

    void setPlayerSession(const QGstreamerPlayerSession *playerSession);

    GstElement *videoSink();

    void stopRenderer();
    bool isReady() const { return m_surface != 0; }

signals:
    void sinkChanged();
    void readyChanged(bool);
    void nativeSizeChanged();

private slots:
    void handleFormatChange();
    void updateNativeVideoSize();
    void handleFocusWindowChanged(QWindow *window);
    void renderFrame();

private:
    QWindow *createOffscreenWindow(const QSurfaceFormat &format);
    static void handleFrameReady(gpointer userData);
    static GstPadProbeReturn padBufferProbe(GstPad *pad, GstPadProbeInfo *info, gpointer userData);

    GstElement *m_videoSink;
    QPointer<QAbstractVideoSurface> m_surface;
    QPointer<QAbstractVideoSurface> m_glSurface;
    QGLContext *m_context;
    QOpenGLContext *m_glContext;
    unsigned int m_textureId;
    QWindow *m_offscreenSurface;
    QGstreamerPlayerSession *m_playerSession;
    QGstreamerMirTextureBuffer *m_textureBuffer;
    QSize m_nativeSize;

    QMutex m_mutex;
};

QT_END_NAMESPACE

#endif // QGSTREAMERMIRTEXTURERENDRER_H
