/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
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

#ifndef QGSTREAMERGLTEXTURERENDERER_H
#define QGSTREAMERGLTEXTURERENDERER_H

#include <qvideorenderercontrol.h>
#include <private/qvideosurfacegstsink_p.h>
#include <private/qgstreamerbushelper_p.h>

#include "qgstreamervideorendererinterface_p.h"
#include <QtGui/qcolor.h>

#include <X11/extensions/Xv.h>

QT_BEGIN_NAMESPACE

class QGLContext;

class QGstreamerGLTextureRenderer : public QVideoRendererControl,
        public QGstreamerVideoRendererInterface,
        public QGstreamerSyncMessageFilter,
        public QGstreamerBusMessageFilter
{
    Q_OBJECT
    Q_INTERFACES(QGstreamerVideoRendererInterface QGstreamerSyncMessageFilter QGstreamerBusMessageFilter)

    Q_PROPERTY(bool overlayEnabled READ overlayEnabled WRITE setOverlayEnabled)
    Q_PROPERTY(qulonglong winId READ winId WRITE setWinId)
    Q_PROPERTY(QRect overlayGeometry READ overlayGeometry WRITE setOverlayGeometry)
    Q_PROPERTY(QColor colorKey READ colorKey)
    Q_PROPERTY(QSize nativeSize READ nativeSize NOTIFY nativeSizeChanged)

public:
    QGstreamerGLTextureRenderer(QObject *parent = 0);
    virtual ~QGstreamerGLTextureRenderer();

    QAbstractVideoSurface *surface() const;
    void setSurface(QAbstractVideoSurface *surface);

    GstElement *videoSink();

    bool isReady() const;
    bool processBusMessage(const QGstreamerMessage &message);
    bool processSyncMessage(const QGstreamerMessage &message);
    void stopRenderer();

    int framebufferNumber() const;

    bool overlayEnabled() const;
    WId winId() const;
    QRect overlayGeometry() const;
    QColor colorKey() const;
    QSize nativeSize() const;

public slots:
    void renderGLFrame(int);

    void setOverlayEnabled(bool);
    void setWinId(WId id);
    void setOverlayGeometry(const QRect &geometry);
    void repaintOverlay();

signals:
    void sinkChanged();
    void readyChanged(bool);
    void nativeSizeChanged();

private slots:
    void handleFormatChange();
    void updateNativeVideoSize();

private:
    static void handleFrameReady(GstElement *sink, gint frame, gpointer data);
    static gboolean padBufferProbe(GstPad *pad, GstBuffer *buffer, gpointer user_data);

    GstElement *m_videoSink;
    QAbstractVideoSurface *m_surface;
    QGLContext *m_context;
    QSize m_nativeSize;

    WId m_winId;
    QColor m_colorKey;
    QRect m_displayRect;
    bool m_overlayEnabled;
    int m_bufferProbeId;

    QMutex m_mutex;
    QWaitCondition m_renderCondition;
};

QT_END_NAMESPACE

#endif // QGSTREAMERVIDEORENDRER_H
