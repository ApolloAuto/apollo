/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Copyright (C) 2012 Research In Motion
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

#ifndef QDECLARATIVEVIDEOOUTPUT_RENDER_P_H
#define QDECLARATIVEVIDEOOUTPUT_RENDER_P_H

#include "qdeclarativevideooutput_backend_p.h"
#include "qsgvideonode_yuv.h"
#include "qsgvideonode_rgb.h"
#include "qsgvideonode_texture.h"

#include <QtCore/qmutex.h>
#include <QtMultimedia/qabstractvideosurface.h>

QT_BEGIN_NAMESPACE

class QSGVideoItemSurface;
class QVideoRendererControl;
class QOpenGLContext;
class QAbstractVideoFilter;
class QVideoFilterRunnable;

class QDeclarativeVideoRendererBackend : public QDeclarativeVideoBackend
{
public:
    QDeclarativeVideoRendererBackend(QDeclarativeVideoOutput *parent);
    ~QDeclarativeVideoRendererBackend();

    bool init(QMediaService *service) Q_DECL_OVERRIDE;
    void itemChange(QQuickItem::ItemChange change, const QQuickItem::ItemChangeData &changeData) Q_DECL_OVERRIDE;
    void releaseSource() Q_DECL_OVERRIDE;
    void releaseControl() Q_DECL_OVERRIDE;
    QSize nativeSize() const Q_DECL_OVERRIDE;
    void updateGeometry() Q_DECL_OVERRIDE;
    QSGNode *updatePaintNode(QSGNode *oldNode, QQuickItem::UpdatePaintNodeData *data) Q_DECL_OVERRIDE;
    QAbstractVideoSurface *videoSurface() const Q_DECL_OVERRIDE;
    QRectF adjustedViewport() const Q_DECL_OVERRIDE;
    QOpenGLContext *glContext() const;

    friend class QSGVideoItemSurface;
    void present(const QVideoFrame &frame);
    void stop();

    void appendFilter(QAbstractVideoFilter *filter) Q_DECL_OVERRIDE;
    void clearFilters() Q_DECL_OVERRIDE;
    void releaseResources() Q_DECL_OVERRIDE;
    void invalidateSceneGraph() Q_DECL_OVERRIDE;

private:
    void scheduleDeleteFilterResources();

    QPointer<QVideoRendererControl> m_rendererControl;
    QList<QSGVideoNodeFactoryInterface*> m_videoNodeFactories;
    QSGVideoItemSurface *m_surface;
    QOpenGLContext *m_glContext;
    QVideoFrame m_frame;
    bool m_frameChanged;
    QSGVideoNodeFactory_YUV m_i420Factory;
    QSGVideoNodeFactory_RGB m_rgbFactory;
    QSGVideoNodeFactory_Texture m_textureFactory;
    QMutex m_frameMutex;
    QRectF m_renderedRect;         // Destination pixel coordinates, clipped
    QRectF m_sourceTextureRect;    // Source texture coordinates

    struct Filter {
        Filter() : filter(0), runnable(0) { }
        Filter(QAbstractVideoFilter *filter) : filter(filter), runnable(0) { }
        QAbstractVideoFilter *filter;
        QVideoFilterRunnable *runnable;
    };
    QList<Filter> m_filters;
};

class QSGVideoItemSurface : public QAbstractVideoSurface
{
    Q_OBJECT
public:
    explicit QSGVideoItemSurface(QDeclarativeVideoRendererBackend *backend, QObject *parent = 0);
    ~QSGVideoItemSurface();
    QList<QVideoFrame::PixelFormat> supportedPixelFormats(QAbstractVideoBuffer::HandleType handleType) const;
    bool start(const QVideoSurfaceFormat &format);
    void stop();
    bool present(const QVideoFrame &frame);
    void scheduleOpenGLContextUpdate();

private slots:
    void updateOpenGLContext();

private:
    QDeclarativeVideoRendererBackend *m_backend;
};

QT_END_NAMESPACE

#endif
