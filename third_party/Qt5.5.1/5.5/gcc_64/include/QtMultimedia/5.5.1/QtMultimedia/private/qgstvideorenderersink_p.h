/****************************************************************************
**
** Copyright (C) 2014 Jolla Ltd.
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

#ifndef QGSTVIDEORENDERERSINK_P_H
#define QGSTVIDEORENDERERSINK_P_H

//
//  W A R N I N G
//  -------------
//
// This file is not part of the Qt API. It exists purely as an
// implementation detail. This header file may change from version to
// version without notice, or even be removed.
//
// We mean it.
//

#include <gst/video/gstvideosink.h>
#include <gst/video/video.h>

#include <QtCore/qlist.h>
#include <QtCore/qmutex.h>
#include <QtCore/qqueue.h>
#include <QtCore/qpointer.h>
#include <QtCore/qwaitcondition.h>
#include <qvideosurfaceformat.h>
#include <qvideoframe.h>
#include <qabstractvideobuffer.h>

#include "qgstvideorendererplugin_p.h"

#include "qgstvideorendererplugin_p.h"

QT_BEGIN_NAMESPACE
class QAbstractVideoSurface;

class QGstDefaultVideoRenderer : public QGstVideoRenderer
{
public:
    QGstDefaultVideoRenderer();
    ~QGstDefaultVideoRenderer();

    GstCaps *getCaps(QAbstractVideoSurface *surface);
    bool start(QAbstractVideoSurface *surface, GstCaps *caps);
    void stop(QAbstractVideoSurface *surface);

    bool proposeAllocation(GstQuery *query);

    bool present(QAbstractVideoSurface *surface, GstBuffer *buffer);
    void flush(QAbstractVideoSurface *surface);

private:
    QVideoSurfaceFormat m_format;
    GstVideoInfo m_videoInfo;
    bool m_flushed;
};

class QVideoSurfaceGstDelegate : public QObject
{
    Q_OBJECT
public:
    QVideoSurfaceGstDelegate(QAbstractVideoSurface *surface);
    ~QVideoSurfaceGstDelegate();

    GstCaps *caps();

    bool start(GstCaps *caps);
    void stop();
    void unlock();
    bool proposeAllocation(GstQuery *query);

    void flush();

    GstFlowReturn render(GstBuffer *buffer);

    bool event(QEvent *event);

private slots:
    bool handleEvent(QMutexLocker *locker);
    void updateSupportedFormats();

private:
    void notify();
    bool waitForAsyncEvent(QMutexLocker *locker, QWaitCondition *condition, unsigned long time);

    QPointer<QAbstractVideoSurface> m_surface;

    QMutex m_mutex;
    QWaitCondition m_setupCondition;
    QWaitCondition m_renderCondition;
    GstFlowReturn m_renderReturn;
    QList<QGstVideoRenderer *> m_renderers;
    QGstVideoRenderer *m_renderer;
    QGstVideoRenderer *m_activeRenderer;

    GstCaps *m_surfaceCaps;
    GstCaps *m_startCaps;
    GstBuffer *m_renderBuffer;

    bool m_notified;
    bool m_stop;
    bool m_flush;
};

class QGstVideoRendererSink
{
public:
    GstVideoSink parent;

    static QGstVideoRendererSink *createSink(QAbstractVideoSurface *surface);

private:
    static GType get_type();
    static void class_init(gpointer g_class, gpointer class_data);
    static void base_init(gpointer g_class);
    static void instance_init(GTypeInstance *instance, gpointer g_class);

    static void finalize(GObject *object);

    static void handleShowPrerollChange(GObject *o, GParamSpec *p, gpointer d);

    static GstStateChangeReturn change_state(GstElement *element, GstStateChange transition);

    static GstCaps *get_caps(GstBaseSink *sink, GstCaps *filter);
    static gboolean set_caps(GstBaseSink *sink, GstCaps *caps);

    static gboolean propose_allocation(GstBaseSink *sink, GstQuery *query);

    static gboolean stop(GstBaseSink *sink);

    static gboolean unlock(GstBaseSink *sink);

    static GstFlowReturn show_frame(GstVideoSink *sink, GstBuffer *buffer);

private:
    QVideoSurfaceGstDelegate *delegate;
};


class QGstVideoRendererSinkClass
{
public:
    GstVideoSinkClass parent_class;
};

QT_END_NAMESPACE

#endif
