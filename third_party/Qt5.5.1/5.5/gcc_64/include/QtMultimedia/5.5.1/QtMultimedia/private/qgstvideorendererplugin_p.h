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

#ifndef QGSTVIDEORENDERERPLUGIN_P_H
#define QGSTVIDEORENDERERPLUGIN_P_H

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

#include <qabstractvideobuffer.h>
#include <qvideosurfaceformat.h>
#include <QtCore/qobject.h>
#include <QtCore/qplugin.h>

#include <gst/gst.h>

QT_BEGIN_NAMESPACE

class QAbstractVideoSurface;

const QLatin1String QGstVideoRendererPluginKey("gstvideorenderer");

class QGstVideoRenderer
{
public:
    virtual ~QGstVideoRenderer() {}

    virtual GstCaps *getCaps(QAbstractVideoSurface *surface) = 0;
    virtual bool start(QAbstractVideoSurface *surface, GstCaps *caps) = 0;
    virtual void stop(QAbstractVideoSurface *surface) = 0;  // surface may be null if unexpectedly deleted.
    virtual bool proposeAllocation(GstQuery *query) = 0;    // may be called from a thread.

    virtual bool present(QAbstractVideoSurface *surface, GstBuffer *buffer) = 0;
    virtual void flush(QAbstractVideoSurface *surface) = 0; // surface may be null if unexpectedly deleted.
};

/*
    Abstract interface for video buffers allocation.
*/
class QGstVideoRendererInterface
{
public:
    virtual ~QGstVideoRendererInterface() {}

    virtual QGstVideoRenderer *createRenderer() = 0;
};

#define QGstVideoRendererInterface_iid "org.qt-project.qt.gstvideorenderer/5.4"
Q_DECLARE_INTERFACE(QGstVideoRendererInterface, QGstVideoRendererInterface_iid)

class QGstVideoRendererPlugin : public QObject, public QGstVideoRendererInterface
{
    Q_OBJECT
    Q_INTERFACES(QGstVideoRendererInterface)
public:
    explicit QGstVideoRendererPlugin(QObject *parent = 0);
    virtual ~QGstVideoRendererPlugin() {}

    virtual QGstVideoRenderer *createRenderer() = 0;

};

QT_END_NAMESPACE

#endif
