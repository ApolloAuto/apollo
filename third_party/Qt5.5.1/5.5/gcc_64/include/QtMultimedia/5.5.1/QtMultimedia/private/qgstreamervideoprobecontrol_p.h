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

#ifndef QGSTREAMERVIDEOPROBECONTROL_H
#define QGSTREAMERVIDEOPROBECONTROL_H

#include <gst/gst.h>
#include <gst/video/video.h>
#include <qmediavideoprobecontrol.h>
#include <QtCore/qmutex.h>
#include <qvideoframe.h>
#include <qvideosurfaceformat.h>

#include <private/qgstreamerbufferprobe_p.h>

QT_BEGIN_NAMESPACE

class QGstreamerVideoProbeControl
    : public QMediaVideoProbeControl
    , public QGstreamerBufferProbe
    , public QSharedData
{
    Q_OBJECT
public:
    explicit QGstreamerVideoProbeControl(QObject *parent);
    virtual ~QGstreamerVideoProbeControl();

    void probeCaps(GstCaps *caps);
    bool probeBuffer(GstBuffer *buffer);

    void startFlushing();
    void stopFlushing();

private slots:
    void frameProbed();

private:
    QVideoSurfaceFormat m_format;
    QVideoFrame m_pendingFrame;
    QMutex m_frameMutex;
#if GST_CHECK_VERSION(1,0,0)
    GstVideoInfo m_videoInfo;
#else
    int m_bytesPerLine;
#endif
    bool m_flushing;
    bool m_frameProbed; // true if at least one frame was probed
};

QT_END_NAMESPACE

#endif // QGSTREAMERVIDEOPROBECONTROL_H
