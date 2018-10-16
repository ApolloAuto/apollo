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

#ifndef QGSTVIDEOBUFFER_P_H
#define QGSTVIDEOBUFFER_P_H

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
#include <QtCore/qvariant.h>

#include <gst/gst.h>
#include <gst/video/video.h>

QT_BEGIN_NAMESPACE

#if GST_CHECK_VERSION(1,0,0)
class QGstVideoBuffer : public QAbstractPlanarVideoBuffer
{
public:
    QGstVideoBuffer(GstBuffer *buffer, const GstVideoInfo &info);
    QGstVideoBuffer(GstBuffer *buffer, const GstVideoInfo &info,
                    HandleType handleType, const QVariant &handle);
#else
class QGstVideoBuffer : public QAbstractVideoBuffer
{
public:
    QGstVideoBuffer(GstBuffer *buffer, int bytesPerLine);
    QGstVideoBuffer(GstBuffer *buffer, int bytesPerLine,
                    HandleType handleType, const QVariant &handle);
#endif

    ~QGstVideoBuffer();

    MapMode mapMode() const;

#if GST_CHECK_VERSION(1,0,0)
    int map(MapMode mode, int *numBytes, int bytesPerLine[4], uchar *data[4]);
#else
    uchar *map(MapMode mode, int *numBytes, int *bytesPerLine);
#endif

    void unmap();

    QVariant handle() const { return m_handle; }
private:
#if GST_CHECK_VERSION(1,0,0)
    GstVideoInfo m_videoInfo;
    GstVideoFrame m_frame;
#else
    int m_bytesPerLine;
#endif
    GstBuffer *m_buffer;
    MapMode m_mode;
    QVariant m_handle;
};

QT_END_NAMESPACE

#endif
