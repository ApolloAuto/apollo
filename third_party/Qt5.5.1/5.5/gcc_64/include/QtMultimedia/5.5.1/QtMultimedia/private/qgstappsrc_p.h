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

#ifndef QGSTAPPSRC_H
#define QGSTAPPSRC_H

#include <QtCore/qobject.h>
#include <QtCore/qiodevice.h>

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>

#if GST_VERSION_MAJOR < 1
#include <gst/app/gstappbuffer.h>
#endif

QT_BEGIN_NAMESPACE

class QGstAppSrc  : public QObject
{
    Q_OBJECT
public:
    QGstAppSrc(QObject *parent = 0);
    ~QGstAppSrc();

    bool setup(GstElement *);
    bool isReady() const { return m_setup; }

    void setStream(QIODevice *);
    QIODevice *stream() const;

    GstAppSrc *element();

    qint64 queueSize() const { return m_maxBytes; }

    bool& enoughData() { return m_enoughData; }
    bool& dataRequested() { return m_dataRequested; }
    unsigned int& dataRequestSize() { return m_dataRequestSize; }

    bool isStreamValid() const
    {
        return m_stream != 0 &&
               m_stream->isOpen();
    }

private slots:
    void pushDataToAppSrc();
    bool doSeek(qint64);
    void onDataReady();

    void streamDestroyed();
private:
    static gboolean on_seek_data(GstAppSrc *element, guint64 arg0, gpointer userdata);
    static void on_enough_data(GstAppSrc *element, gpointer userdata);
    static void on_need_data(GstAppSrc *element, uint arg0, gpointer userdata);
    static void destroy_notify(gpointer data);

    void sendEOS();

    QIODevice *m_stream;
    GstAppSrc *m_appSrc;
    bool m_sequential;
    GstAppStreamType m_streamType;
    GstAppSrcCallbacks m_callbacks;
    qint64 m_maxBytes;
    bool m_setup;
    unsigned int m_dataRequestSize;
    bool m_dataRequested;
    bool m_enoughData;
    bool m_forceData;
};

QT_END_NAMESPACE

#endif
