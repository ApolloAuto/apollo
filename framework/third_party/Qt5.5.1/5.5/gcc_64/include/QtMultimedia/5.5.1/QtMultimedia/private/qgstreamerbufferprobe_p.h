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

#ifndef QGSTREAMERBUFFERPROBE_H
#define QGSTREAMERBUFFERPROBE_H

#include <gst/gst.h>

#include <QtCore/qglobal.h>

QT_BEGIN_NAMESPACE

class QGstreamerBufferProbe
{
public:
    enum Flags
    {
        ProbeCaps       = 0x01,
        ProbeBuffers    = 0x02,
        ProbeAll    = ProbeCaps | ProbeBuffers
    };

    explicit QGstreamerBufferProbe(Flags flags = ProbeAll);
    virtual ~QGstreamerBufferProbe();

    void addProbeToPad(GstPad *pad, bool downstream = true);
    void removeProbeFromPad(GstPad *pad);

protected:
    virtual void probeCaps(GstCaps *caps);
    virtual bool probeBuffer(GstBuffer *buffer);

private:
#if GST_CHECK_VERSION(1,0,0)
    static GstPadProbeReturn capsProbe(GstPad *pad, GstPadProbeInfo *info, gpointer user_data);
    static GstPadProbeReturn bufferProbe(GstPad *pad, GstPadProbeInfo *info, gpointer user_data);
    int m_capsProbeId;
#else
    static gboolean bufferProbe(GstElement *element, GstBuffer *buffer, gpointer user_data);
    GstCaps *m_caps;
#endif
    int m_bufferProbeId;
    const Flags m_flags;
};

QT_END_NAMESPACE

#endif // QGSTREAMERAUDIOPROBECONTROL_H
