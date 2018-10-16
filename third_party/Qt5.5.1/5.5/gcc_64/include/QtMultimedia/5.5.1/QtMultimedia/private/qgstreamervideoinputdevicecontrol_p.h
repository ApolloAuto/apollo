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

#ifndef QGSTREAMERVIDEOINPUTDEVICECONTROL_H
#define QGSTREAMERVIDEOINPUTDEVICECONTROL_H

#include <qvideodeviceselectorcontrol.h>
#include <QtCore/qstringlist.h>

#include <gst/gst.h>
#include <qcamera.h>

QT_BEGIN_NAMESPACE

class QGstreamerVideoInputDeviceControl : public QVideoDeviceSelectorControl
{
Q_OBJECT
public:
    QGstreamerVideoInputDeviceControl(QObject *parent);
    QGstreamerVideoInputDeviceControl(GstElementFactory *factory, QObject *parent);
    ~QGstreamerVideoInputDeviceControl();

    int deviceCount() const;

    QString deviceName(int index) const;
    QString deviceDescription(int index) const;

    int defaultDevice() const;
    int selectedDevice() const;

    static QString primaryCamera() { return tr("Main camera"); }
    static QString secondaryCamera() { return tr("Front camera"); }

public Q_SLOTS:
    void setSelectedDevice(int index);

private:
    GstElementFactory *m_factory;

    int m_selectedDevice;
};

QT_END_NAMESPACE

#endif // QGSTREAMERAUDIOINPUTDEVICECONTROL_H
