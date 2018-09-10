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

#ifndef QMEDIARECORDER_P_H
#define QMEDIARECORDER_P_H

#include "qmediarecorder.h"
#include "qmediaobject_p.h"
#include <QtCore/qurl.h>

QT_BEGIN_NAMESPACE

class QMediaRecorderControl;
class QMediaContainerControl;
class QAudioEncoderSettingsControl;
class QVideoEncoderSettingsControl;
class QMetaDataWriterControl;
class QMediaAvailabilityControl;
class QTimer;

class QMediaRecorderPrivate
{
    Q_DECLARE_NON_CONST_PUBLIC(QMediaRecorder)

public:
    QMediaRecorderPrivate();
    virtual ~QMediaRecorderPrivate() {}

    void applySettingsLater();
    void restartCamera();

    QMediaObject *mediaObject;

    QMediaRecorderControl *control;
    QMediaContainerControl *formatControl;
    QAudioEncoderSettingsControl *audioControl;
    QVideoEncoderSettingsControl *videoControl;
    QMetaDataWriterControl *metaDataControl;
    QMediaAvailabilityControl *availabilityControl;

    bool settingsChanged;

    QTimer* notifyTimer;

    QMediaRecorder::State state;
    QMediaRecorder::Error error;
    QString errorString;
    QUrl actualLocation;

    void _q_stateChanged(QMediaRecorder::State state);
    void _q_error(int error, const QString &errorString);
    void _q_serviceDestroyed();
    void _q_updateActualLocation(const QUrl &);
    void _q_notify();
    void _q_updateNotifyInterval(int ms);
    void _q_applySettings();
    void _q_availabilityChanged(QMultimedia::AvailabilityStatus availability);

    QMediaRecorder *q_ptr;
};

QT_END_NAMESPACE

#endif

