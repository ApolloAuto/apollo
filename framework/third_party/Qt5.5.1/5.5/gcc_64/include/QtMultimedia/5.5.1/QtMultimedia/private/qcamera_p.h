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

#ifndef QCAMERA_P_H
#define QCAMERA_P_H

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

#include "qmediaobject_p.h"
#include "qvideosurfaceoutput_p.h"
#include "qcamera.h"

QT_BEGIN_NAMESPACE

class QMediaServiceProvider;
class QCameraControl;
class QVideoDeviceSelectorControl;
class QCameraLocksControl;
class QCameraInfoControl;
class QCameraViewfinderSettingsControl;
class QCameraViewfinderSettingsControl2;

class QCameraPrivate : public QMediaObjectPrivate
{
    Q_DECLARE_NON_CONST_PUBLIC(QCamera)
public:
    QCameraPrivate():
        QMediaObjectPrivate(),
        provider(0),
        control(0),
        deviceControl(0),
        locksControl(0),
        infoControl(0),
        viewfinderSettingsControl(0),
        viewfinderSettingsControl2(0),
        viewfinder(0),
        capture(0),
        state(QCamera::UnloadedState),
        error(QCamera::NoError),
        requestedLocks(QCamera::NoLock),
        lockStatus(QCamera::Unlocked),
        lockChangeReason(QCamera::UserRequest),
        supressLockChangedSignal(false),
        restartPending(false)
    {
    }

    void init();
    void initControls();

    void clear();

    QMediaServiceProvider *provider;

    QCameraControl *control;
    QVideoDeviceSelectorControl *deviceControl;
    QCameraLocksControl *locksControl;
    QCameraInfoControl *infoControl;
    QCameraViewfinderSettingsControl *viewfinderSettingsControl;
    QCameraViewfinderSettingsControl2 *viewfinderSettingsControl2;

    QCameraExposure *cameraExposure;
    QCameraFocus *cameraFocus;
    QCameraImageProcessing *imageProcessing;

    QObject *viewfinder;
    QObject *capture;

    QCamera::State state;

    QCamera::Error error;
    QString errorString;

    QCamera::LockTypes requestedLocks;

    QCamera::LockStatus lockStatus;
    QCamera::LockChangeReason lockChangeReason;
    bool supressLockChangedSignal;

    bool restartPending;

    QVideoSurfaceOutput surfaceViewfinder;

    void _q_error(int error, const QString &errorString);
    void unsetError() { error = QCamera::NoError; errorString.clear(); }

    void setState(QCamera::State);

    void _q_updateLockStatus(QCamera::LockType, QCamera::LockStatus, QCamera::LockChangeReason);
    void _q_updateState(QCamera::State newState);
    void _q_preparePropertyChange(int changeType);
    void _q_restartCamera();
    void updateLockStatus();
};

QT_END_NAMESPACE

#endif // QCAMERA_P_H
