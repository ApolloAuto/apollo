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

#ifndef QCAMERAEXPOSURE_H
#define QCAMERAEXPOSURE_H

#include <QtMultimedia/qmediaobject.h>
#include <QtMultimedia/qmediaenumdebug.h>

QT_BEGIN_NAMESPACE


class QCamera;
class QCameraExposurePrivate;

class Q_MULTIMEDIA_EXPORT QCameraExposure : public QObject
{
    Q_OBJECT
    Q_PROPERTY(qreal aperture READ aperture NOTIFY apertureChanged)
    Q_PROPERTY(qreal shutterSpeed READ shutterSpeed NOTIFY shutterSpeedChanged)
    Q_PROPERTY(int isoSensitivity READ isoSensitivity NOTIFY isoSensitivityChanged)
    Q_PROPERTY(qreal exposureCompensation READ exposureCompensation WRITE setExposureCompensation NOTIFY exposureCompensationChanged)
    Q_PROPERTY(bool flashReady READ isFlashReady NOTIFY flashReady)
    Q_PROPERTY(QCameraExposure::FlashModes flashMode READ flashMode WRITE setFlashMode)
    Q_PROPERTY(QCameraExposure::ExposureMode exposureMode READ exposureMode WRITE setExposureMode)
    Q_PROPERTY(QCameraExposure::MeteringMode meteringMode READ meteringMode WRITE setMeteringMode)

    Q_ENUMS(FlashMode)
    Q_ENUMS(ExposureMode)
    Q_ENUMS(MeteringMode)
public:
    enum FlashMode {
        FlashAuto = 0x1,
        FlashOff = 0x2,
        FlashOn = 0x4,
        FlashRedEyeReduction  = 0x8,
        FlashFill = 0x10,
        FlashTorch = 0x20,
        FlashVideoLight = 0x40,
        FlashSlowSyncFrontCurtain = 0x80,
        FlashSlowSyncRearCurtain = 0x100,
        FlashManual = 0x200
    };
    Q_DECLARE_FLAGS(FlashModes, FlashMode)

    enum ExposureMode {
        ExposureAuto = 0,
        ExposureManual = 1,
        ExposurePortrait = 2,
        ExposureNight = 3,
        ExposureBacklight = 4,
        ExposureSpotlight = 5,
        ExposureSports = 6,
        ExposureSnow = 7,
        ExposureBeach = 8,
        ExposureLargeAperture = 9,
        ExposureSmallAperture = 10,
        ExposureAction = 11,
        ExposureLandscape = 12,
        ExposureNightPortrait = 13,
        ExposureTheatre = 14,
        ExposureSunset = 15,
        ExposureSteadyPhoto = 16,
        ExposureFireworks = 17,
        ExposureParty = 18,
        ExposureCandlelight = 19,
        ExposureBarcode = 20,
        ExposureModeVendor = 1000
    };

    enum MeteringMode {
        MeteringMatrix = 1,
        MeteringAverage = 2,
        MeteringSpot = 3
    };

    bool isAvailable() const;

    FlashModes flashMode() const;
    bool isFlashModeSupported(FlashModes mode) const;
    bool isFlashReady() const;

    ExposureMode exposureMode() const;
    bool isExposureModeSupported(ExposureMode mode) const;

    qreal exposureCompensation() const;

    MeteringMode meteringMode() const;
    bool isMeteringModeSupported(MeteringMode mode) const;

    QPointF spotMeteringPoint() const;
    void setSpotMeteringPoint(const QPointF &point);

    int isoSensitivity() const;
    qreal aperture() const;
    qreal shutterSpeed() const;

    int requestedIsoSensitivity() const;
    qreal requestedAperture() const;
    qreal requestedShutterSpeed() const;

    QList<int> supportedIsoSensitivities(bool *continuous = 0) const;
    QList<qreal> supportedApertures(bool * continuous = 0) const;
    QList<qreal> supportedShutterSpeeds(bool *continuous = 0) const;

public Q_SLOTS:
    void setFlashMode(FlashModes mode);
    void setExposureMode(ExposureMode mode);
    void setMeteringMode(MeteringMode mode);

    void setExposureCompensation(qreal ev);

    void setManualIsoSensitivity(int iso);
    void setAutoIsoSensitivity();

    void setManualAperture(qreal aperture);
    void setAutoAperture();

    void setManualShutterSpeed(qreal seconds);
    void setAutoShutterSpeed();

Q_SIGNALS:
    void flashReady(bool);

    void apertureChanged(qreal);
    void apertureRangeChanged();
    void shutterSpeedChanged(qreal);
    void shutterSpeedRangeChanged();
    void isoSensitivityChanged(int);
    void exposureCompensationChanged(qreal);

private:
    friend class QCamera;
    friend class QCameraPrivate;
    explicit QCameraExposure(QCamera *parent = 0);
    virtual ~QCameraExposure();

    Q_DISABLE_COPY(QCameraExposure)
    Q_DECLARE_PRIVATE(QCameraExposure)
    Q_PRIVATE_SLOT(d_func(), void _q_exposureParameterChanged(int))
    Q_PRIVATE_SLOT(d_func(), void _q_exposureParameterRangeChanged(int))
    QCameraExposurePrivate *d_ptr;
};

Q_DECLARE_OPERATORS_FOR_FLAGS(QCameraExposure::FlashModes)

QT_END_NAMESPACE

Q_DECLARE_METATYPE(QCameraExposure::ExposureMode)
Q_DECLARE_METATYPE(QCameraExposure::FlashModes)
Q_DECLARE_METATYPE(QCameraExposure::MeteringMode)

Q_MEDIA_ENUM_DEBUG(QCameraExposure, ExposureMode)
Q_MEDIA_ENUM_DEBUG(QCameraExposure, FlashMode)
Q_MEDIA_ENUM_DEBUG(QCameraExposure, MeteringMode)

#endif // QCAMERAEXPOSURE_H
