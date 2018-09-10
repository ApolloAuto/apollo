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

#ifndef QCAMERAEXPOSURECONTROL_H
#define QCAMERAEXPOSURECONTROL_H

#include <QtMultimedia/qmediacontrol.h>
#include <QtMultimedia/qmediaobject.h>

#include <QtMultimedia/qcameraexposure.h>
#include <QtMultimedia/qcamera.h>
#include <QtMultimedia/qmediaenumdebug.h>

QT_BEGIN_NAMESPACE

// Required for QDoc workaround
class QString;

class Q_MULTIMEDIA_EXPORT QCameraExposureControl : public QMediaControl
{
    Q_OBJECT
    Q_ENUMS(ExposureParameter)

public:
    ~QCameraExposureControl();

    enum ExposureParameter {
        ISO,
        Aperture,
        ShutterSpeed,
        ExposureCompensation,
        FlashPower,
        FlashCompensation,
        TorchPower,
        SpotMeteringPoint,
        ExposureMode,
        MeteringMode,
        ExtendedExposureParameter = 1000
    };

    virtual bool isParameterSupported(ExposureParameter parameter) const = 0;
    virtual QVariantList supportedParameterRange(ExposureParameter parameter, bool *continuous) const = 0;

    virtual QVariant requestedValue(ExposureParameter parameter) const = 0;
    virtual QVariant actualValue(ExposureParameter parameter) const = 0;
    virtual bool setValue(ExposureParameter parameter, const QVariant& value) = 0;

Q_SIGNALS:
    void requestedValueChanged(int parameter);
    void actualValueChanged(int parameter);
    void parameterRangeChanged(int parameter);

protected:
    QCameraExposureControl(QObject* parent = 0);
};

#define QCameraExposureControl_iid "org.qt-project.qt.cameraexposurecontrol/5.0"
Q_MEDIA_DECLARE_CONTROL(QCameraExposureControl, QCameraExposureControl_iid)

QT_END_NAMESPACE

Q_DECLARE_METATYPE(QCameraExposureControl::ExposureParameter)

Q_MEDIA_ENUM_DEBUG(QCameraExposureControl, ExposureParameter)


#endif  // QCAMERAEXPOSURECONTROL_H

