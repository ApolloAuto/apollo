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

#ifndef QCAMERAIMAGEPROCESSING_H
#define QCAMERAIMAGEPROCESSING_H

#include <QtCore/qstringlist.h>
#include <QtCore/qpair.h>
#include <QtCore/qsize.h>
#include <QtCore/qpoint.h>
#include <QtCore/qrect.h>

#include <QtMultimedia/qmediacontrol.h>
#include <QtMultimedia/qmediaobject.h>
#include <QtMultimedia/qmediaservice.h>
#include <QtMultimedia/qmediaenumdebug.h>

QT_BEGIN_NAMESPACE


class QCamera;

class QCameraImageProcessingPrivate;
class Q_MULTIMEDIA_EXPORT QCameraImageProcessing : public QObject
{
    Q_OBJECT
    Q_ENUMS(WhiteBalanceMode ColorFilter)
public:
    enum WhiteBalanceMode {
        WhiteBalanceAuto = 0,
        WhiteBalanceManual = 1,
        WhiteBalanceSunlight = 2,
        WhiteBalanceCloudy = 3,
        WhiteBalanceShade = 4,
        WhiteBalanceTungsten = 5,
        WhiteBalanceFluorescent = 6,
        WhiteBalanceFlash = 7,
        WhiteBalanceSunset = 8,
        WhiteBalanceVendor = 1000
    };

    enum ColorFilter {
        ColorFilterNone,
        ColorFilterGrayscale,
        ColorFilterNegative,
        ColorFilterSolarize,
        ColorFilterSepia,
        ColorFilterPosterize,
        ColorFilterWhiteboard,
        ColorFilterBlackboard,
        ColorFilterAqua,
        ColorFilterVendor = 1000
    };

    bool isAvailable() const;

    WhiteBalanceMode whiteBalanceMode() const;
    void setWhiteBalanceMode(WhiteBalanceMode mode);
    bool isWhiteBalanceModeSupported(WhiteBalanceMode mode) const;

    qreal manualWhiteBalance() const;
    void setManualWhiteBalance(qreal colorTemperature);

    qreal contrast() const;
    void setContrast(qreal value);

    qreal saturation() const;
    void setSaturation(qreal value);

    qreal sharpeningLevel() const;
    void setSharpeningLevel(qreal value);

    qreal denoisingLevel() const;
    void setDenoisingLevel(qreal value);

    ColorFilter colorFilter() const;
    void setColorFilter(ColorFilter filter);
    bool isColorFilterSupported(ColorFilter filter) const;

private:
    friend class QCamera;
    friend class QCameraPrivate;
    QCameraImageProcessing(QCamera *camera);
    ~QCameraImageProcessing();

    Q_DISABLE_COPY(QCameraImageProcessing)
    Q_DECLARE_PRIVATE(QCameraImageProcessing)
    QCameraImageProcessingPrivate *d_ptr;
};

QT_END_NAMESPACE

Q_DECLARE_METATYPE(QCameraImageProcessing::WhiteBalanceMode)
Q_DECLARE_METATYPE(QCameraImageProcessing::ColorFilter)

Q_MEDIA_ENUM_DEBUG(QCameraImageProcessing, WhiteBalanceMode)
Q_MEDIA_ENUM_DEBUG(QCameraImageProcessing, ColorFilter)

#endif  // QCAMERAIMAGEPROCESSING_H
