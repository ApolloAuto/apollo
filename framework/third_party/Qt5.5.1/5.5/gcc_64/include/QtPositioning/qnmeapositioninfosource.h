/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtPositioning module of the Qt Toolkit.
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
#ifndef QNMEAPOSITIONINFOSOURCE_H
#define QNMEAPOSITIONINFOSOURCE_H

#include <QtPositioning/QGeoPositionInfoSource>

QT_BEGIN_NAMESPACE

class QIODevice;

class QNmeaPositionInfoSourcePrivate;
class Q_POSITIONING_EXPORT QNmeaPositionInfoSource : public QGeoPositionInfoSource
{
    Q_OBJECT
public:
    enum UpdateMode {
        RealTimeMode = 1,
        SimulationMode
    };

    explicit QNmeaPositionInfoSource(UpdateMode updateMode, QObject *parent = 0);
    ~QNmeaPositionInfoSource();

    void setUserEquivalentRangeError(double uere);
    double userEquivalentRangeError() const;

    UpdateMode updateMode() const;

    void setDevice(QIODevice *source);
    QIODevice *device() const;

    void setUpdateInterval(int msec);

    QGeoPositionInfo lastKnownPosition(bool fromSatellitePositioningMethodsOnly = false) const;
    PositioningMethods supportedPositioningMethods() const;
    int minimumUpdateInterval() const;
    Error error() const;


public Q_SLOTS:
    void startUpdates();
    void stopUpdates();
    void requestUpdate(int timeout = 0);

protected:
    virtual bool parsePosInfoFromNmeaData(const char *data,
                                          int size,
                                          QGeoPositionInfo *posInfo,
                                          bool *hasFix);

private:
    Q_DISABLE_COPY(QNmeaPositionInfoSource)
    friend class QNmeaPositionInfoSourcePrivate;
    QNmeaPositionInfoSourcePrivate *d;
    void setError(QGeoPositionInfoSource::Error positionError);
};

QT_END_NAMESPACE

#endif
