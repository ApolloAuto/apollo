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
#ifndef QGEOAREAMONITORSOURCE_H
#define QGEOAREAMONITORSOURCE_H

#include <QtPositioning/QGeoCoordinate>
#include <QtPositioning/QGeoAreaMonitorInfo>
#include <QtPositioning/QGeoPositionInfoSource>

#include <QtCore/QObject>
#include <QtCore/QStringList>

QT_BEGIN_NAMESPACE

class QGeoPositionInfo;
class QGeoAreaMonitorSourcePrivate;
class Q_POSITIONING_EXPORT QGeoAreaMonitorSource : public QObject
{
    Q_OBJECT

public:
    enum Error {
        AccessError = 0,
        InsufficientPositionInfo = 1,
        UnknownSourceError = 2,
        NoError = 3
    };
    Q_ENUMS(Error)

    enum AreaMonitorFeature {
        PersistentAreaMonitorFeature = 0x00000001,
        AnyAreaMonitorFeature = 0xffffffff
    };
    Q_DECLARE_FLAGS(AreaMonitorFeatures, AreaMonitorFeature)

    explicit QGeoAreaMonitorSource(QObject *parent);
    virtual ~QGeoAreaMonitorSource();

    static QGeoAreaMonitorSource *createDefaultSource(QObject *parent);
    static QGeoAreaMonitorSource *createSource(const QString& sourceName, QObject *parent);
    static QStringList availableSources();

    virtual void setPositionInfoSource(QGeoPositionInfoSource *source);
    virtual QGeoPositionInfoSource* positionInfoSource() const;

    QString sourceName() const;

    virtual Error error() const = 0;
    virtual AreaMonitorFeatures supportedAreaMonitorFeatures() const = 0;

    virtual bool startMonitoring(const QGeoAreaMonitorInfo &monitor) = 0;
    virtual bool stopMonitoring(const QGeoAreaMonitorInfo &monitor) = 0;
    virtual bool requestUpdate(const QGeoAreaMonitorInfo &monitor, const char *signal) = 0;

    virtual QList<QGeoAreaMonitorInfo> activeMonitors() const = 0;
    virtual QList<QGeoAreaMonitorInfo> activeMonitors(const QGeoShape &lookupArea) const = 0;

Q_SIGNALS:
    void areaEntered(const QGeoAreaMonitorInfo &monitor, const QGeoPositionInfo &update);
    void areaExited(const QGeoAreaMonitorInfo &monitor, const QGeoPositionInfo &update);
    void monitorExpired(const QGeoAreaMonitorInfo &monitor);
    void error(QGeoAreaMonitorSource::Error error);

private:
    Q_DISABLE_COPY(QGeoAreaMonitorSource)
    QGeoAreaMonitorSourcePrivate *d;
};


QT_END_NAMESPACE

#endif
